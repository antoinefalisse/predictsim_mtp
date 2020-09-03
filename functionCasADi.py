from sys import path
path.append(r"C:/Users/u0101727/Documents/Software/CasADi/casadi-windows-py37-v3.5.1-64bit")
import casadi as ca
import numpy as np

def polynomialApproximation(musclesPolynomials, polynomialData, NPolynomial):    
    
    from polynomials import polynomials
    
    qin = ca.SX.sym('qin', 1, NPolynomial)
    qdotin  = ca.SX.sym('qdotin', 1, NPolynomial)
    lMT = ca.SX(len(musclesPolynomials), 1)
    vMT = ca.SX(len(musclesPolynomials), 1)
    dM = ca.SX(len(musclesPolynomials), NPolynomial)
    
    for count, musclePolynomials in enumerate(musclesPolynomials):
        
        coefficients = polynomialData[musclePolynomials]['coefficients']
        dimension = polynomialData[musclePolynomials]['dimension']
        order = polynomialData[musclePolynomials]['order']        
        spanning = polynomialData[musclePolynomials]['spanning']          
        
        polynomial = polynomials(coefficients, dimension, order)
        
        idxSpanning = [i for i, e in enumerate(spanning) if e == 1]        
        lMT[count] = polynomial.calcValue(qin[0, idxSpanning])
        
        dM[count, :] = 0
        vMT[count] = 0        
        for i in range(len(idxSpanning)):
            dM[count, idxSpanning[i]] = - polynomial.calcDerivative(
                    qin[0, idxSpanning], i)
            vMT[count] += (-dM[count, idxSpanning[i]] * 
               qdotin[0, idxSpanning[i]])
        
    f_polynomial = ca.Function('f_polynomial',[qin, qdotin],[lMT, vMT, dM])
    
    return f_polynomial
        

def hillEquilibrium(mtParameters, tendonCompliance, specificTension):
    
    NMuscles = mtParameters.shape[1]
    # Function variables
    activation = ca.SX.sym('activation', NMuscles)
    mtLength = ca.SX.sym('mtLength', NMuscles)
    mtVelocity = ca.SX.sym('mtVelocity', NMuscles)
    normTendonForce = ca.SX.sym('normTendonForce', NMuscles)
    normTendonForceDT = ca.SX.sym('normTendonForceDT', NMuscles)
     
    hillEquilibrium = ca.SX(NMuscles, 1)
    tendonForce = ca.SX(NMuscles, 1)
    activeFiberForce = ca.SX(NMuscles, 1)
    normActiveFiberLengthForce = ca.SX(NMuscles, 1)
    passiveFiberForce = ca.SX(NMuscles, 1)
    normFiberLength = ca.SX(NMuscles, 1)
    fiberVelocity = ca.SX(NMuscles, 1)
    
    from muscleModel import muscleModel
    for m in range(NMuscles):    
        muscle = muscleModel(mtParameters[:, m], activation[m], mtLength[m],
                             mtVelocity[m], normTendonForce[m], 
                             normTendonForceDT[m], tendonCompliance[:, m],
                             specificTension[:, m])
        
        hillEquilibrium[m] = muscle.deriveHillEquilibrium()
        tendonForce[m] = muscle.getTendonForce()
        activeFiberForce[m] = muscle.getActiveFiberForce()[0]
        passiveFiberForce[m] = muscle.getPassiveFiberForce()[0]
        normActiveFiberLengthForce[m] = muscle.getActiveFiberLengthForce()
        normFiberLength[m] = muscle.getFiberLength()[1]
        fiberVelocity[m] = muscle.getFiberVelocity()[0]
        
    f_hillEquilibrium = ca.Function('f_hillEquilibrium',
                                    [activation, mtLength, mtVelocity, 
                                     normTendonForce, normTendonForceDT], 
                                     [hillEquilibrium, tendonForce,
                                      activeFiberForce, passiveFiberForce,
                                      normActiveFiberLengthForce,
                                      normFiberLength, fiberVelocity]) 
    
    return f_hillEquilibrium

def armActivationDynamics(NArmJoints):
    t = 0.035 # time constant       
    
    eArm = ca.SX.sym('eArm',NArmJoints)
    aArm = ca.SX.sym('aArm',NArmJoints)
    
    aArmDt = (eArm - aArm) / t
    
    f_armActivationDynamics = ca.Function('f_armActivationDynamics',
                                          [eArm, aArm], [aArmDt])
    
    return f_armActivationDynamics  

def metabolicsBhargava(slowTwitchRatio, maximalIsometricForce,
                       muscleMass, smoothingConstant,
                       use_fiber_length_dep_curve=False,
                       use_force_dependent_shortening_prop_constant=True,
                       include_negative_mechanical_work=False):
    
    NMuscles = maximalIsometricForce.shape[0]
    
    # Function variables
    excitation = ca.SX.sym('excitation', NMuscles)
    activation = ca.SX.sym('activation', NMuscles)
    normFiberLength = ca.SX.sym('normFiberLength', NMuscles)
    fiberVelocity = ca.SX.sym('fiberVelocity', NMuscles)
    activeFiberForce = ca.SX.sym('activeFiberForce', NMuscles)
    passiveFiberForce = ca.SX.sym('passiveFiberForce', NMuscles)
    normActiveFiberLengthForce = (
            ca.SX.sym('normActiveFiberLengthForce', NMuscles))
    
    activationHeatRate = ca.SX(NMuscles, 1)
    maintenanceHeatRate = ca.SX(NMuscles, 1)
    shorteningHeatRate = ca.SX(NMuscles, 1)
    mechanicalWork = ca.SX(NMuscles, 1)
    totalHeatRate = ca.SX(NMuscles, 1) 
    metabolicEnergyRate = ca.SX(NMuscles, 1) 
    slowTwitchExcitation = ca.SX(NMuscles, 1) 
    fastTwitchExcitation = ca.SX(NMuscles, 1) 
    
    from metabolicEnergyModel import smoothBhargava2004
    
    for m in range(NMuscles):   
        metabolics = (smoothBhargava2004(excitation[m], activation[m], 
                                         normFiberLength[m],
                                         fiberVelocity[m],
                                         activeFiberForce[m], 
                                         passiveFiberForce[m],
                                         normActiveFiberLengthForce[m],
                                         slowTwitchRatio[m], 
                                         maximalIsometricForce[m],
                                         muscleMass[m], smoothingConstant))
        
        slowTwitchExcitation[m] = metabolics.getTwitchExcitation()[0] 
        fastTwitchExcitation[m] = metabolics.getTwitchExcitation()[1] 
        activationHeatRate[m] = metabolics.getActivationHeatRate()        
        maintenanceHeatRate[m] = metabolics.getMaintenanceHeatRate(
                use_fiber_length_dep_curve)        
        shorteningHeatRate[m] = metabolics.getShorteningHeatRate(
                use_force_dependent_shortening_prop_constant)        
        mechanicalWork[m] = metabolics.getMechanicalWork(
                include_negative_mechanical_work)        
        totalHeatRate[m] = metabolics.getTotalHeatRate()
        metabolicEnergyRate[m] = metabolics.getMetabolicEnergyRate()
        
#    basal_coef = 1.2 # default in OpenSim
#    basal_exp = 1 # default in OpenSim
#    energyModel = (basal_coef * np.power(modelMass, basal_exp) + 
#                   np.sum(metabolicEnergyRate))
    
    f_metabolicsBhargava = ca.Function('metabolicsBhargava',
                                    [excitation, activation, normFiberLength, 
                                     fiberVelocity, activeFiberForce, 
                                     passiveFiberForce, 
                                     normActiveFiberLengthForce], 
                                     [activationHeatRate, maintenanceHeatRate,
                                      shorteningHeatRate, mechanicalWork, 
                                      totalHeatRate, metabolicEnergyRate])
    
    return f_metabolicsBhargava

def passiveJointTorque(k, theta, d):
    
    # Function variables
    Q = ca.SX.sym('Q', 1)
    Qdot = ca.SX.sym('Qdot', 1)
    
    passiveJointTorque = (k[0] * np.exp(k[1] * (Q - theta[1])) + k[2] * 
                           np.exp(k[3] * (Q - theta[0])) - d * Qdot)
    
    f_passiveJointTorque = ca.Function('f_passiveJointTorque', [Q, Qdot], 
                                       [passiveJointTorque])
    
    return f_passiveJointTorque

def passiveTorqueActuatedJointTorque(k, d):
    # Function variables
    Q = ca.SX.sym('Q', 1)
    Qdot = ca.SX.sym('Qdot', 1)
    
    passiveJointTorque = -k * Q - d * Qdot
    f_passiveMtpTorque = ca.Function('f_passiveMtpTorque', [Q, Qdot], 
                                     [passiveJointTorque])
    
    return f_passiveMtpTorque  

def mySum(N):
    # Function variables
    x = ca.SX.sym('x', 1,  N) 
    mysum = 0;
    for m in range(N):
        mysum += x[0, m]
        
    f_mySum = ca.Function('f_mySum', [x], [mysum])
    
    return f_mySum
    

def normSumPow(N, exp):
    
    # Function variables
    x = ca.SX.sym('x', N,  1)    
    nsp = 0
    
    for m in range(N):
        nsp += x[m, 0]**exp        
    nsp = nsp / N
    
    f_normSumPow = ca.Function('f_normSumPow', [x], [nsp])
    
    return f_normSumPow

def normSumPowDev(N, exp, ref):
    
    # Function variables
    x = ca.SX.sym('x', N,  1)    
    nsp = 0
    
    for m in range(N):
        nsp += (x[m, 0]-ref)**exp        
    nsp = nsp / N
    
    f_normSumPowDev = ca.Function('f_normSumPowDev', [x], [nsp])
    
    return f_normSumPowDev

def sumSqr(N):
    
    x = ca.SX.sym('x', N, 1)
    ss = 0
    for m in range(N):
        ss += x[m, 0]**2
        
    f_sumSqr = ca.Function('f_sumSqr', [x], [ss])
    
    return f_sumSqr

def norm2(N):
    
    x = ca.SX.sym('x', N, 1)
    ss = 0
    for m in range(N):
        ss += x[m, 0]**2
        
    f_sumSqr = ca.Function('f_sumSqr', [x], [np.sqrt(ss)])
    
    return f_sumSqr

def sumProd(N):
    
    x1 = ca.SX.sym('x', N, 1) 
    x2 = ca.SX.sym('x', N, 1) 
    sp = 0
    for m in range(N):
        sp += x1[m, 0] * x2[m, 0]
        
    f_sumProd = ca.Function('f_sumProd', [x1, x2], [sp])
    
    return f_sumProd

def diffTorques():
    
    jointTorque = ca.SX.sym('x', 1) 
    muscleTorque = ca.SX.sym('x', 1) 
    passiveTorque = ca.SX.sym('x', 1)
    
    diffTorque = jointTorque - (muscleTorque + passiveTorque)
    
    f_diffTorques = ca.Function(
            'f_diffTorques', [jointTorque, muscleTorque, passiveTorque], 
            [diffTorque])
        
    return f_diffTorques

def normSqrtDiff(dim):
    
#    x = ca.SX.sym('x', dim, 1) 
#    x_ref = ca.SX.sym('x_ref', dim, 1)     
#    sigma = ca.SX.sym('sigma', dim, 1) 
#    
#    nSD = 0
#    for d in range(dim):
#        nSD += ((x[d, 0] - x_ref[d, 0])/sigma[d, 0])**2
#        
#    f_normSqrtDiff = ca.Function('f_normSqrtDiff', [x, x_ref, sigma], [nSD])
    
    x = ca.SX.sym('x', dim, 1) 
    x_ref = ca.SX.sym('x_ref', dim, 1)  
    
    nSD = 0
    for d in range(dim):
        nSD += ((x[d, 0] - x_ref[d, 0]))**2
    nSD = nSD / dim
        
    f_normSqrtDiff = ca.Function('f_normSqrtDiff', [x, x_ref], [nSD])
    
    return f_normSqrtDiff

def smoothSphereHalfSpaceForce(dissipation, transitionVelocity,
                 staticFriction, dynamicFriction, viscousFriction, normal):
    
    stiffness = ca.SX.sym('stiffness', 1) 
    radius = ca.SX.sym('radius', 1)     
    locSphere_inB = ca.SX.sym('locSphere_inB', 3) 
    posB_inG = ca.SX.sym('posB_inG', 3) 
    lVelB_inG = ca.SX.sym('lVelB_inG', 3) 
    aVelB_inG = ca.SX.sym('aVelB_inG', 3) 
    RBG_inG = ca.SX.sym('RBG_inG', 3, 3) 
    TBG_inG = ca.SX.sym('TBG_inG', 3) 
    
    from contactModel import smoothSphereHalfSpaceForce_ca
    
    contactElement = smoothSphereHalfSpaceForce_ca(stiffness, radius, dissipation,
                                                transitionVelocity,
                                                staticFriction,
                                                dynamicFriction,
                                                viscousFriction, normal)
    
    contactForce = contactElement.getContactForce(locSphere_inB, posB_inG,
                                                  lVelB_inG, aVelB_inG,
                                                  RBG_inG, TBG_inG)
    
    f_smoothSphereHalfSpaceForce = ca.Function(
            'f_smoothSphereHalfSpaceForce',[stiffness, radius, locSphere_inB,
                                            posB_inG, lVelB_inG, aVelB_inG,
                                            RBG_inG, TBG_inG], [contactForce])
    
    return f_smoothSphereHalfSpaceForce    

def muscleMechanicalWorkRate(NMuscles):    
    # Function variables
    fiberVelocity = ca.SX.sym('fiberVelocity', NMuscles)
    activeFiberForce = ca.SX.sym('activeFiberForce', NMuscles)
    
    mechanicalWorkRate = ca.SX(NMuscles, 1)
    
    for m in range(NMuscles):           
        mechanicalWorkRate[m] = -activeFiberForce[m] * fiberVelocity[m]
        
    f_muscleMechanicalWorkRate = ca.Function('f_muscleMechanicalWorkRate',
                                             [activeFiberForce, fiberVelocity],
                                             [mechanicalWorkRate])
    
    return f_muscleMechanicalWorkRate

def jointMechanicalWorkRate(NJoints):    
    # Function variables
    jointVelocity = ca.SX.sym('jointVelocity', NJoints)
    jointTorque = ca.SX.sym('jointTorque', NJoints)
    
    mechanicalWorkRate = ca.SX(NJoints, 1)
    
    for m in range(NJoints):           
        mechanicalWorkRate[m] = jointTorque[m] * jointVelocity[m]
        
    f_jointMechanicalWorkRate = ca.Function('f_jointMechanicalWorkRate',
                                             [jointTorque, jointVelocity],
                                             [mechanicalWorkRate])
    
    return f_jointMechanicalWorkRate    

# Test f_hillEquilibrium
#import numpy as np
#mtParametersT = np.array([[819, 573, 653],
#                 [0.0520776466291754, 0.0823999283675263, 0.0632190293747345],
#                 [0.0759262885434707, 0.0516827953074425, 0.0518670055241629],
#                 [0.139626340000000, 0,	0.331612560000000],
#                 [0.520776466291754,	0.823999283675263,	0.632190293747345]])
#tendonComplianceT = np.array([35, 35, 35])
#tendonShiftT = np.array([0, 0, 0])
#specificTensionT = np.array([0.74455, 0.75395, 0.75057])
#f_hillEquilibrium = hillEquilibrium(mtParametersT, tendonComplianceT,
#                                    tendonShiftT, specificTensionT)
#
#activationT = [0.8, 0.7, 0.6]
#mtLengthT = [1.2, 0.9, 1.3]
#mtVelocity = [0.8, 0.1, 5.4]
#normTendonForce = [0.8, 0.4, 0.9]
#normTendonForceDT = [2.1, 3.4, -5.6]
#
#hillEquilibriumT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity,
#                                     normTendonForce, normTendonForceDT)[0]
#tendonForceT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity,
#                                 normTendonForce, normTendonForceDT)[1]
#activeFiberForceT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity,
#                                      normTendonForce, normTendonForceDT)[2]
#passiveFiberForceT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity,
#                                     normTendonForce, normTendonForceDT)[3]
#normActiveFiberLengthForceT = f_hillEquilibrium(activationT, mtLengthT,
#                                                mtVelocity, normTendonForce,
#                                                normTendonForceDT)[4]
#maximalFiberVelocityT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity,
#                                          normTendonForce, 
#                                          normTendonForceDT)[5]
#muscleMassT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity, 
#                                normTendonForce, normTendonForceDT)[6]
#normFiberLengthT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity, 
#                                     normTendonForce, normTendonForceDT)[7]
#fiberVelocityT = f_hillEquilibrium(activationT, mtLengthT, mtVelocity, 
#                                   normTendonForce, normTendonForceDT)[8]
#
#print(hillEquilibriumT)
#print(tendonForceT)
#print(activeFiberForceT)
#print(passiveFiberForceT)
#print(normActiveFiberLengthForceT)
#print(maximalFiberVelocityT)
#print(muscleMassT)
#print(normFiberLengthT)
#print(fiberVelocityT)

# Test f_armActivationDynamics
#f_armActivationDynamics = armActivationDynamics(3)
#eArmT = [0.8, 0.6, 0.4]
#aArmT = [0.5, 0.4, 0.3]
#aArmDtT = f_armActivationDynamics(eArmT, aArmT)
#
#print(aArmDtT)
    
#import polynomialData
#from polynomials import polynomials
#polynomialData = polynomialData.polynomialData()
#coefficients = polynomialData['glut_med1_r']['coefficients']
#dimension = polynomialData['glut_med1_r']['dimension']
#order = polynomialData['glut_med1_r']['order']        
#spanning = polynomialData['glut_med1_r']['spanning']   
#
#
#polynomial = polynomials(coefficients, dimension, order)
#
#qin     = ca.SX.sym('qin', 1, len(spanning));
##qdotin  = ca.SX.sym('qdotin', 1, len(spanning));
#
#idxSpanning = [i for i, e in enumerate(spanning) if e == 1]
#
#lmT = polynomial.calcValue(qin[0, idxSpanning])
#
#f_polynomial = ca.Function('f_polynomial',[qin],[lmT])
#
#qinT = [0.814483478343008, 1.05503342897057, 0.162384573599574,
#        0.0633034484654646, 0.433004984392647, 0.716775413397760,
#        -0.0299471169706956, 0.200356847296188, 0.716775413397760]
#lmTT = f_polynomial(qinT)
#print(lmTT)
    
#Qin = 5
#Qdotin = 7
#
#passiveJointTorque_hfT = f_passiveJointTorque_hip_flexion(Qin, Qdotin)
#print(passiveJointTorque_hfT)
#passiveJointTorque_haT = f_passiveJointTorque_hip_adduction(Qin, Qdotin)
#print(passiveJointTorque_haT)
#passiveJointTorque_hrT = f_passiveJointTorque_hip_rotation(Qin, Qdotin)
#print(passiveJointTorque_hrT)
#passiveJointTorque_kaT = f_passiveJointTorque_knee_angle(Qin, Qdotin)
#print(passiveJointTorque_kaT)
#passiveJointTorque_aaT = f_passiveJointTorque_ankle_angle(Qin, Qdotin)
#print(passiveJointTorque_aaT)
#passiveJointTorque_saT = f_passiveJointTorque_subtalar_angle(Qin, Qdotin)
#print(passiveJointTorque_saT)
#passiveJointTorque_leT = f_passiveJointTorque_lumbar_extension(Qin, Qdotin)
#print(passiveJointTorque_leT)
#passiveJointTorque_lbT = f_passiveJointTorque_lumbar_bending(Qin, Qdotin)
#print(passiveJointTorque_lbT)
#passiveJointTorque_lrT = f_passiveJointTorque_lumbar_rotation(Qin, Qdotin)
#print(passiveJointTorque_lrT)