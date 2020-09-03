from sys import path
import os
if os.environ['COMPUTERNAME'] == 'GBW-L-W2003':
    path.append(r"C:/Users/u0101727/Documents/Software/CasADi/casadi-windows-py37-v3.5.1-64bit")
    # Workaround to get OpenSim to work with Python
    pathOS = "C:/Users/u0101727/Documents/MyRepositories/opensim-fork/install/sdk/Python"
elif os.environ['COMPUTERNAME'] == 'GBW-D-W0529':
    path.append(r"D:/u0101727/MySoftware/casadi-windows-py37-v3.5.1-64bit")
    # Workaround to get OpenSim to work with Python
    pathOS = "C:/OpenSim_4.1/sdk/Python"
elif os.environ['COMPUTERNAME'] == 'GBW-D-W2711': 
    path.append(r"C:/Users/Public/Documents/Software/casadi-windows-py37-v3.5.1-64bit")
    # Workaround to get OpenSim to work with Python
    pathOS = "C:/OpenSim_4.1/sdk/Python"
import casadi as ca
import numpy as np

solveProblem = False
saveResults = True
analyzeResults = True
loadResults = True
writeMotion = True
saveTrajectories = True
decomposeCost = True
loadMTParameters = True
loadPolynomialData = True
plotPolynomials = False
subject = 'subject1_3D_no_mtp'
model = 'subject1_no_mtp'

cases = ['0', '1','2', '3','6', '7']

from settings_predictsim import getSettings_predictsim_no_mtp   
settings = getSettings_predictsim_no_mtp() 
for case in cases:
    # Weights in cost function
    weights = {'metabolicEnergyRateTerm' : 500,
               'activationTerm': 2000,
               'jointAccelerationTerm': 50000,
               'armExcitationTerm': 1000000, 
               'passiveJointTorqueTerm': 1000, 
               'controls': 0.001}  

    # Other settings
    tol = settings[case]['tol']
    N = settings[case]['N']
    NThreads = 8
    d = 3
    parallelMode = "thread"
    contactConfiguration = settings[case]['contactConfiguration']
    guessType = settings[case]['guessType']
    targetSpeed = settings[case]['targetSpeed']
          
    # Paths
    pathMain = os.getcwd()
    pathData = os.path.join(pathMain, 'OpenSimModel', subject)
    pathModel = os.path.join(pathData, 'Model', model + ".osim")
    pathMTParameters = os.path.join(pathData, 'Model')
    filename = os.path.basename(__file__)
    pathCase = 'Case_' + case    
    pathTrajectories = os.path.join(pathMain, 'Results', filename[:-3]) 
    pathResults = os.path.join(pathTrajectories, pathCase) 
    if not os.path.exists(pathResults):
        os.makedirs(pathResults)
    
    # %% Load external function
    pathExternalFunction = os.path.join(pathMain, 'ExternalFunction')
    os.chdir(pathExternalFunction)
    if contactConfiguration == '0':
        F = ca.external('F','PredSim_no_mtpPin_cm0.dll')
        if analyzeResults:
            F1 = ca.external('F','PredSim_no_mtpPin_pp_cm0.dll')
        os.chdir(pathMain)        
    elif contactConfiguration == '3':
        F = ca.external('F','PredSim_no_mtpPin_cm3.dll')
        if analyzeResults:
            F1 = ca.external('F','PredSim_no_mtpPin_pp_cm3.dll')
        os.chdir(pathMain)
    # vec1 = -np.zeros((87, 1))
    # res1 = (F1(vec1)).full()
    
    # Helper indices
    # origins bodies 
    # Calcaneus
    idxCalcOr_r = list(range(29, 31))
    idxCalcOr_l = list(range(31, 33))
    # Femurs
    idxFemurOr_r = list(range(33, 35))
    idxFemurOr_l = list(range(35, 37))
    # Hands
    idxHandOr_r = list(range(37, 39))
    idxHandOr_l = list(range(39, 41))
    # Tibias
    idxTibiaOr_r = list(range(41, 43))
    idxTibiaOr_l = list(range(43, 45))
    # Toes
    idxToesOr_r = list(range(45, 47))
    idxToesOr_l = list(range(47, 49))
    
    # External function: F1 (post-processing purpose only)
    # Ground reaction forces (GRFs)
    idxGRF_r = list(range(29, 32))
    idxGRF_l = list(range(32, 35))
    idxGRF = idxGRF_r + idxGRF_l
    NGRF = len(idxGRF)
    # Origins calcaneus (3D)
    idxCalcOr3D_r = list(range(35, 38))
    idxCalcOr3D_l = list(range(38, 41))
    idxCalcOr3D = idxCalcOr3D_r + idxCalcOr3D_l
    NCalcOr3D = len(idxCalcOr3D)
    # Number of outputs
    NF1_out = idxCalcOr3D_l[-1] + 1
    
    # %% Muscle-tendon parameters
    muscles = ['glut_med1_r', 'glut_med2_r', 'glut_med3_r', 'glut_min1_r', 
               'glut_min2_r', 'glut_min3_r', 'semimem_r', 'semiten_r', 'bifemlh_r',
               'bifemsh_r', 'sar_r', 'add_long_r', 'add_brev_r', 'add_mag1_r',
               'add_mag2_r', 'add_mag3_r', 'tfl_r', 'pect_r', 'grac_r', 
               'glut_max1_r', 'glut_max2_r', 'glut_max3_r', 'iliacus_r', 'psoas_r',
               'quad_fem_r', 'gem_r', 'peri_r', 'rect_fem_r', 'vas_med_r', 
               'vas_int_r', 'vas_lat_r', 'med_gas_r', 'lat_gas_r', 'soleus_r',
               'tib_post_r', 'flex_dig_r', 'flex_hal_r', 'tib_ant_r', 'per_brev_r',
               'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r', 'ercspn_r', 
               'intobl_r', 'extobl_r', 'ercspn_l', 'intobl_l', 'extobl_l']
    rightSideMuscles = muscles[:-3]
    leftSideMuscles = [muscle[:-1] + 'l' for muscle in rightSideMuscles]
    bothSidesMuscles = leftSideMuscles + rightSideMuscles
    NMuscles = len(bothSidesMuscles)
    NSideMuscles = len(rightSideMuscles)
    
    from muscleData import getMTParameters
    sideMtParameters = getMTParameters(pathOS, pathModel, rightSideMuscles,
                                   loadMTParameters, pathMTParameters)
    mtParameters = np.concatenate((sideMtParameters, sideMtParameters), axis=1)
    
    from muscleData import tendonCompliance
    sideTendonCompliance = tendonCompliance(NSideMuscles)
    tendonCompliance = np.concatenate((sideTendonCompliance, 
                                       sideTendonCompliance), axis=1)
    
    from muscleData import specificTension_3D
    sideSpecificTension = specificTension_3D(rightSideMuscles)
    specificTension = np.concatenate((sideSpecificTension, 
                                      sideSpecificTension), axis=1)
    
    from functionCasADi import hillEquilibrium
    f_hillEquilibrium = hillEquilibrium(mtParameters, tendonCompliance, 
                                        specificTension)
    # Time constants
    activationTimeConstant = 0.015
    deactivationTimeConstant = 0.06
    # Periodicity
    idxPeriodicMuscles = (list(range(NSideMuscles, NMuscles)) + 
                          list(range(0, NSideMuscles)))
    
    # %% Experimental data
    from variousFunctions import getJointIndices    
    joints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 
              'pelvis_ty', 'pelvis_tz', 'hip_flexion_l', 'hip_adduction_l', 
              'hip_rotation_l', 'hip_flexion_r', 'hip_adduction_r', 
              'hip_rotation_r', 'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
              'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r', 
              'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
              'arm_flex_l', 'arm_add_l', 'arm_rot_l', 'arm_flex_r', 'arm_add_r', 
              'arm_rot_r', 'elbow_flex_l', 'elbow_flex_r']
    NJoints = len(joints)
    # Rotational degrees of freedom
    rotationalJoints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 
                        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                        'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
                        'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r', 
                        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
                        'arm_flex_l', 'arm_add_l', 'arm_rot_l', 'arm_flex_r', 
                        'arm_add_r', 'arm_rot_r', 'elbow_flex_l', 'elbow_flex_r']
    idxRotationalJoints = getJointIndices(joints, rotationalJoints)
    # Periodicity
    # Joints whose positions should match after half a gait cycle
    periodicQsJointsA = ['pelvis_tilt', 'pelvis_ty', 
                         'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                         'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                         'knee_angle_l', 'knee_angle_r', 
                         'ankle_angle_l', 'ankle_angle_r', 
                         'subtalar_angle_l', 'subtalar_angle_r', 
                         'lumbar_extension', 
                         'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                         'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                         'elbow_flex_l', 'elbow_flex_r']
    idxPeriodicQsJointsA = getJointIndices(joints, periodicQsJointsA)
    periodicQsJointsB = ['pelvis_tilt', 'pelvis_ty', 
                         'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                         'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                         'knee_angle_r', 'knee_angle_l', 
                         'ankle_angle_r', 'ankle_angle_l', 
                         'subtalar_angle_r', 'subtalar_angle_l', 
                         'lumbar_extension', 
                         'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                         'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                         'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicQsJointsB = getJointIndices(joints, periodicQsJointsB)
    # Joints whose velocities should match after half a gait cycle
    periodicQdotsJointsA = ['pelvis_tilt', 'pelvis_tx', 'pelvis_ty', 
                            'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                            'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                            'knee_angle_l', 'knee_angle_r', 
                            'ankle_angle_l', 'ankle_angle_r', 
                            'subtalar_angle_l', 'subtalar_angle_r', 
                            'lumbar_extension', 
                            'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                            'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                            'elbow_flex_l', 'elbow_flex_r']
    idxPeriodicQdotsJointsA = getJointIndices(joints, periodicQdotsJointsA)
    periodicQdotsJointsB = ['pelvis_tilt', 'pelvis_tx', 'pelvis_ty', 
                            'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                            'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                            'knee_angle_r', 'knee_angle_l', 
                            'ankle_angle_r', 'ankle_angle_l', 
                            'subtalar_angle_r', 'subtalar_angle_l',
                            'lumbar_extension', 
                            'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                            'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                            'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicQdotsJointsB = getJointIndices(joints, periodicQdotsJointsB)
    # Joints whose positions and velocities should be equal and opposite after half 
    # a gait cycle
    periodicOppositeJoints = ['pelvis_list', 'pelvis_rotation', 'pelvis_tz', 
                              'lumbar_bending', 'lumbar_rotation']
    idxPeriodicOppositeJoints = getJointIndices(joints, periodicOppositeJoints)
    # Arm joints
    armJoints = ['arm_flex_l', 'arm_add_l', 'arm_rot_l', 'arm_flex_r', 'arm_add_r', 
                 'arm_rot_r', 'elbow_flex_l', 'elbow_flex_r']
    NArmJoints = len(armJoints)
    idxArmJoints = getJointIndices(joints, armJoints)
    # Arm joints whose activations should match after half a gait cycle
    periodicArmJoints = ['arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                         'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                         'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicArmJoints = getJointIndices(armJoints, periodicArmJoints)
    # All joints per arms    
    noArmJoints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 
              'pelvis_ty', 'pelvis_tz', 'hip_flexion_l', 'hip_adduction_l', 
              'hip_rotation_l', 'hip_flexion_r', 'hip_adduction_r', 
              'hip_rotation_r', 'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
              'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r', 
              'lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    NNoArmJoints = len(noArmJoints)
    idxNoArmJoints = getJointIndices(joints, noArmJoints)
    # Joints with passive torques
    passiveTorqueJoints = ['hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
                           'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
                           'knee_angle_l', 'knee_angle_r', 'ankle_angle_l', 
                           'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r',
                           'lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    NPassiveTorqueJoints = len(passiveTorqueJoints)
    # Ground pelvis joints
    groundPelvisJoints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                          'pelvis_tx', 'pelvis_ty', 'pelvis_tz']
    idxGroundPelvisJoints = getJointIndices(joints, groundPelvisJoints)
    
    GRFNames = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r', 'GRF_x_l','GRF_y_l', 'GRF_z_l']
    periodicGRFA = ['GRF_x_r', 'GRF_y_r', 'GRF_x_l', 'GRF_y_l']
    pidxPeriodicGRFB = ['GRF_x_l', 'GRF_y_l', 'GRF_x_r', 'GRF_y_r']
    idxPeriodicGRFA = getJointIndices(GRFNames, periodicGRFA)
    idxPeriodicGRFB = getJointIndices(GRFNames, pidxPeriodicGRFB)  
    oppositeGRFA = ['GRF_z_r', 'GRF_z_l']
    oppositeGRFB = ['GRF_z_l', 'GRF_z_r']
    idxOppositeGRFA = getJointIndices(GRFNames, oppositeGRFA)
    idxOppositeGRFB = getJointIndices(GRFNames, oppositeGRFB) 
    
    # %% Extract joint positions from average walking motion
    motion_walk = 'walking'
    nametrial_walk_id = 'average_' +  motion_walk + '_HGC_mtp'
    nametrial_walk_IK = 'IK_' + nametrial_walk_id
    pathIK_walk = os.path.join(pathData, 'IK', nametrial_walk_IK + '.mot')
    from variousFunctions import getIK
    Qs_walk_filt = getIK(pathIK_walk, joints)[1]
    
    # %% Arm activation dynamics
    from functionCasADi import armActivationDynamics
    f_armActivationDynamics = armActivationDynamics(NArmJoints)
    
    # %% Polynomials
    from functionCasADi import polynomialApproximation
    leftPolynomialJoints = ['hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l',
                           'knee_angle_l', 'ankle_angle_l', 'subtalar_angle_l',
                           'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'] 
    rightPolynomialJoints = ['hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r',
                             'knee_angle_r', 'ankle_angle_r', 'subtalar_angle_r', 
                             'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'] 
    
    from muscleData import getPolynomialData      
    pathCoordinates = os.path.join(pathData, 'MA', 'dummy_motion.mot')
    pathMuscleAnalysis = os.path.join(pathData, 'MA', 'ResultsMA', 
                                      'subject1', 
                                      'subject01_MuscleAnalysis_') 
    polynomialData = getPolynomialData(loadPolynomialData, pathMTParameters, 
                                       pathCoordinates, pathMuscleAnalysis,
                                       rightPolynomialJoints, muscles)        
    if loadPolynomialData:
        polynomialData = polynomialData.item()
    
    NPolynomial = len(leftPolynomialJoints)
    f_polynomial = polynomialApproximation(muscles, polynomialData, NPolynomial)
    leftPolynomialJointIndices = getJointIndices(joints, leftPolynomialJoints)
    rightPolynomialJointIndices = getJointIndices(joints, rightPolynomialJoints)
    
    leftPolynomialMuscleIndices = list(range(43)) +  list(range(46, 49))
    rightPolynomialMuscleIndices = list(range(46))
    from variousFunctions import getMomentArmIndices
    momentArmIndices = getMomentArmIndices(rightSideMuscles,
                                           leftPolynomialJoints,
                                           rightPolynomialJoints, polynomialData)
    trunkMomentArmPolynomialIndices = list(range(46, 49)) +  list(range(43, 46))
    
    from functionCasADi import sumProd
    f_NHipSumProd = sumProd(len(momentArmIndices['hip_flexion_r']))
    f_NKneeSumProd = sumProd(len(momentArmIndices['knee_angle_r']))
    f_NAnkleSumProd = sumProd(len(momentArmIndices['ankle_angle_r']))
    f_NSubtalarSumProd = sumProd(len(momentArmIndices['subtalar_angle_r']))
    f_NTrunkSumProd = sumProd(len(momentArmIndices['lumbar_extension']))
    
    # Test polynomials
    if plotPolynomials:
        from polynomials import testPolynomials
        momentArms = testPolynomials(pathCoordinates, pathMuscleAnalysis, 
                                     rightPolynomialJoints, muscles, 
                                     f_polynomial, polynomialData, 
                                     momentArmIndices,
                                     trunkMomentArmPolynomialIndices)
    
    # %% Metabolic energy model
    modelMass = 62
    maximalIsometricForce = mtParameters[0, :]
    optimalFiberLength = mtParameters[1, :]
    muscleVolume = np.multiply(maximalIsometricForce, optimalFiberLength)
    muscleMass = np.divide(np.multiply(muscleVolume, 1059.7), 
                           np.multiply(specificTension[0, :].T, 1e6))
    from muscleData import slowTwitchRatio_3D
    sideSlowTwitchRatio = slowTwitchRatio_3D(rightSideMuscles)
    slowTwitchRatio = (np.concatenate((sideSlowTwitchRatio, 
                                      sideSlowTwitchRatio), axis=1))[0, :].T
    smoothingConstant = 10
    from functionCasADi import metabolicsBhargava
    f_metabolicsBhargava = metabolicsBhargava(slowTwitchRatio, 
                                              maximalIsometricForce, muscleMass, 
                                              smoothingConstant)
    
    # %% Passive joint torques
    from functionCasADi import passiveJointTorque
    from muscleData import passiveJointTorqueData_3D
    damping = 0.1
    f_passiveJointTorque_hip_flexion = passiveJointTorque(
            passiveJointTorqueData_3D('hip_flexion_r')[0],
            passiveJointTorqueData_3D('hip_flexion_r')[1], damping)
    f_passiveJointTorque_hip_adduction = passiveJointTorque(
            passiveJointTorqueData_3D('hip_adduction_r')[0],
            passiveJointTorqueData_3D('hip_adduction_r')[1], damping)
    f_passiveJointTorque_hip_rotation = passiveJointTorque(
            passiveJointTorqueData_3D('hip_rotation_r')[0],
            passiveJointTorqueData_3D('hip_rotation_r')[1], damping)
    f_passiveJointTorque_knee_angle = passiveJointTorque(
            passiveJointTorqueData_3D('knee_angle_r')[0],
            passiveJointTorqueData_3D('knee_angle_r')[1], damping)
    f_passiveJointTorque_ankle_angle = passiveJointTorque(
            passiveJointTorqueData_3D('ankle_angle_r')[0],
            passiveJointTorqueData_3D('ankle_angle_r')[1], damping)
    f_passiveJointTorque_subtalar_angle = passiveJointTorque(
            passiveJointTorqueData_3D('subtalar_angle_r')[0],
            passiveJointTorqueData_3D('subtalar_angle_r')[1], damping)
    f_passiveJointTorque_lumbar_extension = passiveJointTorque(
            passiveJointTorqueData_3D('lumbar_extension')[0],
            passiveJointTorqueData_3D('lumbar_extension')[1], damping)
    f_passiveJointTorque_lumbar_bending = passiveJointTorque(
            passiveJointTorqueData_3D('lumbar_bending')[0],
            passiveJointTorqueData_3D('lumbar_bending')[1], damping)
    f_passiveJointTorque_lumbar_rotation = passiveJointTorque(
            passiveJointTorqueData_3D('lumbar_rotation')[0],
            passiveJointTorqueData_3D('lumbar_rotation')[1], damping)
    
    from functionCasADi import passiveTorqueActuatedJointTorque
    stiffnessArm = 0
    dampingArm = 0.1
    f_linearPassiveArmTorque = passiveTorqueActuatedJointTorque(stiffnessArm, 
                                                                dampingArm)
    
    # %% Other helper CasADi functions
    from functionCasADi import normSumPow
    from functionCasADi import diffTorques
    from functionCasADi import sumSqr
    from functionCasADi import mySum
    f_NMusclesSum2 = normSumPow(NMuscles, 2)
    f_NArmJointsSum2 = normSumPow(NArmJoints, 2)
    f_NNoArmJointsSum2 = normSumPow(NNoArmJoints, 2)
    f_NPassiveTorqueJointsSum2 = normSumPow(NPassiveTorqueJoints, 2)
    f_diffTorques = diffTorques()
    f_sumSqr = sumSqr(2)
    f_mySum = mySum(N)
    
    # %% Bounds
    from bounds import bounds
    bounds = bounds(Qs_walk_filt, joints, rightSideMuscles, armJoints, 
                    targetSpeed)
    # Static parameters
    uBoundsFinalTime, lBoundsFinalTime = bounds.getBoundsFinalTime()
    # States
    uBoundsA, lBoundsA, scalingA = bounds.getBoundsActivation()
    uBoundsAk = ca.vec(uBoundsA.to_numpy().T * np.ones((1, N+1))).full()
    lBoundsAk = ca.vec(lBoundsA.to_numpy().T * np.ones((1, N+1))).full()
    uBoundsAj = ca.vec(uBoundsA.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsAj = ca.vec(lBoundsA.to_numpy().T * np.ones((1, d*N))).full()
    
    uBoundsF, lBoundsF, scalingF = bounds.getBoundsForce()
    uBoundsFk = ca.vec(uBoundsF.to_numpy().T * np.ones((1, N+1))).full()
    lBoundsFk = ca.vec(lBoundsF.to_numpy().T * np.ones((1, N+1))).full()
    uBoundsFj = ca.vec(uBoundsF.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsFj = ca.vec(lBoundsF.to_numpy().T * np.ones((1, d*N))).full()
    
    uBoundsQs, lBoundsQs, scalingQs, uBoundsQs0, lBoundsQs0 = (
            bounds.getBoundsPosition())
    
    uBoundsQsk = ca.vec(uBoundsQs.to_numpy().T * np.ones((1, N+1))).full()
    lBoundsQsk = ca.vec(lBoundsQs.to_numpy().T * np.ones((1, N+1))).full()
    uBoundsQsj = ca.vec(uBoundsQs.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsQsj = ca.vec(lBoundsQs.to_numpy().T * np.ones((1, d*N))).full()
    # We want to constraint the pelvis_tx position at the first mesh point. Since
    # it is the first mesh point, the indexing is fine. Not clean I agree.
    lBoundsQsk[joints.index('pelvis_tx')] = lBoundsQs0['pelvis_tx'].to_numpy()
    uBoundsQsk[joints.index('pelvis_tx')] = uBoundsQs0['pelvis_tx'].to_numpy()
    
    uBoundsQdots, lBoundsQdots, scalingQdots = bounds.getBoundsVelocity()
    uBoundsQdotsk = ca.vec(uBoundsQdots.to_numpy().T * np.ones((1, N+1))).full()
    lBoundsQdotsk = ca.vec(lBoundsQdots.to_numpy().T * np.ones((1, N+1))).full()
    uBoundsQdotsj = ca.vec(uBoundsQdots.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsQdotsj = ca.vec(lBoundsQdots.to_numpy().T * np.ones((1, d*N))).full()
    
    uBoundsArmA, lBoundsArmA, scalingArmA = bounds.getBoundsArmActivation()
    uBoundsArmAk = ca.vec(uBoundsArmA.to_numpy().T * np.ones((1, N+1))).full()
    lBoundsArmAk = ca.vec(lBoundsArmA.to_numpy().T * np.ones((1, N+1))).full()
    uBoundsArmAj = ca.vec(uBoundsArmA.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsArmAj = ca.vec(lBoundsArmA.to_numpy().T * np.ones((1, d*N))).full()
    
    # Controls
    uBoundsADt, lBoundsADt, scalingADt = bounds.getBoundsActivationDerivative()
    uBoundsADtk = ca.vec(uBoundsADt.to_numpy().T * np.ones((1, N))).full()
    lBoundsADtk = ca.vec(lBoundsADt.to_numpy().T * np.ones((1, N))).full()
    
    uBoundsArmE, lBoundsArmE, scalingArmE = bounds.getBoundsArmExcitation()
    uBoundsArmEk = ca.vec(uBoundsArmE.to_numpy().T * np.ones((1, N))).full()
    lBoundsArmEk = ca.vec(lBoundsArmE.to_numpy().T * np.ones((1, N))).full()
    
    # Slack controls
    uBoundsQdotdots, lBoundsQdotdots, scalingQdotdots = (
            bounds.getBoundsAcceleration())
    uBoundsQdotdotsj = ca.vec(uBoundsQdotdots.to_numpy().T * 
                              np.ones((1, d*N))).full()
    lBoundsQdotdotsj = ca.vec(lBoundsQdotdots.to_numpy().T *
                              np.ones((1, d*N))).full()
    
    uBoundsFDt, lBoundsFDt, scalingFDt = bounds.getBoundsForceDerivative()
    uBoundsFDtj = ca.vec(uBoundsFDt.to_numpy().T * np.ones((1, d*N))).full()
    lBoundsFDtj = ca.vec(lBoundsFDt.to_numpy().T * np.ones((1, d*N))).full()
    
    # %% Guesses
    if guessType == 'quasiRandom':
        from guesses import quasiRandomGuess
        guess = quasiRandomGuess(N, d, joints, bothSidesMuscles, targetSpeed)
    elif guessType == 'dataDriven':
        from guesses import dataDrivenGuess
        guess = dataDrivenGuess(Qs_walk_filt, N, d, joints, bothSidesMuscles,
                                targetSpeed, periodicQsJointsA, 
                                periodicQdotsJointsA, periodicOppositeJoints)
    # Static parameters
    guessFinalTime = guess.getGuessFinalTime()
    # States
    guessA = guess.getGuessActivation(scalingA)
    guessACol = guess.getGuessActivationCol()
    guessF = guess.getGuessForce(scalingF)
    guessFCol = guess.getGuessForceCol()
    guessQs = guess.getGuessPosition(scalingQs)
    guessQsCol = guess.getGuessPositionCol()
    guessQdots = guess.getGuessVelocity(scalingQdots)
    guessQdotsCol = guess.getGuessVelocityCol()    
    guessArmA = guess.getGuessTorqueActuatorActivation(armJoints)
    guessArmACol = guess.getGuessTorqueActuatorActivationCol(armJoints)
    # Controls
    guessADt = guess.getGuessActivationDerivative(scalingADt)
    guessArmE = guess.getGuessTorqueActuatorExcitation(armJoints)
    # Slack controls
    guessQdotdots = guess.getGuessAcceleration(scalingQdotdots)
    guessQdotdotsCol = guess.getGuessAccelerationCol()
    guessFDt = guess.getGuessForceDerivative(scalingFDt)
    guessFDtCol = guess.getGuessForceDerivativeCol()
    
    # Missing matrix B, add manually
    B = [-8.88178419700125e-16, 0.376403062700467, 0.512485826188421, 
         0.111111111111111]
    
    # %%Formulate ocp
    if solveProblem:
        ###########################################################################
        # Initialize opti instance
        opti = ca.Opti()
        
        ###########################################################################
        # Static parameters
        finalTime = opti.variable()
        opti.subject_to(opti.bounded(lBoundsFinalTime.iloc[0]['time'], finalTime,
                                     uBoundsFinalTime.iloc[0]['time']))
        opti.set_initial(finalTime, guessFinalTime)
        assert lBoundsFinalTime.iloc[0]['time'] <= guessFinalTime, "lb final time"
        assert uBoundsFinalTime.iloc[0]['time'] >= guessFinalTime, "ub final time"
        
        ###########################################################################
        # States
        # Musle activation at mesh points
        a = opti.variable(NMuscles, N+1)
        opti.subject_to(opti.bounded(lBoundsAk, ca.vec(a), uBoundsAk))
        opti.set_initial(a, guessA.to_numpy().T)
        assert np.alltrue(lBoundsAk <= ca.vec(guessA.to_numpy().T).full()), "lb Musle activation"
        assert np.alltrue(uBoundsAk >= ca.vec(guessA.to_numpy().T).full()), "ub Musle activation"
        # Musle activation at collocation points
        a_col = opti.variable(NMuscles, d*N)
        opti.subject_to(opti.bounded(lBoundsAj, ca.vec(a_col), uBoundsAj))
        opti.set_initial(a_col, guessACol.to_numpy().T)
        assert np.alltrue(lBoundsAj <= ca.vec(guessACol.to_numpy().T).full()), "lb Musle activation col"
        assert np.alltrue(uBoundsAj >= ca.vec(guessACol.to_numpy().T).full()), "ub Musle activation col"
        # Musle force at mesh points
        normF = opti.variable(NMuscles, N+1)
        opti.subject_to(opti.bounded(lBoundsFk, ca.vec(normF), uBoundsFk))
        opti.set_initial(normF, guessF.to_numpy().T)
        assert np.alltrue(lBoundsFk <= ca.vec(guessF.to_numpy().T).full()), "lb Musle force"
        assert np.alltrue(uBoundsFk >= ca.vec(guessF.to_numpy().T).full()), "ub Musle force"
        # Musle force at collocation points
        normF_col = opti.variable(NMuscles, d*N)
        opti.subject_to(opti.bounded(lBoundsFj, ca.vec(normF_col), uBoundsFj))
        opti.set_initial(normF_col, guessFCol.to_numpy().T)
        assert np.alltrue(lBoundsFj <= ca.vec(guessFCol.to_numpy().T).full()), "lb Musle force col"
        assert np.alltrue(uBoundsFj >= ca.vec(guessFCol.to_numpy().T).full()), "ub Musle force col"
        # Joint position at mesh points
        Qs = opti.variable(NJoints, N+1)
        opti.subject_to(opti.bounded(lBoundsQsk, ca.vec(Qs), uBoundsQsk))
        opti.set_initial(Qs, guessQs.to_numpy().T)
        if not guessType == 'quasiRandom':
            assert np.alltrue(lBoundsQsk <= ca.vec(guessQs.to_numpy().T).full()), "lb Joint position"
            assert np.alltrue(uBoundsQsk >= ca.vec(guessQs.to_numpy().T).full()), "ub Joint position"
        # Joint position at collocation points
        Qs_col = opti.variable(NJoints, d*N)
        opti.subject_to(opti.bounded(lBoundsQsj, ca.vec(Qs_col), uBoundsQsj))
        opti.set_initial(Qs_col, guessQsCol.to_numpy().T)
        if not guessType == 'quasiRandom':
            assert np.alltrue(lBoundsQsj <= ca.vec(guessQsCol.to_numpy().T).full()), "lb Joint position col"
            assert np.alltrue(uBoundsQsj >= ca.vec(guessQsCol.to_numpy().T).full()), "ub Joint position col"
        # Joint velocity at mesh points
        Qdots = opti.variable(NJoints, N+1)
        opti.subject_to(opti.bounded(lBoundsQdotsk, ca.vec(Qdots), uBoundsQdotsk))
        opti.set_initial(Qdots, guessQdots.to_numpy().T)
        assert np.alltrue(lBoundsQdotsk <= ca.vec(guessQdots.to_numpy().T).full()), "lb Joint velocity"
        assert np.alltrue(uBoundsQdotsk >= ca.vec(guessQdots.to_numpy().T).full()), "ub Joint velocity"
        # Joint velocity at collocation points
        Qdots_col = opti.variable(NJoints, d*N)
        opti.subject_to(opti.bounded(lBoundsQdotsj, ca.vec(Qdots_col), uBoundsQdotsj))
        opti.set_initial(Qdots_col, guessQdotsCol.to_numpy().T)
        assert np.alltrue(lBoundsQdotsj <= ca.vec(guessQdotsCol.to_numpy().T).full()), "lb Joint velocity col"
        assert np.alltrue(uBoundsQdotsj >= ca.vec(guessQdotsCol.to_numpy().T).full()), "ub Joint velocity col"
        # Arm activation at mesh points
        aArm = opti.variable(NArmJoints, N+1)
        opti.subject_to(opti.bounded(lBoundsArmAk, ca.vec(aArm), uBoundsArmAk))
        opti.set_initial(aArm, guessArmA.to_numpy().T)
        assert np.alltrue(lBoundsArmAk <= ca.vec(guessArmA.to_numpy().T).full()), "lb Arm activation"
        assert np.alltrue(uBoundsArmAk >= ca.vec(guessArmA.to_numpy().T).full()), "ub Arm activation"
        # Arm activation at collocation points
        aArm_col = opti.variable(NArmJoints, d*N)
        opti.subject_to(opti.bounded(lBoundsArmAj, ca.vec(aArm_col), uBoundsArmAj))
        opti.set_initial(aArm_col, guessArmACol.to_numpy().T)
        assert np.alltrue(lBoundsArmAj <= ca.vec(guessArmACol.to_numpy().T).full()), "lb Arm activation col"
        assert np.alltrue(uBoundsArmAj >= ca.vec(guessArmACol.to_numpy().T).full()), "ub Arm activation col"
        
        ###########################################################################
        # Controls
        # Muscle activation derivative at mesh points
        aDt = opti.variable(NMuscles, N)
        opti.subject_to(opti.bounded(lBoundsADtk, ca.vec(aDt), uBoundsADtk))
        opti.set_initial(aDt, guessADt.to_numpy().T)
        assert np.alltrue(lBoundsADtk <= ca.vec(guessADt.to_numpy().T).full()), "lb Muscle activation derivative"
        assert np.alltrue(uBoundsADtk >= ca.vec(guessADt.to_numpy().T).full()), "ub Muscle activation derivative"
        # Arm excitation at mesh points
        eArm = opti.variable(NArmJoints, N)
        opti.subject_to(opti.bounded(lBoundsArmEk, ca.vec(eArm), uBoundsArmEk))
        opti.set_initial(eArm, guessArmE.to_numpy().T)
        assert np.alltrue(lBoundsArmEk <= ca.vec(guessArmE.to_numpy().T).full()), "lb Arm excitation"
        assert np.alltrue(uBoundsArmEk >= ca.vec(guessArmE.to_numpy().T).full()), "ub Arm excitation"
        
        ###########################################################################
        # Slack controls
        # Muscle force derivative at collocation points
        normFDt_col = opti.variable(NMuscles, d*N)
        opti.subject_to(opti.bounded(lBoundsFDtj, ca.vec(normFDt_col), uBoundsFDtj))
        opti.set_initial(normFDt_col, guessFDtCol.to_numpy().T)
        assert np.alltrue(lBoundsFDtj <= ca.vec(guessFDtCol.to_numpy().T).full()), "lb Muscle force derivative"
        assert np.alltrue(uBoundsFDtj >= ca.vec(guessFDtCol.to_numpy().T).full()), "ub Muscle force derivative"
        # Joint velocity derivative (acceleration) at collocation points
        Qdotdots_col = opti.variable(NJoints, d*N)
        opti.subject_to(opti.bounded(lBoundsQdotdotsj, ca.vec(Qdotdots_col),
                                     uBoundsQdotdotsj))
        opti.set_initial(Qdotdots_col, guessQdotdotsCol.to_numpy().T)
        assert np.alltrue(lBoundsQdotdotsj <= ca.vec(guessQdotdotsCol.to_numpy().T).full()), "lb Joint velocity derivative"
        assert np.alltrue(uBoundsQdotdotsj >= ca.vec(guessQdotdotsCol.to_numpy().T).full()), "ub Joint velocity derivative"
            
        ###########################################################################   
        # Parallel formulation
        # Static parameters
        tf = ca.MX.sym('tf')
        # States
        ak = ca.MX.sym('ak', NMuscles)
        aj = ca.MX.sym('aj', NMuscles, d)    
        akj = ca.horzcat(ak, aj)    
        normFk = ca.MX.sym('normFk', NMuscles)
        normFj = ca.MX.sym('normFj', NMuscles, d)
        normFkj = ca.horzcat(normFk, normFj)       
        Qsk = ca.MX.sym('Qsk', NJoints)
        Qsj = ca.MX.sym('Qsj', NJoints, d)
        Qskj = ca.horzcat(Qsk, Qsj)    
        Qdotsk = ca.MX.sym('Qdotsk', NJoints)
        Qdotsj = ca.MX.sym('Qdotsj', NJoints, d)
        Qdotskj = ca.horzcat(Qdotsk, Qdotsj)    
        aArmk = ca.MX.sym('aArmk', NArmJoints)
        aArmj = ca.MX.sym('aArmj', NArmJoints, d)
        aArmkj = ca.horzcat(aArmk, aArmj)    
        # Controls
        aDtk = ca.MX.sym('aDtk', NMuscles)    
        eArmk = ca.MX.sym('eArmk', NArmJoints)  
        # Slack controls
        normFDtj = ca.MX.sym('normFDtj', NMuscles, d)
        Qdotdotsj = ca.MX.sym('Qdotdotsj', NJoints, d)
        
        ###########################################################################
        # Time step
        h = tf / N
        
        ###########################################################################
        # Collocation matrices
        tau = ca.collocation_points(d,'radau')
        [C,D] = ca.collocation_interpolators(tau)        
        
        ###########################################################################
        # Initialize cost function and constraint vectors
        J = 0
        eq_constr = []
        ineq_constr1 = []
        ineq_constr2 = []
        ineq_constr3 = []
        ineq_constr4 = []
        ineq_constr5 = [] 
        ineq_constr6 = [] 
            
        ###########################################################################
        # Loop over collocation points
        for j in range(d):
            #######################################################################
            # Unscale variables
            # States
            normFkj_nsc = normFkj * (scalingF.to_numpy().T * np.ones((1, d+1)))
            Qskj_nsc = Qskj * (scalingQs.to_numpy().T * np.ones((1, d+1)))
            Qdotskj_nsc = Qdotskj * (scalingQdots.to_numpy().T * np.ones((1, d+1)))
            # Controls
            aDtk_nsc = aDtk * (scalingADt.to_numpy().T)
            # Slack controls
            normFDtj_nsc = normFDtj * (scalingFDt.to_numpy().T * np.ones((1, d)))
            Qdotdotsj_nsc = Qdotdotsj * (scalingQdotdots.to_numpy().T * 
                                         np.ones((1, d))) 
            # Qs and Qdots are intertwined in external function
            QsQdotskj_nsc = ca.MX(NJoints*2, d+1)
            QsQdotskj_nsc[::2, :] = Qskj_nsc
            QsQdotskj_nsc[1::2, :] = Qdotskj_nsc   
            
            #######################################################################
            # Polynomial approximations
            # Left leg
            Qsinj_l = Qskj_nsc[leftPolynomialJointIndices, j+1]
            Qdotsinj_l = Qdotskj_nsc[leftPolynomialJointIndices, j+1]
            [lMTj_l, vMTj_l, dMj_l] = f_polynomial(Qsinj_l, Qdotsinj_l)        
            dMj_hip_flexion_l = dMj_l[momentArmIndices['hip_flexion_l'] ,0]
            dMj_hip_adduction_l = dMj_l[momentArmIndices['hip_adduction_l'] ,1]
            dMj_hip_rotation_l = dMj_l[momentArmIndices['hip_rotation_l'] ,2]
            dMj_knee_angle_l = dMj_l[momentArmIndices['knee_angle_l'] ,3]
            dMj_ankle_angle_l = dMj_l[momentArmIndices['ankle_angle_l'] ,4]
            dMj_subtalar_angle_l = dMj_l[momentArmIndices['subtalar_angle_l'] ,5]        
            # Right leg
            Qsinj_r = Qskj_nsc[rightPolynomialJointIndices, j+1]
            Qdotsinj_r = Qdotskj_nsc[rightPolynomialJointIndices, j+1]
            [lMTj_r, vMTj_r, dMj_r] = f_polynomial(Qsinj_r, Qdotsinj_r)
            dMj_hip_flexion_r = dMj_r[momentArmIndices['hip_flexion_l'] ,0]
            dMj_hip_adduction_r = dMj_r[momentArmIndices['hip_adduction_l'] ,1]
            dMj_hip_rotation_r = dMj_r[momentArmIndices['hip_rotation_l'] ,2]
            dMj_knee_angle_r = dMj_r[momentArmIndices['knee_angle_l'] ,3]
            dMj_ankle_angle_r = dMj_r[momentArmIndices['ankle_angle_l'] ,4]
            dMj_subtalar_angle_r = dMj_r[momentArmIndices['subtalar_angle_l'] ,5]
            # Trunk
            dMj_lumbar_extension = dMj_l[trunkMomentArmPolynomialIndices, 6]
            dMj_lumbar_bending = dMj_l[trunkMomentArmPolynomialIndices, 7]
            dMj_lumbar_rotation = dMj_l[trunkMomentArmPolynomialIndices, 8]        
            # Both legs        
            lMTj_lr = ca.vertcat(lMTj_l[leftPolynomialMuscleIndices], 
                                 lMTj_r[rightPolynomialMuscleIndices])
            vMTj_lr = ca.vertcat(vMTj_l[leftPolynomialMuscleIndices], 
                                 vMTj_r[rightPolynomialMuscleIndices])
            
            #######################################################################
            # Derive Hill-equilibrium        
            [hillEquilibriumj, Fj, activeFiberForcej, passiveFiberForcej,
             normActiveFiberLengthForcej, normFiberLengthj, fiberVelocityj] = (
             f_hillEquilibrium(akj[:, j+1], lMTj_lr, vMTj_lr, normFkj_nsc[:, j+1], 
                               normFDtj_nsc[:, j]))  
            
            #######################################################################
            # Get metabolic energy rate
            metabolicEnergyRatej = f_metabolicsBhargava(akj[:, j+1], akj[:, j+1], 
                                         normFiberLengthj, fiberVelocityj, 
                                         activeFiberForcej, passiveFiberForcej, 
                                         normActiveFiberLengthForcej)[5]
            
            #######################################################################
            # Get passive joint torques
            passiveJointTorque_hip_flexion_rj = f_passiveJointTorque_hip_flexion(
                    Qskj_nsc[joints.index('hip_flexion_r'), j+1], 
                    Qdotskj_nsc[joints.index('hip_flexion_r'), j+1])
            passiveJointTorque_hip_flexion_lj = f_passiveJointTorque_hip_flexion(
                    Qskj_nsc[joints.index('hip_flexion_l'), j+1], 
                    Qdotskj_nsc[joints.index('hip_flexion_l'), j+1])        
            passiveJointTorque_hip_adduction_rj = (
                    f_passiveJointTorque_hip_adduction(
                    Qskj_nsc[joints.index('hip_adduction_r'), j+1], 
                    Qdotskj_nsc[joints.index('hip_adduction_r'), j+1]))
            passiveJointTorque_hip_adduction_lj = (
                    f_passiveJointTorque_hip_adduction(
                    Qskj_nsc[joints.index('hip_adduction_l'), j+1], 
                    Qdotskj_nsc[joints.index('hip_adduction_l'), j+1]))        
            passiveJointTorque_hip_rotation_rj = f_passiveJointTorque_hip_rotation(
                    Qskj_nsc[joints.index('hip_rotation_r'), j+1], 
                    Qdotskj_nsc[joints.index('hip_rotation_r'), j+1])
            passiveJointTorque_hip_rotation_lj = f_passiveJointTorque_hip_rotation(
                    Qskj_nsc[joints.index('hip_rotation_l'), j+1], 
                    Qdotskj_nsc[joints.index('hip_rotation_l'), j+1])        
            passiveJointTorque_knee_angle_rj = f_passiveJointTorque_knee_angle(
                    Qskj_nsc[joints.index('knee_angle_r'), j+1], 
                    Qdotskj_nsc[joints.index('knee_angle_r'), j+1])
            passiveJointTorque_knee_angle_lj = f_passiveJointTorque_knee_angle(
                    Qskj_nsc[joints.index('knee_angle_l'), j+1], 
                    Qdotskj_nsc[joints.index('knee_angle_l'), j+1])        
            passiveJointTorque_ankle_angle_rj = f_passiveJointTorque_ankle_angle(
                    Qskj_nsc[joints.index('ankle_angle_r'), j+1], 
                    Qdotskj_nsc[joints.index('ankle_angle_r'), j+1])
            passiveJointTorque_ankle_angle_lj = f_passiveJointTorque_ankle_angle(
                    Qskj_nsc[joints.index('ankle_angle_l'), j+1], 
                    Qdotskj_nsc[joints.index('ankle_angle_l'), j+1])        
            passiveJointTorque_subtalar_angle_rj = (
                    f_passiveJointTorque_subtalar_angle(
                    Qskj_nsc[joints.index('subtalar_angle_r'), j+1], 
                    Qdotskj_nsc[joints.index('subtalar_angle_r'), j+1]))
            passiveJointTorque_subtalar_angle_lj = (
                    f_passiveJointTorque_subtalar_angle(
                    Qskj_nsc[joints.index('subtalar_angle_l'), j+1], 
                    Qdotskj_nsc[joints.index('subtalar_angle_l'), j+1]))   
            passiveJointTorque_lumbar_extensionj = (
                    f_passiveJointTorque_lumbar_extension(
                    Qskj_nsc[joints.index('lumbar_extension'), j+1], 
                    Qdotskj_nsc[joints.index('lumbar_extension'), j+1]))        
            passiveJointTorque_lumbar_bendingj = (
                    f_passiveJointTorque_lumbar_bending(
                    Qskj_nsc[joints.index('lumbar_bending'), j+1], 
                    Qdotskj_nsc[joints.index('lumbar_bending'), j+1]))        
            passiveJointTorque_lumbar_rotationj = (
                    f_passiveJointTorque_lumbar_rotation(
                    Qskj_nsc[joints.index('lumbar_rotation'), j+1], 
                    Qdotskj_nsc[joints.index('lumbar_rotation'), j+1])) 
    
            linearPassiveJointTorque_arm_flex_lj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_flex_l'), j+1],
                    Qdotskj_nsc[joints.index('arm_flex_l'), j+1])
            linearPassiveJointTorque_arm_add_lj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_add_l'), j+1],
                    Qdotskj_nsc[joints.index('arm_add_l'), j+1])   
            linearPassiveJointTorque_arm_rot_lj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_rot_l'), j+1],
                    Qdotskj_nsc[joints.index('arm_rot_l'), j+1])
            linearPassiveJointTorque_arm_flex_rj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_flex_r'), j+1],
                    Qdotskj_nsc[joints.index('arm_flex_r'), j+1]) 
            linearPassiveJointTorque_arm_add_rj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_add_r'), j+1],
                    Qdotskj_nsc[joints.index('arm_add_r'), j+1])
            linearPassiveJointTorque_arm_rot_rj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('arm_rot_r'), j+1],
                    Qdotskj_nsc[joints.index('arm_rot_r'), j+1]) 
            linearPassiveJointTorque_elbow_flex_lj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('elbow_flex_l'), j+1],
                    Qdotskj_nsc[joints.index('elbow_flex_l'), j+1]) 
            linearPassiveJointTorque_elbow_flex_rj = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index('elbow_flex_r'), j+1],
                    Qdotskj_nsc[joints.index('elbow_flex_r'), j+1])   
            
            passiveJointTorquesj = ca.vertcat(passiveJointTorque_hip_flexion_rj,
                                              passiveJointTorque_hip_flexion_lj,
                                              passiveJointTorque_hip_adduction_rj,
                                              passiveJointTorque_hip_adduction_lj,
                                              passiveJointTorque_hip_rotation_rj,
                                              passiveJointTorque_hip_rotation_lj,
                                              passiveJointTorque_knee_angle_rj,
                                              passiveJointTorque_knee_angle_lj,
                                              passiveJointTorque_ankle_angle_rj,
                                              passiveJointTorque_ankle_angle_lj,
                                              passiveJointTorque_subtalar_angle_rj,
                                              passiveJointTorque_subtalar_angle_lj,
                                              passiveJointTorque_lumbar_extensionj,
                                              passiveJointTorque_lumbar_bendingj,
                                              passiveJointTorque_lumbar_rotationj)
            #######################################################################
            # Cost function
            metabolicEnergyRateTerm = (f_NMusclesSum2(metabolicEnergyRatej) / 
                                       modelMass)
            activationTerm = f_NMusclesSum2(akj[:, j+1])
            armExcitationTerm = f_NArmJointsSum2(eArmk)      
            jointAccelerationTerm = (
                    f_NNoArmJointsSum2(Qdotdotsj[idxNoArmJoints, j]))                
            passiveJointTorqueTerm = (
                    f_NPassiveTorqueJointsSum2(passiveJointTorquesj))       
            activationDtTerm = f_NMusclesSum2(aDtk)
            forceDtTerm = f_NMusclesSum2(normFDtj[:, j])
            armAccelerationTerm = f_NArmJointsSum2(Qdotdotsj[idxArmJoints, j])
                    
            J += ((weights['metabolicEnergyRateTerm'] * metabolicEnergyRateTerm +
                   weights['activationTerm'] * activationTerm + 
                   weights['armExcitationTerm'] * armExcitationTerm + 
                   weights['jointAccelerationTerm'] * jointAccelerationTerm +                
                   weights['passiveJointTorqueTerm'] * passiveJointTorqueTerm + 
                   weights['controls'] * (forceDtTerm + activationDtTerm 
                          + armAccelerationTerm)) * h * B[j + 1])
            
            #######################################################################
            # Expression for the state derivatives at the collocation points
            ap = ca.mtimes(akj, C[j+1])        
            normFp_nsc = ca.mtimes(normFkj_nsc, C[j+1])
            Qsp_nsc = ca.mtimes(Qskj_nsc, C[j+1])
            Qdotsp_nsc = ca.mtimes(Qdotskj_nsc, C[j+1])        
            aArmp = ca.mtimes(aArmkj, C[j+1])
            # Append collocation equations
            # Muscle activation dynamics (implicit formulation)
            eq_constr.append((h*aDtk_nsc - ap))
            # Muscle contraction dynamics (implicit formulation)  
            eq_constr.append((h*normFDtj_nsc[:, j] - normFp_nsc) / 
                            scalingF.to_numpy().T)
            # Skeleton dynamics (explicit formulation) 
            # Position derivative
            eq_constr.append((h*Qdotskj_nsc[:, j+1] - Qsp_nsc) / 
                            scalingQs.to_numpy().T)
            # Velocity derivative
            eq_constr.append((h*Qdotdotsj_nsc[:, j] - Qdotsp_nsc) / 
                            scalingQdots.to_numpy().T)
            # Arm activation dynamics (implicit formulation) 
            aArmDtj = f_armActivationDynamics(eArmk, aArmkj[:, j+1])
            eq_constr.append(h*aArmDtj - aArmp)
            
            #######################################################################
            # Path constraints        
            # Call external function (run inverse dynamics)
            Tj = F(ca.vertcat(QsQdotskj_nsc[:, j+1], Qdotdotsj_nsc[:, j]))
            # Null pelvis residuals
            eq_constr.append(Tj[idxGroundPelvisJoints, 0])
            
            ######################################################################
            # Muscle-driven joint torques
            # Hip flexion: left
            Fj_hip_flexion_l = Fj[momentArmIndices['hip_flexion_l']] 
            mTj_hip_flexion_l = f_NHipSumProd(dMj_hip_flexion_l, Fj_hip_flexion_l)        
            diffTj_hip_flexion_l = f_diffTorques(
                    Tj[joints.index('hip_flexion_l')], mTj_hip_flexion_l, 
                    passiveJointTorque_hip_flexion_lj)
            eq_constr.append(diffTj_hip_flexion_l)
            # Hip flexion: right
            Fj_hip_flexion_r = Fj[momentArmIndices['hip_flexion_r']]
            mTj_hip_flexion_r = f_NHipSumProd(dMj_hip_flexion_r, Fj_hip_flexion_r)
            diffTj_hip_flexion_r = f_diffTorques(
                    Tj[joints.index('hip_flexion_r')], mTj_hip_flexion_r, 
                    passiveJointTorque_hip_flexion_rj)
            eq_constr.append(diffTj_hip_flexion_r)
            # Hip adduction: left
            Fj_hip_adduction_l = Fj[momentArmIndices['hip_adduction_l']] 
            mTj_hip_adduction_l = f_NHipSumProd(dMj_hip_adduction_l, 
                                                Fj_hip_adduction_l)
            diffTj_hip_adduction_l = f_diffTorques(
                    Tj[joints.index('hip_adduction_l')], mTj_hip_adduction_l, 
                    passiveJointTorque_hip_adduction_lj)
            eq_constr.append(diffTj_hip_adduction_l)
            # Hip adduction: right
            Fj_hip_adduction_r = Fj[momentArmIndices['hip_adduction_r']]
            mTj_hip_adduction_r = f_NHipSumProd(dMj_hip_adduction_r, 
                                                Fj_hip_adduction_r)
            diffTj_hip_adduction_r = f_diffTorques(
                    Tj[joints.index('hip_adduction_r')], mTj_hip_adduction_r, 
                    passiveJointTorque_hip_adduction_rj)
            eq_constr.append(diffTj_hip_adduction_r)
            # Hip rotation: left
            Fj_hip_rotation_l = Fj[momentArmIndices['hip_rotation_l']] 
            mTj_hip_rotation_l = f_NHipSumProd(dMj_hip_rotation_l, 
                                               Fj_hip_rotation_l)
            diffTj_hip_rotation_l = f_diffTorques(
                    Tj[joints.index('hip_rotation_l')], mTj_hip_rotation_l, 
                    passiveJointTorque_hip_rotation_lj)
            eq_constr.append(diffTj_hip_rotation_l)
            # Hip rotation: right
            Fj_hip_rotation_r = Fj[momentArmIndices['hip_rotation_r']]
            mTj_hip_rotation_r = f_NHipSumProd(dMj_hip_rotation_r, 
                                               Fj_hip_rotation_r)
            diffTj_hip_rotation_r = f_diffTorques(
                    Tj[joints.index('hip_rotation_r')], mTj_hip_rotation_r, 
                    passiveJointTorque_hip_rotation_rj)
            eq_constr.append(diffTj_hip_rotation_r)
            # Knee angle: left
            Fj_knee_angle_l = Fj[momentArmIndices['knee_angle_l']] 
            mTj_knee_angle_l = f_NKneeSumProd(dMj_knee_angle_l, Fj_knee_angle_l)
            diffTj_knee_angle_l = f_diffTorques(
                    Tj[joints.index('knee_angle_l')], mTj_knee_angle_l, 
                    passiveJointTorque_knee_angle_lj)
            eq_constr.append(diffTj_knee_angle_l)
            # Knee angle: right
            Fj_knee_angle_r = Fj[momentArmIndices['knee_angle_r']]
            mTj_knee_angle_r = f_NKneeSumProd(dMj_knee_angle_r, Fj_knee_angle_r)
            diffTj_knee_angle_r = f_diffTorques(
                    Tj[joints.index('knee_angle_r')], mTj_knee_angle_r, 
                    passiveJointTorque_knee_angle_rj)
            eq_constr.append(diffTj_knee_angle_r)
            # Ankle angle: left
            Fj_ankle_angle_l = Fj[momentArmIndices['ankle_angle_l']] 
            mTj_ankle_angle_l = f_NAnkleSumProd(dMj_ankle_angle_l, 
                                                Fj_ankle_angle_l)
            diffTj_ankle_angle_l = f_diffTorques(
                    Tj[joints.index('ankle_angle_l')], mTj_ankle_angle_l, 
                    passiveJointTorque_ankle_angle_lj)
            eq_constr.append(diffTj_ankle_angle_l)
            # Ankle angle: right
            Fj_ankle_angle_r = Fj[momentArmIndices['ankle_angle_r']]
            mTj_ankle_angle_r = f_NAnkleSumProd(dMj_ankle_angle_r, 
                                                Fj_ankle_angle_r)
            diffTj_ankle_angle_r = f_diffTorques(
                    Tj[joints.index('ankle_angle_r')], mTj_ankle_angle_r, 
                    passiveJointTorque_ankle_angle_rj)
            eq_constr.append(diffTj_ankle_angle_r)
            # Subtalar angle: left
            Fj_subtalar_angle_l = Fj[momentArmIndices['subtalar_angle_l']] 
            mTj_subtalar_angle_l = f_NSubtalarSumProd(dMj_subtalar_angle_l, 
                                                      Fj_subtalar_angle_l)
            diffTj_subtalar_angle_l = f_diffTorques(
                    Tj[joints.index('subtalar_angle_l')], mTj_subtalar_angle_l, 
                    passiveJointTorque_subtalar_angle_lj)
            eq_constr.append(diffTj_subtalar_angle_l)
            # Subtalar angle: right
            Fj_subtalar_angle_r = Fj[momentArmIndices['subtalar_angle_r']]
            mTj_subtalar_angle_r = f_NSubtalarSumProd(dMj_subtalar_angle_r, 
                                                      Fj_subtalar_angle_r)
            diffTj_subtalar_angle_r = f_diffTorques(
                    Tj[joints.index('subtalar_angle_r')], mTj_subtalar_angle_r, 
                    passiveJointTorque_subtalar_angle_rj)
            eq_constr.append(diffTj_subtalar_angle_r)
            # Trunk extension
            Fj_lumbar_extension = Fj[momentArmIndices['lumbar_extension']]      
            mTj_lumbar_extension = f_NTrunkSumProd(dMj_lumbar_extension, 
                                                   Fj_lumbar_extension)
            diffTj_lumbar_extension = f_diffTorques(
                    Tj[joints.index('lumbar_extension')], mTj_lumbar_extension, 
                    passiveJointTorque_lumbar_extensionj)
            eq_constr.append(diffTj_lumbar_extension)
            # Trunk bending
            Fj_lumbar_bending = Fj[momentArmIndices['lumbar_bending']] 
            mTj_lumbar_bending = f_NTrunkSumProd(dMj_lumbar_bending, 
                                                 Fj_lumbar_bending)
            diffTj_lumbar_bending = f_diffTorques(
                    Tj[joints.index('lumbar_bending')], mTj_lumbar_bending, 
                    passiveJointTorque_lumbar_bendingj)
            eq_constr.append(diffTj_lumbar_bending)
            # Trunk rotation
            Fj_lumbar_rotation = Fj[momentArmIndices['lumbar_rotation']]  
            mTj_lumbar_rotation = f_NTrunkSumProd(dMj_lumbar_rotation, 
                                                  Fj_lumbar_rotation)
            diffTj_lumbar_rotation = f_diffTorques(
                    Tj[joints.index('lumbar_rotation')], mTj_lumbar_rotation, 
                    passiveJointTorque_lumbar_rotationj)
            eq_constr.append(diffTj_lumbar_rotation)
            
            #######################################################################
            # Torque-driven joint torques (arm joints)       
            diffTJ_arm_flex_l = f_diffTorques(
                    Tj[joints.index('arm_flex_l')] /
                    scalingArmE.iloc[0]['arm_flex_l'],
                    aArmkj[0, j+1], linearPassiveJointTorque_arm_flex_lj /
                    scalingArmE.iloc[0]['arm_flex_l'])
            eq_constr.append(diffTJ_arm_flex_l)
            diffTJ_arm_add_l = f_diffTorques(
                    Tj[joints.index('arm_add_l')] /
                    scalingArmE.iloc[0]['arm_add_l'],
                    aArmkj[1, j+1], linearPassiveJointTorque_arm_add_lj /
                    scalingArmE.iloc[0]['arm_add_l'])
            eq_constr.append(diffTJ_arm_add_l)
            diffTJ_arm_rot_l = f_diffTorques(
                    Tj[joints.index('arm_rot_l')] /
                    scalingArmE.iloc[0]['arm_rot_l'], 
                    aArmkj[2, j+1], linearPassiveJointTorque_arm_rot_lj / 
                    scalingArmE.iloc[0]['arm_rot_l'])
            eq_constr.append(diffTJ_arm_rot_l)
            diffTJ_arm_flex_r = f_diffTorques(
                    Tj[joints.index('arm_flex_r')] / 
                    scalingArmE.iloc[0]['arm_flex_r'],
                    aArmkj[3, j+1], linearPassiveJointTorque_arm_flex_rj / 
                    scalingArmE.iloc[0]['arm_flex_r'])
            eq_constr.append(diffTJ_arm_flex_r)
            diffTJ_arm_add_r = f_diffTorques(
                    Tj[joints.index('arm_add_r')] / 
                    scalingArmE.iloc[0]['arm_add_r'],
                    aArmkj[4, j+1], linearPassiveJointTorque_arm_add_rj / 
                    scalingArmE.iloc[0]['arm_add_r'])
            eq_constr.append(diffTJ_arm_add_r)
            diffTJ_arm_rot_r = f_diffTorques(
                    Tj[joints.index('arm_rot_r')] / 
                    scalingArmE.iloc[0]['arm_rot_r'],
                    aArmkj[5, j+1], linearPassiveJointTorque_arm_rot_rj / 
                    scalingArmE.iloc[0]['arm_rot_r'])
            eq_constr.append(diffTJ_arm_rot_r)
            diffTJ_elbow_flex_l = f_diffTorques(
                    Tj[joints.index('elbow_flex_l')] / 
                    scalingArmE.iloc[0]['elbow_flex_l'],
                    aArmkj[6, j+1], linearPassiveJointTorque_elbow_flex_lj / 
                    scalingArmE.iloc[0]['elbow_flex_l'])
            eq_constr.append(diffTJ_elbow_flex_l)
            diffTJ_elbow_flex_r = f_diffTorques(
                    Tj[joints.index('elbow_flex_r')] / 
                    scalingArmE.iloc[0]['elbow_flex_r'],
                    aArmkj[7, j+1], linearPassiveJointTorque_elbow_flex_rj / 
                    scalingArmE.iloc[0]['elbow_flex_r'])
            eq_constr.append(diffTJ_elbow_flex_r)     
            
            #######################################################################
            # Activation dynamics (implicit formulation)
            act1 = aDtk_nsc + akj[:, j+1] / deactivationTimeConstant
            act2 = aDtk_nsc + akj[:, j+1] / activationTimeConstant
            ineq_constr1.append(act1)
            ineq_constr2.append(act2)
            
            #######################################################################
            # Contraction dynamics (implicit formulation)
            eq_constr.append(hillEquilibriumj)
            
            #######################################################################
            # Prevent inter-penetrations of body parts
            diffCalcOrs = f_sumSqr(Tj[idxCalcOr_r] - Tj[idxCalcOr_l])
            ineq_constr3.append(diffCalcOrs)
            diffFemurHandOrs_r = f_sumSqr(Tj[idxFemurOr_r] - Tj[idxHandOr_r])
            ineq_constr4.append(diffFemurHandOrs_r)
            diffFemurHandOrs_l = f_sumSqr(Tj[idxFemurOr_l] - Tj[idxHandOr_l])
            ineq_constr4.append(diffFemurHandOrs_l)
            diffTibiaOrs = f_sumSqr(Tj[idxTibiaOr_r] - Tj[idxTibiaOr_l])
            ineq_constr5.append(diffTibiaOrs)
            diffToesOrs = f_sumSqr(Tj[idxToesOr_r] - Tj[idxToesOr_l])
            ineq_constr6.append(diffToesOrs)
        # End loop over collocation points
        
        ###########################################################################
        # Flatten constraint vectors
        eq_constr = ca.vertcat(*eq_constr)
        ineq_constr1 = ca.vertcat(*ineq_constr1)
        ineq_constr2 = ca.vertcat(*ineq_constr2)
        ineq_constr3 = ca.vertcat(*ineq_constr3)
        ineq_constr4 = ca.vertcat(*ineq_constr4)
        ineq_constr5 = ca.vertcat(*ineq_constr5)
        ineq_constr6 = ca.vertcat(*ineq_constr6)
        # Create function for map construct (parallel computing)
        f_coll = ca.Function('f_coll', [tf, ak, aj, normFk, normFj, Qsk, 
                                        Qsj, Qdotsk, Qdotsj, aArmk, aArmj,
                                        aDtk, eArmk, normFDtj, Qdotdotsj], 
                [eq_constr, ineq_constr1, ineq_constr2, ineq_constr3, 
                 ineq_constr4, ineq_constr5, ineq_constr6, J])     
        # Create map construct
        f_coll_map = f_coll.map(N, parallelMode, NThreads)   
        # Call function with opti variables and set constraints
        (coll_eq_constr, coll_ineq_constr1, coll_ineq_constr2, coll_ineq_constr3,
         coll_ineq_constr4, coll_ineq_constr5, coll_ineq_constr6, Jall) = (
                 f_coll_map(finalTime, a[:, :-1], a_col, normF[:, :-1], normF_col, 
                            Qs[:, :-1], Qs_col, Qdots[:, :-1], Qdots_col, 
                            aArm[:, :-1], aArm_col, aDt, eArm,normFDt_col,
                            Qdotdots_col))       
        opti.subject_to(ca.vec(coll_eq_constr) == 0)
        opti.subject_to(ca.vec(coll_ineq_constr1) >= 0)
        opti.subject_to(ca.vec(coll_ineq_constr2) <= 1 / activationTimeConstant)    
        opti.subject_to(opti.bounded(0.0081, ca.vec(coll_ineq_constr3), 4))
        opti.subject_to(opti.bounded(0.0324 , ca.vec(coll_ineq_constr4), 4))
        opti.subject_to(opti.bounded(0.0121, ca.vec(coll_ineq_constr5), 4))
        opti.subject_to(opti.bounded(0.01, ca.vec(coll_ineq_constr6), 4))
                
        ###########################################################################
        # Equality / continuity constraints
        # Loop over mesh points
        for k in range(N):
            akj2 = (ca.horzcat(a[:, k], a_col[:, k*d:(k+1)*d]))
            normFkj2 = (ca.horzcat(normF[:, k], normF_col[:, k*d:(k+1)*d]))
            Qskj2 = (ca.horzcat(Qs[:, k], Qs_col[:, k*d:(k+1)*d]))
            Qdotskj2 = (ca.horzcat(Qdots[:, k], Qdots_col[:, k*d:(k+1)*d]))    
            aArmkj2 = (ca.horzcat(aArm[:, k], aArm_col[:, k*d:(k+1)*d]))
            
            opti.subject_to(a[:, k+1] == ca.mtimes(akj2, D))
            opti.subject_to(normF[:, k+1] == ca.mtimes(normFkj2, D))    
            opti.subject_to(Qs[:, k+1] == ca.mtimes(Qskj2, D))
            opti.subject_to(Qdots[:, k+1] == ca.mtimes(Qdotskj2, D))    
            opti.subject_to(aArm[:, k+1] == ca.mtimes(aArmkj2, D))  
            
        ###########################################################################
        # Periodic constraints 
        # Joint positions and velocities
        opti.subject_to(Qs[idxPeriodicQsJointsA ,-1] - 
                        Qs[idxPeriodicQsJointsB, 0] == 0)
        opti.subject_to(Qdots[idxPeriodicQdotsJointsA ,-1] - 
                        Qdots[idxPeriodicQdotsJointsB, 0] == 0)
        opti.subject_to(Qs[idxPeriodicOppositeJoints ,-1] + 
                        Qs[idxPeriodicOppositeJoints, 0] == 0)
        opti.subject_to(Qdots[idxPeriodicOppositeJoints ,-1] + 
                        Qdots[idxPeriodicOppositeJoints, 0] == 0)
        # Muscle activations
        opti.subject_to(a[:, -1] - a[idxPeriodicMuscles, 0] == 0)
        # Muscle force
        opti.subject_to(normF[:, -1] - normF[idxPeriodicMuscles, 0] == 0)
        # Arm activations
        opti.subject_to(aArm[:, -1] - aArm[idxPeriodicArmJoints, 0] == 0)
        
        ###########################################################################
        # Average speed constraint
        Qs_nsc = Qs * (scalingQs.to_numpy().T * np.ones((1, N+1)))
        distTraveled =  (Qs_nsc[joints.index('pelvis_tx'), -1] - 
                                Qs_nsc[joints.index('pelvis_tx'), 0])
        averageSpeed = distTraveled / finalTime
        opti.subject_to(averageSpeed - targetSpeed == 0)
        
        ###########################################################################
        # Scale cost function with distance traveled
        Jall_sc = f_mySum(Jall)/distTraveled  
        
        ###########################################################################
        # Create NLP solver
        opti.minimize(Jall_sc)
                
        ###########################################################################
        # Solve problem
        from variousFunctions import solve_with_bounds
        w_opt, stats = solve_with_bounds(opti, tol)
        if saveResults:               
            np.save(os.path.join(pathResults, 'w_opt.npy'), w_opt)
            np.save(os.path.join(pathResults, 'stats.npy'), stats)
        
    # %% Analyze results
    if analyzeResults:
        if loadResults:
            w_opt = np.load(os.path.join(pathResults, 'w_opt.npy'))
            stats = np.load(os.path.join(pathResults, 'stats.npy'), 
                            allow_pickle=True).item()  
        NParameters = 1    
        finalTime_opt = w_opt[:NParameters]
        starti = NParameters    
        a_opt = (np.reshape(w_opt[starti:starti+NMuscles*(N+1)],
                                  (N+1, NMuscles))).T
        starti = starti + NMuscles*(N+1)
        a_col_opt = (np.reshape(w_opt[starti:starti+NMuscles*(d*N)],
                                      (d*N, NMuscles))).T    
        starti = starti + NMuscles*(d*N)
        normF_opt = (np.reshape(w_opt[starti:starti+NMuscles*(N+1)],
                                      (N+1, NMuscles))  ).T  
        starti = starti + NMuscles*(N+1)
        normF_col_opt = (np.reshape(w_opt[starti:starti+NMuscles*(d*N)],
                                          (d*N, NMuscles))).T
        starti = starti + NMuscles*(d*N)
        Qs_opt = (np.reshape(w_opt[starti:starti+NJoints*(N+1)],
                                   (N+1, NJoints))  ).T  
        starti = starti + NJoints*(N+1)    
        Qs_col_opt = (np.reshape(w_opt[starti:starti+NJoints*(d*N)],
                                       (d*N, NJoints))).T
        starti = starti + NJoints*(d*N)
        Qdots_opt = (np.reshape(w_opt[starti:starti+NJoints*(N+1)],
                                      (N+1, NJoints)) ).T   
        starti = starti + NJoints*(N+1)    
        Qdots_col_opt = (np.reshape(w_opt[starti:starti+NJoints*(d*N)],
                                          (d*N, NJoints))).T
        starti = starti + NJoints*(d*N)    
        aArm_opt = (np.reshape(w_opt[starti:starti+NArmJoints*(N+1)],
                                     (N+1, NArmJoints))).T
        starti = starti + NArmJoints*(N+1)    
        aArm_col_opt = (np.reshape(w_opt[starti:starti+NArmJoints*(d*N)],
                                         (d*N, NArmJoints))).T
        starti = starti + NArmJoints*(d*N)                 
        aDt_opt = (np.reshape(w_opt[starti:starti+NMuscles*N],
                              (N, NMuscles))).T
        starti = starti + NMuscles*N
        eArm_opt = (np.reshape(w_opt[starti:starti+NArmJoints*N],
                               (N, NArmJoints))).T
        starti = starti + NArmJoints*N     
        normFDt_col_opt = (np.reshape(w_opt[starti:starti+NMuscles*(d*N)],
                                            (d*N, NMuscles))).T
        starti = starti + NMuscles*(d*N)
        Qdotdots_col_opt = (np.reshape(w_opt[starti:starti+NJoints*(d*N)],
                                             (d*N, NJoints))).T
        starti = starti + NJoints*(d*N)
        assert (starti == w_opt.shape[0]), "error when extracting results"
            
        # %% Unscale results
        normF_opt_nsc = normF_opt * (scalingF.to_numpy().T * np.ones((1, N+1)))
        normF_col_opt_nsc = normF_col_opt * (scalingF.to_numpy().T * 
                                             np.ones((1, d*N)))    
        Qs_opt_nsc = Qs_opt * (scalingQs.to_numpy().T * np.ones((1, N+1)))
        Qs_col_opt_nsc = Qs_col_opt * (scalingQs.to_numpy().T * 
                                       np.ones((1, d*N)))
        Qdots_opt_nsc = Qdots_opt * (scalingQdots.to_numpy().T * 
                                     np.ones((1, N+1)))
        Qdots_col_opt_nsc = Qdots_col_opt * (scalingQdots.to_numpy().T * 
                                             np.ones((1, d*N)))
        aDt_opt_nsc = aDt_opt * (scalingADt.to_numpy().T * np.ones((1, N)))
        Qdotdots_col_opt_nsc = Qdotdots_col_opt * (scalingQdotdots.to_numpy().T * 
                                                   np.ones((1, d*N)))
        normFDt_col_opt_nsc = normFDt_col_opt * (scalingFDt.to_numpy().T * 
                                                 np.ones((1, d*N)))
        normFDt_opt_nsc = normFDt_col_opt_nsc[:,d-1::d]
        # Assert speed
        distTraveled_opt = (Qs_opt_nsc[joints.index('pelvis_tx'), -1] - 
                                       Qs_opt_nsc[joints.index('pelvis_tx'), 0])
        averageSpeed = distTraveled_opt / finalTime_opt
        assert (np.abs(averageSpeed - targetSpeed) < 10**(-4)), "error in speed"
        
        # %% Extract passive joint torques     
        linearPassiveJointTorque_arm_flex_l_opt = np.zeros((1, N+1))
        linearPassiveJointTorque_arm_flex_r_opt = np.zeros((1, N+1))        
        linearPassiveJointTorque_arm_add_l_opt = np.zeros((1, N+1))
        linearPassiveJointTorque_arm_add_r_opt = np.zeros((1, N+1))        
        linearPassiveJointTorque_arm_rot_l_opt = np.zeros((1, N+1))
        linearPassiveJointTorque_arm_rot_r_opt = np.zeros((1, N+1))        
        linearPassiveJointTorque_elbow_flex_l_opt = np.zeros((1, N+1))
        linearPassiveJointTorque_elbow_flex_r_opt = np.zeros((1, N+1))
        
        for k in range(N+1):          
            linearPassiveJointTorque_arm_flex_l_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_flex_l'), k],
                    Qdots_opt_nsc[joints.index('arm_flex_l'), k])
            linearPassiveJointTorque_arm_flex_r_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_flex_r'), k],
                    Qdots_opt_nsc[joints.index('arm_flex_r'), k])            
            linearPassiveJointTorque_arm_add_l_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_add_l'), k],
                    Qdots_opt_nsc[joints.index('arm_add_l'), k])
            linearPassiveJointTorque_arm_add_r_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_add_r'), k],
                    Qdots_opt_nsc[joints.index('arm_add_r'), k])            
            linearPassiveJointTorque_arm_rot_l_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_rot_l'), k],
                    Qdots_opt_nsc[joints.index('arm_rot_l'), k])
            linearPassiveJointTorque_arm_rot_r_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('arm_rot_r'), k],
                    Qdots_opt_nsc[joints.index('arm_rot_r'), k])            
            linearPassiveJointTorque_elbow_flex_l_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('elbow_flex_l'), k],
                    Qdots_opt_nsc[joints.index('elbow_flex_l'), k])
            linearPassiveJointTorque_elbow_flex_r_opt[0, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index('elbow_flex_r'), k],
                    Qdots_opt_nsc[joints.index('elbow_flex_r'), k])
            
        # %% Extract joint torques and ground reaction forces
        QsQdots_opt_nsc = np.zeros((NJoints*2, N+1))
        QsQdots_opt_nsc[::2, :] = Qs_opt_nsc
        QsQdots_opt_nsc[1::2, :] = Qdots_opt_nsc
        Qdotdots_opt = Qdotdots_col_opt_nsc[:,d-1::d]
        F1_out = np.zeros((NF1_out, N))
        armT = np.zeros((NArmJoints, N))
        for k in range(N):    
            Tj = F1(ca.vertcat(QsQdots_opt_nsc[:, k+1], Qdotdots_opt[:, k]))
            F1_out[:, k] = Tj.full().T                             
            armT[0, k] = f_diffTorques(F1_out[joints.index('arm_flex_l'), k] / 
                    scalingArmE.iloc[0]['arm_flex_l'], aArm_opt[0, k+1], 
                    linearPassiveJointTorque_arm_flex_l_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_flex_l'])                 
            armT[1, k] = f_diffTorques(F1_out[joints.index('arm_add_l'), k] / 
                    scalingArmE.iloc[0]['arm_add_l'], aArm_opt[1, k+1], 
                    linearPassiveJointTorque_arm_add_l_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_add_l'])              
            armT[2, k] = f_diffTorques(F1_out[joints.index('arm_rot_l'), k] / 
                    scalingArmE.iloc[0]['arm_rot_l'], aArm_opt[2, k+1], 
                    linearPassiveJointTorque_arm_rot_l_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_rot_l'])              
            armT[3, k] = f_diffTorques(F1_out[joints.index('arm_flex_r'), k] / 
                    scalingArmE.iloc[0]['arm_flex_r'], aArm_opt[3, k+1], 
                    linearPassiveJointTorque_arm_flex_r_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_flex_r'])              
            armT[4, k] = f_diffTorques(F1_out[joints.index('arm_add_r'), k] / 
                    scalingArmE.iloc[0]['arm_add_r'], aArm_opt[4, k+1], 
                    linearPassiveJointTorque_arm_add_r_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_add_r'])             
            armT[5, k] = f_diffTorques(F1_out[joints.index('arm_rot_r'), k] / 
                    scalingArmE.iloc[0]['arm_rot_r'], aArm_opt[5, k+1], 
                    linearPassiveJointTorque_arm_rot_r_opt[0, k+1] / 
                    scalingArmE.iloc[0]['arm_rot_r'])              
            armT[6, k] = f_diffTorques(F1_out[joints.index('elbow_flex_l'), k]/ 
                    scalingArmE.iloc[0]['elbow_flex_l'], aArm_opt[6, k+1], 
                    linearPassiveJointTorque_elbow_flex_l_opt[0, k+1] / 
                    scalingArmE.iloc[0]['elbow_flex_l'])             
            armT[7, k] = f_diffTorques(F1_out[joints.index('elbow_flex_r'), k]/ 
                    scalingArmE.iloc[0]['elbow_flex_r'], aArm_opt[7, k+1], 
                    linearPassiveJointTorque_elbow_flex_r_opt[0, k+1] / 
                    scalingArmE.iloc[0]['elbow_flex_r'])                 
        GRF_opt = F1_out[idxGRF, :]
        torques_opt = F1_out[getJointIndices(joints, joints), :] 
        # Assert arm torques    
        assert np.alltrue(np.abs(armT) < 10**(-8)), "error in arm torques"  
        
        # %% Decompose cost
        if decomposeCost:     
            metabolicEnergyRateTerm_opt_all = 0
            activationTerm_opt_all = 0
            armExcitationTerm_opt_all = 0
            jointAccelerationTerm_opt_all = 0
            passiveJointTorqueTerm_opt_all = 0
            activationDtTerm_opt_all = 0
            forceDtTerm_opt_all = 0
            armAccelerationTerm_opt_all = 0
            h_opt = finalTime_opt / N
            for k in range(N):
                # States 
                akj_opt = (ca.horzcat(a_opt[:, k], a_col_opt[:, k*d:(k+1)*d]))
                normFkj_opt = (ca.horzcat(normF_opt[:, k], normF_col_opt[:, k*d:(k+1)*d]))
                normFkj_opt_nsc = normFkj_opt * (scalingF.to_numpy().T * np.ones((1, d+1)))   
                Qskj_opt = (ca.horzcat(Qs_opt[:, k], Qs_col_opt[:, k*d:(k+1)*d]))
                Qskj_opt_nsc = Qskj_opt * (scalingQs.to_numpy().T * np.ones((1, d+1)))
                Qdotskj_opt = (ca.horzcat(Qdots_opt[:, k], Qdots_col_opt[:, k*d:(k+1)*d]))
                Qdotskj_opt_nsc = Qdotskj_opt * (scalingQdots.to_numpy().T * np.ones((1, d+1)))
                # Controls
                aDtk_opt = aDt_opt[:, k]
                aDtk_opt_nsc = aDt_opt_nsc[:, k]
                eArmk_opt = eArm_opt[:, k]
                # Slack controls
                Qdotdotsj_opt = Qdotdots_col_opt[:, k*d:(k+1)*d]
                Qdotdotsj_opt_nsc = Qdotdotsj_opt * (scalingQdotdots.to_numpy().T * np.ones((1, d)))
                normFDtj_opt = normFDt_col_opt[:, k*d:(k+1)*d] 
                normFDtj_opt_nsc = normFDtj_opt * (scalingFDt.to_numpy().T * np.ones((1, d)))
            
                QsQdotskj_opt_nsc = ca.DM(NJoints*2, d+1)
                QsQdotskj_opt_nsc[::2, :] = Qskj_opt_nsc
                QsQdotskj_opt_nsc[1::2, :] = Qdotskj_opt_nsc
                
                for j in range(d):                                 
                    passiveJointTorque_hip_flexion_rj_opt = f_passiveJointTorque_hip_flexion(
                            Qskj_opt_nsc[joints.index('hip_flexion_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_flexion_r'), j+1])
                    passiveJointTorque_hip_flexion_lj_opt = f_passiveJointTorque_hip_flexion(
                            Qskj_opt_nsc[joints.index('hip_flexion_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_flexion_l'), j+1])   
                    passiveJointTorque_hip_adduction_rj_opt = f_passiveJointTorque_hip_adduction(
                            Qskj_opt_nsc[joints.index('hip_adduction_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_adduction_r'), j+1])
                    passiveJointTorque_hip_adduction_lj_opt = f_passiveJointTorque_hip_adduction(
                            Qskj_opt_nsc[joints.index('hip_adduction_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_adduction_l'), j+1])   
                    passiveJointTorque_hip_rotation_rj_opt = f_passiveJointTorque_hip_rotation(
                            Qskj_opt_nsc[joints.index('hip_rotation_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_rotation_r'), j+1])
                    passiveJointTorque_hip_rotation_lj_opt = f_passiveJointTorque_hip_rotation(
                            Qskj_opt_nsc[joints.index('hip_rotation_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('hip_rotation_l'), j+1])   
                    passiveJointTorque_knee_angle_rj_opt = f_passiveJointTorque_knee_angle(
                            Qskj_opt_nsc[joints.index('knee_angle_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('knee_angle_r'), j+1])
                    passiveJointTorque_knee_angle_lj_opt = f_passiveJointTorque_knee_angle(
                            Qskj_opt_nsc[joints.index('knee_angle_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('knee_angle_l'), j+1])        
                    passiveJointTorque_ankle_angle_rj_opt = f_passiveJointTorque_ankle_angle(
                            Qskj_opt_nsc[joints.index('ankle_angle_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('ankle_angle_r'), j+1])
                    passiveJointTorque_ankle_angle_lj_opt = f_passiveJointTorque_ankle_angle(
                            Qskj_opt_nsc[joints.index('ankle_angle_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('ankle_angle_l'), j+1]) 
                    passiveJointTorque_subtalar_angle_rj_opt = f_passiveJointTorque_subtalar_angle(
                            Qskj_opt_nsc[joints.index('subtalar_angle_r'), j+1], 
                            Qdotskj_opt_nsc[joints.index('subtalar_angle_r'), j+1])
                    passiveJointTorque_subtalar_angle_lj_opt = f_passiveJointTorque_subtalar_angle(
                            Qskj_opt_nsc[joints.index('subtalar_angle_l'), j+1], 
                            Qdotskj_opt_nsc[joints.index('subtalar_angle_l'), j+1])
                    passiveJointTorque_lumbar_extensionj_opt = (
                            f_passiveJointTorque_lumbar_extension(
                            Qskj_opt_nsc[joints.index('lumbar_extension'), j+1], 
                            Qdotskj_opt_nsc[joints.index('lumbar_extension'), j+1])) 
                    passiveJointTorque_lumbar_bendingj_opt = (
                            f_passiveJointTorque_lumbar_bending(
                            Qskj_opt_nsc[joints.index('lumbar_bending'), j+1], 
                            Qdotskj_opt_nsc[joints.index('lumbar_bending'), j+1]))
                    passiveJointTorque_lumbar_rotationj_opt = (
                            f_passiveJointTorque_lumbar_rotation(
                            Qskj_opt_nsc[joints.index('lumbar_rotation'), j+1], 
                            Qdotskj_opt_nsc[joints.index('lumbar_rotation'), j+1]))                    
                    passiveJointTorquesj_opt = ca.vertcat(
                        passiveJointTorque_hip_flexion_rj_opt,
                        passiveJointTorque_hip_flexion_lj_opt,
                        passiveJointTorque_hip_adduction_rj_opt,
                        passiveJointTorque_hip_adduction_lj_opt,
                        passiveJointTorque_hip_rotation_rj_opt,
                        passiveJointTorque_hip_rotation_lj_opt,
                        passiveJointTorque_knee_angle_rj_opt,
                        passiveJointTorque_knee_angle_lj_opt,
                        passiveJointTorque_ankle_angle_rj_opt,
                        passiveJointTorque_ankle_angle_lj_opt,
                        passiveJointTorque_subtalar_angle_rj_opt,
                        passiveJointTorque_subtalar_angle_lj_opt,
                        passiveJointTorque_lumbar_extensionj_opt,
                        passiveJointTorque_lumbar_bendingj_opt,
                        passiveJointTorque_lumbar_rotationj_opt)
                    
                    ###########################################################
                    # Polynomial approximations
                    # Left leg
                    Qsinj_opt_l = Qskj_opt_nsc[leftPolynomialJointIndices, j+1]
                    Qdotsinj_opt_l = Qdotskj_opt_nsc[leftPolynomialJointIndices, j+1]
                    [lMTj_opt_l, vMTj_opt_l, _] = f_polynomial(Qsinj_opt_l, Qdotsinj_opt_l)       
                    # Right leg
                    Qsinj_opt_r = Qskj_opt_nsc[rightPolynomialJointIndices, j+1]
                    Qdotsinj_opt_r = Qdotskj_opt_nsc[rightPolynomialJointIndices, j+1]
                    [lMTj_opt_r, vMTj_opt_r, _] = f_polynomial(Qsinj_opt_r, Qdotsinj_opt_r)
                    # Both legs        
                    lMTj_opt_lr = ca.vertcat(lMTj_opt_l[leftPolynomialMuscleIndices], 
                                             lMTj_opt_r[rightPolynomialMuscleIndices])
                    vMTj_opt_lr = ca.vertcat(vMTj_opt_l[leftPolynomialMuscleIndices], 
                                             vMTj_opt_r[rightPolynomialMuscleIndices])                    
                    
                    ###########################################################
                    # Derive Hill-equilibrium        
                    [hillEquilibriumj_opt, Fj_opt, activeFiberForcej_opt, 
                     passiveFiberForcej_opt, normActiveFiberLengthForcej_opt, 
                     normFiberLengthj_opt, fiberVelocityj_opt] = (
                         f_hillEquilibrium(akj_opt[:, j+1], lMTj_opt_lr, 
                                           vMTj_opt_lr,
                                           normFkj_opt_nsc[:, j+1], 
                                           normFDtj_opt_nsc[:, j]))   
                    
                    ###########################################################
                    # Get metabolic energy rate
                    [activationHeatRatej_opt, maintenanceHeatRatej_opt, 
                     shorteningHeatRatej_opt, mechanicalWorkRatej_opt, _, 
                     metabolicEnergyRatej_opt] = f_metabolicsBhargava(
                         akj_opt[:, j+1], akj_opt[:, j+1], normFiberLengthj_opt, 
                         fiberVelocityj_opt, activeFiberForcej_opt,
                         passiveFiberForcej_opt, normActiveFiberLengthForcej_opt)
                    
                    ###########################################################
                    # Motor control terms.
                    activationTerm_opt = f_NMusclesSum2(akj_opt[:, j+1])  
                    jointAccelerationTerm_opt = f_NNoArmJointsSum2(Qdotdotsj_opt[idxNoArmJoints, j])          
                    passiveJointTorqueTerm_opt = f_NPassiveTorqueJointsSum2(passiveJointTorquesj_opt)     
                    activationDtTerm_opt = f_NMusclesSum2(aDtk_opt)
                    forceDtTerm_opt = f_NMusclesSum2(normFDtj_opt[:, j])
                    armAccelerationTerm_opt = f_NArmJointsSum2(Qdotdotsj_opt[idxArmJoints, j])
                    armExcitationTerm_opt = f_NArmJointsSum2(eArmk_opt) 
                    metabolicEnergyRateTerm_opt = (f_NMusclesSum2(metabolicEnergyRatej_opt) / modelMass)
                    
                    metabolicEnergyRateTerm_opt_all += weights['metabolicEnergyRateTerm'] * metabolicEnergyRateTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    activationTerm_opt_all += weights['activationTerm'] * activationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    armExcitationTerm_opt_all += weights['armExcitationTerm'] * armExcitationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    jointAccelerationTerm_opt_all += weights['jointAccelerationTerm'] * jointAccelerationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    passiveJointTorqueTerm_opt_all += weights['passiveJointTorqueTerm'] * passiveJointTorqueTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    activationDtTerm_opt_all += weights['controls'] * activationDtTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    forceDtTerm_opt_all += weights['controls'] * forceDtTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    armAccelerationTerm_opt_all += weights['controls'] * armAccelerationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 

        JAll_opt = (metabolicEnergyRateTerm_opt_all.full() +
                     activationTerm_opt_all.full() + 
                     armExcitationTerm_opt_all.full() + 
                     jointAccelerationTerm_opt_all.full() + 
                     passiveJointTorqueTerm_opt_all.full() + 
                     activationDtTerm_opt_all.full() + 
                     forceDtTerm_opt_all.full() + 
                     armAccelerationTerm_opt_all.full())
        
        assert np.alltrue(
                np.abs(JAll_opt[0][0] - stats['iterations']['obj'][-1]) 
                <= 1e-6), "decomposition cost"
        
        # %% Reconstruct gait cycle
        from variousFunctions import getIdxIC_3D
        threshold = 30
        idxIC, legIC = getIdxIC_3D(GRF_opt, threshold)
        if legIC == "undefined":
            np.disp("Problem with gait reconstruction")  
        idxIC_s = idxIC + 1 # GRF_opt obtained at mesh points starting at k=1
        idxIC_c = idxIC 
            
        # Joint coordinates
        Qs_GC = np.zeros((NJoints, 2*N))
        Qs_GC[:, :N-idxIC_s[0]] = Qs_opt_nsc[:, idxIC_s[0]:-1]
        Qs_GC[idxPeriodicQsJointsA, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                Qs_opt_nsc[idxPeriodicQsJointsB, :-1])
        Qs_GC[idxPeriodicOppositeJoints, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                -Qs_opt_nsc[idxPeriodicOppositeJoints, :-1])
        Qs_GC[joints.index('pelvis_tx'), N-idxIC_s[0]:N-idxIC_s[0]+N]  = (
                Qs_opt_nsc[joints.index('pelvis_tx'), :-1] + 
                Qs_opt_nsc[joints.index('pelvis_tx'), -1])
        Qs_GC[:, N-idxIC_s[0]+N:2*N] = Qs_opt_nsc[:,:idxIC_s[0]] 
        Qs_GC[joints.index('pelvis_tx'), N-idxIC_s[0]+N:2*N] = (
                Qs_opt_nsc[joints.index('pelvis_tx'),:idxIC_s[0]] + 
                2*Qs_opt_nsc[joints.index('pelvis_tx'), -1])
        if legIC == "left":
            Qs_GC[idxPeriodicQsJointsA, :] = Qs_GC[idxPeriodicQsJointsB, :]
            Qs_GC[idxPeriodicOppositeJoints, :] = (
                    -Qs_GC[idxPeriodicOppositeJoints, :])
        Qs_GC[joints.index('pelvis_tx'), :] -= Qs_GC[joints.index('pelvis_tx'), 0]
        Qs_GC[idxRotationalJoints, :] = Qs_GC[idxRotationalJoints, :] * 180 / np.pi    
    
        # Joint velocities
        Qdots_GC = np.zeros((NJoints, 2*N))
        Qdots_GC[:, :N-idxIC_s[0]] = Qdots_opt_nsc[:, idxIC_s[0]:-1]
        Qdots_GC[idxPeriodicQdotsJointsA, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                Qdots_opt_nsc[idxPeriodicQdotsJointsB, :-1])
        Qdots_GC[idxPeriodicQdotsJointsA, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                Qdots_opt_nsc[idxPeriodicQdotsJointsB, :-1])
        Qdots_GC[idxPeriodicOppositeJoints, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                -Qdots_opt_nsc[idxPeriodicOppositeJoints, :-1])
        Qdots_GC[:, N-idxIC_s[0]+N:2*N] = Qdots_opt_nsc[:,:idxIC_s[0]] 
        if legIC == "left":
            Qdots_GC[idxPeriodicQdotsJointsA, :] = Qdots_GC[idxPeriodicQdotsJointsB, :]
            Qdots_GC[idxPeriodicOppositeJoints, :] = -Qdots_GC[idxPeriodicOppositeJoints, :]
        Qdots_GC[idxRotationalJoints, :] = Qdots_GC[idxRotationalJoints, :] * 180 / np.pi
        
        # Joint accelerations
        Qdotdots_GC = np.zeros((NJoints, 2*N))
        Qdotdots_GC[:, :N-idxIC_c[0]] = Qdotdots_opt[:, idxIC_c[0]:]
        Qdotdots_GC[idxPeriodicQdotsJointsA, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                Qdotdots_opt[idxPeriodicQdotsJointsB, :])
        Qdotdots_GC[idxPeriodicOppositeJoints, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                -Qdotdots_opt[idxPeriodicOppositeJoints, :])
        Qdotdots_GC[:, N-idxIC_c[0]+N:2*N] = Qdotdots_opt[:,:idxIC_c[0]] 
        if legIC == "left":
            Qdotdots_GC[idxPeriodicQdotsJointsA, :] = Qdotdots_GC[idxPeriodicQdotsJointsB, :]
            Qdotdots_GC[idxPeriodicOppositeJoints, :] = -Qdotdots_GC[idxPeriodicOppositeJoints, :]
        Qdotdots_GC[idxRotationalJoints, :] = Qdotdots_GC[idxRotationalJoints, :] * 180 / np.pi
        
        # Muscle activations
        A_GC = np.zeros((NMuscles, 2*N))
        A_GC[:, :N-idxIC_s[0]] = a_opt[:, idxIC_s[0]:-1]
        A_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = a_opt[idxPeriodicMuscles, :-1]
        A_GC[:, N-idxIC_s[0]+N:2*N] = a_opt[:,:idxIC_s[0]] 
        if legIC == "left":
            A_GC = A_GC[idxPeriodicMuscles, :]
            
        # Muscle force
        F_GC = np.zeros((NMuscles, 2*N))
        F_GC[:, :N-idxIC_s[0]] = normF_opt_nsc[:, idxIC_s[0]:-1]
        F_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = normF_opt_nsc[idxPeriodicMuscles, :-1]
        F_GC[:, N-idxIC_s[0]+N:2*N] = normF_opt_nsc[:,:idxIC_s[0]] 
        if legIC == "left":
            F_GC = F_GC[idxPeriodicMuscles, :]
            
        # Muscle force derivative
        FDt_GC = np.zeros((NMuscles, 2*N))
        FDt_GC[:, :N-idxIC_c[0]] = normFDt_opt_nsc[:, idxIC_c[0]:]
        FDt_GC[:, N-idxIC_c[0]:N-idxIC_c[0]+N] = normFDt_opt_nsc[idxPeriodicMuscles, :]
        FDt_GC[:, N-idxIC_c[0]+N:2*N] = normFDt_opt_nsc[:,:idxIC_c[0]] 
        if legIC == "left":
            FDt_GC = FDt_GC[idxPeriodicMuscles, :]
            
        # Arm actuator activations
        aArm_GC = np.zeros((NArmJoints, 2*N))
        aArm_GC[:, :N-idxIC_s[0]] = aArm_opt[:, idxIC_s[0]:-1]
        aArm_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                aArm_opt[idxPeriodicArmJoints, :-1])
        aArm_GC[:, N-idxIC_s[0]+N:2*N] = aArm_opt[:,:idxIC_s[0]] 
        if legIC == "left":
            aArm_GC = aArm_GC[idxPeriodicArmJoints, :]
            
        # Joint torques
        torques_GC = np.zeros((NJoints, 2*N))
        torques_GC[:, :N-idxIC_c[0]] = torques_opt[:, idxIC_c[0]:]
        torques_GC[idxPeriodicQdotsJointsA, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                torques_opt[idxPeriodicQdotsJointsB, :])
        torques_GC[idxPeriodicOppositeJoints, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                -torques_opt[idxPeriodicOppositeJoints, :])
        torques_GC[:, N-idxIC_c[0]+N:2*N] = torques_opt[:,:idxIC_c[0]] 
        if legIC == "left":
            torques_GC[idxPeriodicQdotsJointsA, :] = torques_GC[idxPeriodicQdotsJointsB, :]
            torques_GC[idxPeriodicOppositeJoints, :] = -torques_GC[idxPeriodicOppositeJoints, :]
            
        # Ground reaction forces
        GRF_GC = np.zeros((NGRF, 2*N))
        GRF_GC[:, :N-idxIC_c[0]] = GRF_opt[:, idxIC_c[0]:]
        GRF_GC[idxPeriodicGRFA, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                GRF_opt[idxPeriodicGRFB, :])
        GRF_GC[idxOppositeGRFA, N-idxIC_c[0]:N-idxIC_c[0]+N] = -(
                GRF_opt[idxOppositeGRFB, :])    
        GRF_GC[:, N-idxIC_c[0]+N:2*N] = GRF_opt[:,:idxIC_c[0]] 
        if legIC == "left":
            GRF_GC[idxPeriodicGRFA, :] = GRF_GC[idxPeriodicGRFB, :]
            GRF_GC[idxOppositeGRFA, :] = -GRF_GC[idxOppositeGRFB, :]
            
        # Time grid
        tgrid = np.linspace(0, finalTime_opt[0], N+1)
        tgrid_GC = np.zeros((1, 2*N)) 
        tgrid_GC[:,:N] = tgrid[:N].T
        tgrid_GC[:,N:] = tgrid[:N].T + tgrid[-1].T
        
        # %% Write motion file for visualization in OpenSim GUI
        if writeMotion:        
            muscleLabels = [bothSidesMuscle + '/activation'
                            for bothSidesMuscle in bothSidesMuscles]        
            labels = ['time'] + joints + muscleLabels
            data = np.concatenate((tgrid_GC.T, Qs_GC.T, A_GC.T), axis=1)             
            from variousFunctions import numpy2storage
            numpy2storage(labels, data, os.path.join(pathResults,'motion.mot'))
            
        # %% Compute metabolic cost of transport for whole gait cycle    
        Qs_GC_deg = Qs_GC.copy()        
        Qs_GC_deg[idxRotationalJoints, :] = (
            Qs_GC_deg[idxRotationalJoints, :] * np.pi / 180)
        Qdots_GC_deg = Qdots_GC.copy()        
        Qdots_GC_deg[idxRotationalJoints, :] = (
            Qdots_GC_deg[idxRotationalJoints, :] * np.pi / 180)     
        basal_coef = 1.2
        basal_exp = 1        
        totalMetabolicEnergyRate = np.zeros((1,2*N))
        for k in range(2*N):
            ###################################################################
            # Polynomial approximations
            # Left leg
            Qsk_GC_l = Qs_GC_deg[leftPolynomialJointIndices, k]
            Qdotsk_GC_l = Qdots_GC_deg[leftPolynomialJointIndices, k]
            [lMTk_GC_l, vMTk_GC_l, _] = f_polynomial(Qsk_GC_l, Qdotsk_GC_l)       
            # Right leg
            Qsk_GC_r = Qs_GC_deg[rightPolynomialJointIndices, k]
            Qdotsk_GC_r = Qdots_GC_deg[rightPolynomialJointIndices, k]
            [lMTk_GC_r, vMTk_GC_r, _] = f_polynomial(Qsk_GC_r, Qdotsk_GC_r)
            # Both legs        
            lMTk_GC_lr = ca.vertcat(lMTk_GC_l[leftPolynomialMuscleIndices], 
                                     lMTk_GC_r[rightPolynomialMuscleIndices])
            vMTk_GC_lr = ca.vertcat(vMTk_GC_l[leftPolynomialMuscleIndices], 
                                     vMTk_GC_r[rightPolynomialMuscleIndices])                    
            
            ###################################################################
            # Derive Hill-equilibrium        
            [hillEquilibriumk_GC, Fk_GC, activeFiberForcek_GC, 
             passiveFiberForcek_GC, normActiveFiberLengthForcek_GC, 
             normFiberLengthk_GC, fiberVelocityk_GC] = (
                 f_hillEquilibrium(A_GC[:, k], lMTk_GC_lr, vMTk_GC_lr,
                                   F_GC[:, k], FDt_GC[:, k]))   
            
            ###################################################################
            # Get metabolic energy rate
            [activationHeatRatek_GC, maintenanceHeatRatek_GC, 
             shorteningHeatRatek_GC, mechanicalWorkRatek_GC, _, 
             metabolicEnergyRatek_GC] = f_metabolicsBhargava(
                 A_GC[:, k], A_GC[:, k], normFiberLengthj_opt, 
                 fiberVelocityj_opt, activeFiberForcej_opt,
                 passiveFiberForcej_opt, normActiveFiberLengthForcej_opt)    
            # Sum over all muscles                     
            metabolicEnergyRatek_allMuscles = np.sum(
                metabolicEnergyRatek_GC.full())
            # Add basal rate
            basalRatek = basal_coef*modelMass**basal_exp
            totalMetabolicEnergyRate[0, k] = (metabolicEnergyRatek_allMuscles 
                                              + basalRatek)      
        # Integrate
        totalMetabolicEnergyRate_int = np.trapz(totalMetabolicEnergyRate,
                                                tgrid_GC)   
        # Total distance traveled
        distTraveled_opt_GC = (Qs_GC_deg[joints.index('pelvis_tx'),-1] - 
                               Qs_GC_deg[joints.index('pelvis_tx'),0])
        # Cost of transport (COT)
        COT_GC = totalMetabolicEnergyRate_int / modelMass / distTraveled_opt_GC  
            
         # %% Save trajectories for further analysis
        if saveTrajectories: 
            if not os.path.exists(os.path.join(pathTrajectories,
                                               'optimaltrajectories.npy')): 
                    optimaltrajectories = {}
            else:  
                optimaltrajectories = np.load(
                        os.path.join(pathTrajectories, 'optimaltrajectories.npy'),
                        allow_pickle=True)   
                optimaltrajectories = optimaltrajectories.item()  
                
            GC_percent = np.linspace(1, 100, 2*N)
            
            optimaltrajectories[case] = {
                                'coordinate_values': Qs_GC, 
                                'coordinate_speeds': Qdots_GC, 
                                'coordinate_accelerations': Qdotdots_GC,
                                'muscle_activations': A_GC,
                                'arm_activations': aArm_GC,
                                'joint_torques': torques_GC,
                                'GRF': GRF_GC,
                                'time': tgrid_GC,
                                'joints': joints,
                                'muscles': bothSidesMuscles,
                                'GRF_labels': GRFNames,
                                'COT': COT_GC,
                                'GC_percent': GC_percent}              
            np.save(os.path.join(pathTrajectories, 'optimaltrajectories.npy'),
                    optimaltrajectories)
            
        # %% Error message
        if not stats['success'] == True:
            print("WARNING: PROBLEM DID NOT CONVERGE - " 
                  + stats['return_status']) 
            