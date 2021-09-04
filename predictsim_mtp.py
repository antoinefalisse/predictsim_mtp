import os
import casadi as ca
import numpy as np

solveProblem = True
saveResults = True
analyzeResults = True
loadResults = True
writeMotion = True
saveTrajectories = True
decomposeCost = True
loadMTParameters = True
loadPolynomialData = True
plotPolynomials = False

cases = ['177']

from settings_predictsim import getSettings_predictsim_mtp   
settings = getSettings_predictsim_mtp() 
for case in cases:
    
    # Default cost term weights.
    weights = {'metabolicEnergyRateTerm' : 500,
               'activationTerm': 2000,
               'jointAccelerationTerm': 50000,
               'armExcitationTerm': 1000000,
               'passiveTorqueTerm': 1000, 
               'controls': 0.001}  
    # Set different weights through settings.
    if 'metabolicEnergyRateTerm' in settings[case]:
        weights['metabolicEnergyRateTerm'] = (
            settings[case]['metabolicEnergyRateTerm'])
    if 'activationTerm' in settings[case]:
        weights['activationTerm'] = (
            settings[case]['activationTerm'])
    if 'jointAccelerationTerm' in settings[case]:
        weights['jointAccelerationTerm'] = (
            settings[case]['jointAccelerationTerm'])
    if 'armExcitationTerm' in settings[case]:
        weights['armExcitationTerm'] = (
            settings[case]['armExcitationTerm'])
    if 'passiveTorqueTerm' in settings[case]:
        weights['passiveTorqueTerm'] = (
            settings[case]['passiveTorqueTerm'])
    if 'controls' in settings[case]:
        weights['controls'] = (
            settings[case]['controls'])

    # Numerical settings
    tol = 4 # default IPOPT convergence tolerance.
    if 'tol' in settings[case]:
        tol = settings[case]['tol']
    
    N = 50 # default number of mesh intervals.
    if 'tol' in settings[case]:
        N = settings[case]['N']
        
    d = 3 # default interpolating polynomial order.
    if 'd' in settings[case]:
        d = settings[case]['d']    
    
    nThreads = 8 # default number of threads.
    if 'nThreads' in settings[case]:
        nThreads = settings[case]['nThreads']
    parallelMode = "thread" # only supported mode.
    
    # Problem settings.
    targetSpeed = 1.33 # default target walking.
    if 'targetSpeed' in settings[case]:
        targetSpeed = settings[case]['targetSpeed']
        
    guessType = 'quasiRandom' # default initial guess mode.
    if 'guessType' in settings[case]:
        guessType = settings[case]['guessType']
    
    contactConfiguration = 'generic' # default foot-ground contact model.
    if 'contactConfiguration' in settings[case]:
        contactConfiguration = settings[case]['contactConfiguration']
        
    adjustAchillesTendonCompliance = False # default Achilles tendon stiffness.
    if 'adjustAchillesTendonCompliance' in settings[case]:
        adjustAchillesTendonCompliance = (
            settings[case]['adjustAchillesTendonCompliance'])
        
    if 'idxSubject' in settings[case]:
        idxSubject = settings[case]['idxSubject'] 
    else:
        idxSubject = "1"        
    subject = 'subject' + idxSubject + '_mtp'
        
    baseConfig = ''
    if 'baseConfig' in settings[case]:
        baseConfig = settings[case]['baseConfig']
        
    boundsType = 'nominal' 
    if 'boundsType' in settings[case]:
        boundsType = settings[case]['boundsType']
        
    modelMass = settings[case]['modelMass']
         
    # Paths
    pathMain = os.getcwd()
    pathData = os.path.join(pathMain, 'OpenSimModel', subject)
    # TODO
    if idxSubject == '1':
        pathModel = os.path.join(pathData, 'Model', subject + ".osim")
    elif idxSubject == '2':
        pathModel = os.path.join(pathData, 'Model', 'subject' + idxSubject + '_withMTP_scaled.osim')
    pathMTParameters = os.path.join(pathData, 'Model')
    filename = os.path.basename(__file__)
    pathCase = 'Case_' + case    
    pathTrajectories = os.path.join(pathMain, 'Results', filename[:-3]) 
    pathResults = os.path.join(pathTrajectories, pathCase)
    os.makedirs(pathResults, exist_ok=True)
    
    # %% Muscle part.
    muscles = [
        'glut_med1_r', 'glut_med2_r', 'glut_med3_r', 'glut_min1_r', 
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
    nMuscles = len(bothSidesMuscles)
    nSideMuscles = len(rightSideMuscles)
    
    # Muscle-tendon parameters.
    from muscleData import getMTParameters
    sideMtParameters = getMTParameters(pathModel, rightSideMuscles,
                                       loadMTParameters, pathMTParameters)
    mtParameters = np.concatenate((sideMtParameters, sideMtParameters),
                                  axis=1)
    
    # Tendon compliance.
    from muscleData import tendonCompliance
    # Same compliance for all tendons by default.
    sideTendonCompliance = tendonCompliance(nSideMuscles)
    # Adjust Achilles tendon stiffness (triceps surae).
    if adjustAchillesTendonCompliance:
        AchillesTendonCompliance = settings[case]['AchillesTendonCompliance']
        musclesAchillesTendon = ['med_gas_r', 'lat_gas_r', 'soleus_r']
        idxMusclesAchillesTendon = [
            rightSideMuscles.index(muscleAchillesTendon) 
            for muscleAchillesTendon in musclesAchillesTendon]
        sideTendonCompliance[0, idxMusclesAchillesTendon] = (
            AchillesTendonCompliance)                
    tendonCompliance = np.concatenate((sideTendonCompliance, 
                                       sideTendonCompliance), axis=1)
    
    # Muscle specific tension.
    from muscleData import specificTension
    sideSpecificTension = specificTension(rightSideMuscles)
    specificTension = np.concatenate((sideSpecificTension, 
                                      sideSpecificTension), axis=1)
    
    # Hill-equilibrium.
    from functionCasADi import hillEquilibrium
    f_hillEquilibrium = hillEquilibrium(mtParameters, tendonCompliance, 
                                        specificTension)
    
    # Activation dynamics time constants.
    activationTimeConstant = 0.015
    deactivationTimeConstant = 0.06
    
    # Indices periodic muscles.
    idxPeriodicMuscles = (list(range(nSideMuscles, nMuscles)) + 
                          list(range(0, nSideMuscles)))
    
    # %% Joint part.
    from variousFunctions import getJointIndices
    # All joints.
    joints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
              'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 
              'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
              'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
              'knee_angle_l', 'knee_angle_r', 
              'ankle_angle_l', 'ankle_angle_r', 
              'subtalar_angle_l', 'subtalar_angle_r', 
              'mtp_angle_l', 'mtp_angle_r', 
              'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
              'arm_flex_l', 'arm_add_l', 'arm_rot_l',
              'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
              'elbow_flex_l', 'elbow_flex_r']
    nJoints = len(joints)
    
    # Rotational joints.
    rotationalJoints = [
        'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'knee_angle_l', 'knee_angle_r',
        'ankle_angle_l', 'ankle_angle_r', 
        'subtalar_angle_l', 'subtalar_angle_r', 
        'mtp_angle_l', 'mtp_angle_r',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 
        'arm_flex_l', 'arm_add_l', 'arm_rot_l',
        'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
        'elbow_flex_l', 'elbow_flex_r']
    idxRotationalJoints = getJointIndices(joints, rotationalJoints)
    
    # Helper lists for periodic constraints.
    # The joint positions in periodicQsJointsA after half a gait cycle should
    # match the positions in periodicQsJointsB at the first time instant.
    # The order matters, eg 'hip_flexion_l' should match 'hip_flexion_r'.
    periodicQsJointsA = [
        'pelvis_tilt', 'pelvis_ty', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'knee_angle_l', 'knee_angle_r', 
        'ankle_angle_l', 'ankle_angle_r', 
        'subtalar_angle_l', 'subtalar_angle_r', 
        'mtp_angle_l', 'mtp_angle_r',
        'lumbar_extension', 
        'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
        'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
        'elbow_flex_l', 'elbow_flex_r']
    idxPeriodicQsJointsA = getJointIndices(joints, periodicQsJointsA)
    periodicQsJointsB = [
        'pelvis_tilt', 'pelvis_ty', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'knee_angle_r', 'knee_angle_l', 
        'ankle_angle_r', 'ankle_angle_l', 
        'subtalar_angle_r', 'subtalar_angle_l', 
        'mtp_angle_r', 'mtp_angle_l',
        'lumbar_extension', 
        'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
        'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
        'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicQsJointsB = getJointIndices(joints, periodicQsJointsB)
    
    # The joint velocities in periodicQdsJointsA after half a gait cycle
    # should match the velocities in periodicQdsJointsB at the first time
    # instant.
    # The order matters, eg 'hip_flexion_l' should match 'hip_flexion_r'.
    periodicQdsJointsA = [
        'pelvis_tilt', 'pelvis_tx', 'pelvis_ty', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'knee_angle_l', 'knee_angle_r', 
        'ankle_angle_l', 'ankle_angle_r', 
        'subtalar_angle_l', 'subtalar_angle_r', 
        'mtp_angle_l', 'mtp_angle_r',
        'lumbar_extension', 
        'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
        'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
        'elbow_flex_l', 'elbow_flex_r']
    idxPeriodicQdsJointsA = getJointIndices(joints, periodicQdsJointsA)
    periodicQdsJointsB = [
        'pelvis_tilt', 'pelvis_tx', 'pelvis_ty', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'knee_angle_r', 'knee_angle_l', 
        'ankle_angle_r', 'ankle_angle_l', 
        'subtalar_angle_r', 'subtalar_angle_l', 
        'mtp_angle_r', 'mtp_angle_l',
        'lumbar_extension', 
        'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
        'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
        'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicQdsJointsB = getJointIndices(joints, periodicQdsJointsB)
    
    # The joint positions and velocities in periodicOppositeJoints after half
    # a gait cycle should be opposite to those at the first time instant.
    periodicOppositeJoints = ['pelvis_list', 'pelvis_rotation', 'pelvis_tz', 
                              'lumbar_bending', 'lumbar_rotation']
    idxPeriodicOppositeJoints = getJointIndices(joints, periodicOppositeJoints)
    
    # Arm joints
    armJoints = ['arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                 'arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                 'elbow_flex_l', 'elbow_flex_r']
    nArmJoints = len(armJoints)
    idxArmJoints = getJointIndices(joints, armJoints)
    # The activations in periodicArmJoints after half a gait cycle should
    # match the activation in armJoints at the first time instant.
    periodicArmJoints = ['arm_flex_r', 'arm_add_r', 'arm_rot_r', 
                         'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                         'elbow_flex_r', 'elbow_flex_l']
    idxPeriodicArmJoints = getJointIndices(armJoints, periodicArmJoints)
    
    # All but arm joints.
    noArmJoints = [
        'pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
        'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l',
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r',
        'knee_angle_l', 'knee_angle_r',
        'ankle_angle_l', 'ankle_angle_r',
        'subtalar_angle_l', 'subtalar_angle_r', 
        'mtp_angle_l', 'mtp_angle_r',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    nNoArmJoints = len(noArmJoints)
    idxNoArmJoints = getJointIndices(joints, noArmJoints)
    
    # Joints with passive torques.
    passiveTorqueJoints = [
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
        'knee_angle_l', 'knee_angle_r',
        'ankle_angle_l', 'ankle_angle_r',
        'subtalar_angle_l', 'subtalar_angle_r',
        'mtp_angle_l', 'mtp_angle_r',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    nPassiveTorqueJoints = len(passiveTorqueJoints)
    
    # Ground pelvis joints.
    groundPelvisJoints = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                          'pelvis_tx', 'pelvis_ty', 'pelvis_tz']
    idxGroundPelvisJoints = getJointIndices(joints, groundPelvisJoints)
   
    # Mtp joints
    mtpJoints = ['mtp_angle_l', 'mtp_angle_r']
    nMtpJoints = len(mtpJoints)
   
    # Trunk joints
    trunkJoints = ['lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    
    # Muscle-driven joints.
    muscleDrivenJoints = [ 
        'hip_flexion_l', 'hip_flexion_r', 'hip_adduction_l', 'hip_adduction_r',
        'hip_rotation_l', 'hip_rotation_r', 'knee_angle_l', 'knee_angle_r', 
        'ankle_angle_l', 'ankle_angle_r', 
        'subtalar_angle_l', 'subtalar_angle_r',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation']
    
    # %% GRF part.    
    GRFNames = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r',
                'GRF_x_l','GRF_y_l', 'GRF_z_l']
    
    # %% Polynomial part.
    from functionCasADi import polynomialApproximation
    leftPolynomialJoints = [
        'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', 'knee_angle_l',
        'ankle_angle_l', 'subtalar_angle_l', 'mtp_angle_l',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'] 
    rightPolynomialJoints = [
        'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 'knee_angle_r',
        'ankle_angle_r', 'subtalar_angle_r', 'mtp_angle_r',
        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'] 
    
    from muscleData import getPolynomialData      
    pathCoordinates = os.path.join(pathData, 'MA', 'dummy_motion.mot')
    
    pathMuscleAnalysis = os.path.join(
        pathData, 'MA', 'ResultsMA', 'subject' + idxSubject, 
        'subject' + idxSubject + '_MuscleAnalysis_') 
    polynomialData = getPolynomialData(
        loadPolynomialData, pathMTParameters, pathCoordinates, 
        pathMuscleAnalysis, rightPolynomialJoints, muscles)   
    if loadPolynomialData:
        polynomialData = polynomialData.item()
    
    nPolynomialJoints = len(leftPolynomialJoints)
    # The function f_polynomial takes as inputs joint positions and velocities
    # from one side (trunk included), and returns muscle-tendon lengths,
    # velocities, and moments for the muscle of that side (trunk included).
    f_polynomial = polynomialApproximation(muscles, polynomialData,
                                           nPolynomialJoints)
    leftPolynomialJointIndices = getJointIndices(joints,
                                                 leftPolynomialJoints)
    rightPolynomialJointIndices = getJointIndices(joints,
                                                  rightPolynomialJoints)
    
    # The left and right polynomialMuscleIndices below are used to identify
    # the left and right muscles in the output of f_polynomial. Since
    # f_polynomial return data from side muscles (trunk included), we have the
    # side leg muscles + all trunk muscles as output. Here we make sure we
    # only include the side trunk muscles when identifying all side muscles.
    # This is pretty sketchy I know.    
    rightPolynomialMuscleIndices = [muscles.index(i) for i in rightSideMuscles]
    rightTrunkMuscles = ['ercspn_r', 'intobl_r', 'extobl_r']
    leftTrunkMuscles = ['ercspn_l', 'intobl_l', 'extobl_l']
    leftPolynomialMuscleIndices = (
        [muscles.index(i) for i in rightSideMuscles 
         if i not in rightTrunkMuscles] + 
        [muscles.index(i) for i in leftTrunkMuscles])
    from variousFunctions import getMomentArmIndices
    momentArmIndices = getMomentArmIndices(
        rightSideMuscles, leftPolynomialJoints, rightPolynomialJoints,
        polynomialData)
    trunkMomentArmPolynomialIndices = (
        [muscles.index(i) for i in leftTrunkMuscles] + 
        [muscles.index(i) for i in rightTrunkMuscles])
    
    # Plot polynomial approximations
    if plotPolynomials:
        from polynomials import testPolynomials
        momentArms = testPolynomials(pathCoordinates, pathMuscleAnalysis, 
                                     rightPolynomialJoints, muscles, 
                                     f_polynomial, polynomialData, 
                                     momentArmIndices,
                                     trunkMomentArmPolynomialIndices)
        
    # %% External function.
    # The external function is written in C++ and compiled as a library, which
    # can then be called with CasADi. In the external function, we build the
    # OpenSim model and run inverse dynamics. The function takes as inputs
    # joint positions, velocities, and accelerations, which are states and
    # controls of the optimal control problem. The external function returns
    # joint torques as well as some outputs of interest, eg segment origins,
    # that you may want to use as part of the problem formulation.
    # We distinguish two external functions: F and F1. F is used during the
    # problem formulation but F1 is only used for post-processing - F1 has
    # output we do not use as part of the problem formulation. Having unused
    # outputs might slightly impact the optimal control problem, since the
    # external function is used for instance when computing the constraint
    # Jacobian; the number of inputs and outputs therefore matters.    
    pathExternalFunction = os.path.join(pathMain, 'ExternalFunction')
    if subject == "subject1_mtp":
        if contactConfiguration == 'generic':
            F = ca.external('F', os.path.join(
                pathExternalFunction, 'PredSim_mtpPin_cm0{}.dll'.format(baseConfig)))
            if analyzeResults:
                F1 = ca.external('F', os.path.join(
                    pathExternalFunction, 'PredSim_mtpPin_pp_cm0{}.dll'.format(baseConfig)))
    elif subject == "subject2_mtp":
        if contactConfiguration == 'generic':
            if baseConfig == 'ua_corrected':
                F = ca.external('F', os.path.join(
                    pathExternalFunction, 's2_withMTP_gb.dll'))
                if analyzeResults:
                    F1 = ca.external('F', os.path.join(
                        pathExternalFunction, 's2_withMTP_gb_pp.dll'))
    
    # The external function F outputs joint torques, as well as the 2D
    # coordinates of some body origins. The order matters. The joint torques
    # are returned in the order of the list joints. The indices of the 2D
    # coordinates of the body origins are then hard-coded as follows.
    # Origins calcaneus (2D).
    idxCalcOr_r = list(range(nJoints, nJoints+2))
    idxCalcOr_l = list(range(idxCalcOr_r[-1]+1, idxCalcOr_r[-1]+3))
    # Origins femurs (2D).
    idxFemurOr_r = list(range(idxCalcOr_l[-1]+1, idxCalcOr_l[-1]+3))
    idxFemurOr_l = list(range(idxFemurOr_r[-1]+1, idxFemurOr_r[-1]+3))
    # Origins hands (2D).
    idxHandOr_r = list(range(idxFemurOr_l[-1]+1, idxFemurOr_l[-1]+3))
    idxHandOr_l = list(range(idxHandOr_r[-1]+1, idxHandOr_r[-1]+3))
    # Origins tibias (2D).
    idxTibiaOr_r = list(range(idxHandOr_l[-1]+1, idxHandOr_l[-1]+3))
    idxTibiaOr_l = list(range(idxTibiaOr_r[-1]+1, idxTibiaOr_r[-1]+3))
    # Origins toes (2D).
    idxToesOr_r = list(range(idxTibiaOr_l[-1]+1, idxTibiaOr_l[-1]+3))
    idxToesOr_l = list(range(idxToesOr_r[-1]+1, idxToesOr_r[-1]+3))
    
    # The external function F1 outputs joint torques, ground reaction forces,
    # 3D coordinates of the origin of both calcaneus, and gruond reaction
    # moments. The order matters. The joint torques are returned in the order
    # of the list joints. The indices of the remaining outputs are then
    # hard-coded as follows.
    # Ground reaction forces (GRFs).
    idxGRF_r = list(range(nJoints, nJoints+3))
    idxGRF_l = list(range(idxGRF_r[-1]+1, idxGRF_r[-1]+4))
    idxGRF = idxGRF_r + idxGRF_l
    NGRF = len(idxGRF)
    # Origins calcaneus (3D).
    idxCalcOr3D_r = list(range(idxGRF_l[-1]+1, idxGRF_l[-1]+4))
    idxCalcOr3D_l = list(range(idxCalcOr3D_r[-1]+1, idxCalcOr3D_r[-1]+4))
    idxCalcOr3D = idxCalcOr3D_r + idxCalcOr3D_l
    NCalcOr3D = len(idxCalcOr3D)
    # Ground reaction moments (GRMs).
    idxGRM_r = list(range(idxCalcOr3D_l[-1]+1, idxCalcOr3D_l[-1]+4))
    idxGRM_l = list(range(idxGRM_r[-1]+1, idxGRM_r[-1]+4))
    idxGRM = idxGRM_r + idxGRM_l
    NGRM = len(idxGRM)
    # Number of outputs of F1.
    NF1_out = idxGRM_l[-1] + 1
    
    # %% Metabolic energy model. 
    maximalIsometricForce = mtParameters[0, :]
    optimalFiberLength = mtParameters[1, :]
    muscleVolume = np.multiply(maximalIsometricForce, optimalFiberLength)
    muscleMass = np.divide(np.multiply(muscleVolume, 1059.7), 
                           np.multiply(specificTension[0, :].T, 1e6))
    from muscleData import slowTwitchRatio
    sideSlowTwitchRatio = slowTwitchRatio(rightSideMuscles)
    slowTwitchRatio = (np.concatenate((sideSlowTwitchRatio, 
                                      sideSlowTwitchRatio), axis=1))[0, :].T
    smoothingConstant = 10
    from functionCasADi import metabolicsBhargava
    f_metabolicsBhargava = metabolicsBhargava(
        slowTwitchRatio, maximalIsometricForce, muscleMass, smoothingConstant)
    
    # %% Arm activation dynamics.
    from functionCasADi import armActivationDynamics
    f_armActivationDynamics = armActivationDynamics(nArmJoints)
    
    # %% Passive joint torques.
    from functionCasADi import passiveTorque
    from muscleData import passiveTorqueData
    damping = 0.1
    f_passiveTorque = {}
    for joint in passiveTorqueJoints:
        f_passiveTorque[joint] = passiveTorque(
            passiveTorqueData(joint)[0],
            passiveTorqueData(joint)[1], damping)
    
    from functionCasADi import passiveTorqueActuatedJointTorque
    stiffnessArm = 0
    dampingArm = 0.1
    f_linearPassiveArmTorque = passiveTorqueActuatedJointTorque(stiffnessArm, 
                                                                dampingArm)
    stiffnessMtp = 25
    dampingMtp = 0.4
    f_linearPassiveMtpTorque = passiveTorqueActuatedJointTorque(stiffnessMtp, 
                                                                dampingMtp)
    
    # %% Other helper CasADi functions
    from functionCasADi import normSumPow
    from functionCasADi import diffTorques
    f_NMusclesSum2 = normSumPow(nMuscles, 2)
    f_nArmJointsSum2 = normSumPow(nArmJoints, 2)
    f_nNoArmJointsSum2 = normSumPow(nNoArmJoints, 2)
    f_nPassiveTorqueJointsSum2 = normSumPow(nPassiveTorqueJoints, 2)
    f_diffTorques = diffTorques()
    
    # %% Bounds.  
    # Load average walking motion for some of the bounds and initial guess.
    motion_walk = 'walking'
    nametrial_walk_id = 'average_' +  motion_walk + '_HGC_mtp'
    nametrial_walk_IK = 'IK_' + nametrial_walk_id
    pathIK_walk = os.path.join(pathData, 'IK', nametrial_walk_IK + '.mot')
    from variousFunctions import getIK
    Qs_walk_filt = getIK(pathIK_walk, joints)[1]
    
    from bounds import bounds
    bounds = bounds(Qs_walk_filt, joints, rightSideMuscles, armJoints, 
                    targetSpeed, mtpJoints)
    
    # Static parameters.
    ubFinalTime, lbFinalTime = bounds.getBoundsFinalTime()
    
    # States.
    ubA, lbA, scalingA = bounds.getBoundsActivation()
    ubAk = ca.vec(ubA.to_numpy().T * np.ones((1, N+1))).full()
    lbAk = ca.vec(lbA.to_numpy().T * np.ones((1, N+1))).full()
    ubAj = ca.vec(ubA.to_numpy().T * np.ones((1, d*N))).full()
    lbAj = ca.vec(lbA.to_numpy().T * np.ones((1, d*N))).full()
    
    ubF, lbF, scalingF = bounds.getBoundsForce()
    ubFk = ca.vec(ubF.to_numpy().T * np.ones((1, N+1))).full()
    lbFk = ca.vec(lbF.to_numpy().T * np.ones((1, N+1))).full()
    ubFj = ca.vec(ubF.to_numpy().T * np.ones((1, d*N))).full()
    lbFj = ca.vec(lbF.to_numpy().T * np.ones((1, d*N))).full()
        
    ubQs, lbQs, scalingQs, ubQs0, lbQs0 = bounds.getBoundsPosition()
    ubQsk = ca.vec(ubQs.to_numpy().T * np.ones((1, N+1))).full()
    lbQsk = ca.vec(lbQs.to_numpy().T * np.ones((1, N+1))).full()
    ubQsj = ca.vec(ubQs.to_numpy().T * np.ones((1, d*N))).full()
    lbQsj = ca.vec(lbQs.to_numpy().T * np.ones((1, d*N))).full()
    # We want to further constraint the pelvis_tx position at the first mesh
    # point, such that the model starts walking with pelvis_tx=0.
    lbQsk[joints.index('pelvis_tx')] = lbQs0['pelvis_tx'].to_numpy()
    ubQsk[joints.index('pelvis_tx')] = ubQs0['pelvis_tx'].to_numpy()
    
    ubQds, lbQds, scalingQds = bounds.getBoundsVelocity()
    ubQdsk = ca.vec(ubQds.to_numpy().T * np.ones((1, N+1))).full()
    lbQdsk = ca.vec(lbQds.to_numpy().T * np.ones((1, N+1))).full()
    ubQdsj = ca.vec(ubQds.to_numpy().T * np.ones((1, d*N))).full()
    lbQdsj = ca.vec(lbQds.to_numpy().T * np.ones((1, d*N))).full()
    
    ubArmA, lbArmA, scalingArmA = bounds.getBoundsArmActivation()
    ubArmAk = ca.vec(ubArmA.to_numpy().T * np.ones((1, N+1))).full()
    lbArmAk = ca.vec(lbArmA.to_numpy().T * np.ones((1, N+1))).full()
    ubArmAj = ca.vec(ubArmA.to_numpy().T * np.ones((1, d*N))).full()
    lbArmAj = ca.vec(lbArmA.to_numpy().T * np.ones((1, d*N))).full()
    
    # Controls.
    ubADt, lbADt, scalingADt = bounds.getBoundsActivationDerivative()
    ubADtk = ca.vec(ubADt.to_numpy().T * np.ones((1, N))).full()
    lbADtk = ca.vec(lbADt.to_numpy().T * np.ones((1, N))).full()
    
    ubArmE, lbArmE, scalingArmE = bounds.getBoundsArmExcitation()
    ubArmEk = ca.vec(ubArmE.to_numpy().T * np.ones((1, N))).full()
    lbArmEk = ca.vec(lbArmE.to_numpy().T * np.ones((1, N))).full()
    
    # Slack controls.
    ubQdds, lbQdds, scalingQdds = (
            bounds.getBoundsAcceleration())
    ubQddsj = ca.vec(ubQdds.to_numpy().T * 
                              np.ones((1, d*N))).full()
    lbQddsj = ca.vec(lbQdds.to_numpy().T *
                              np.ones((1, d*N))).full()
    
    ubFDt, lbFDt, scalingFDt = bounds.getBoundsForceDerivative()
    ubFDtj = ca.vec(ubFDt.to_numpy().T * np.ones((1, d*N))).full()
    lbFDtj = ca.vec(lbFDt.to_numpy().T * np.ones((1, d*N))).full()
    
    # Other.
    _, _, scalingMtpE = bounds.getBoundsMtpExcitation()
    
    # %% Initial guess.
    if guessType == 'quasiRandom':
        from guesses import quasiRandomGuess
        guess = quasiRandomGuess(N, d, joints, bothSidesMuscles, targetSpeed)
    elif guessType == 'dataDriven':
        from guesses import dataDrivenGuess
        guess = dataDrivenGuess(Qs_walk_filt, N, d, joints, bothSidesMuscles,
                                targetSpeed, periodicQsJointsA, 
                                periodicQdsJointsA, periodicOppositeJoints)

    # Static parameters.
    gFinalTime = guess.getgFinalTime()
    
    # States.
    gA = guess.getgActivation(scalingA)
    gACol = guess.getgActivationCol()
    gF = guess.getgForce(scalingF)
    gFCol = guess.getgForceCol()
    gQs = guess.getGuessPosition(scalingQs)
    gQsCol = guess.getGuessPositionCol()
    gQds = guess.getGuessVelocity(scalingQds)
    gQdsCol = guess.getGuessVelocityCol()    
    gArmA = guess.getGuessTorqueActuatorActivation(armJoints)
    gArmACol = guess.getGuessTorqueActuatorActivationCol(armJoints)
    
    # Controls.
    gADt = guess.getgActivationDerivative(scalingADt)
    gArmE = guess.getGuessTorqueActuatorExcitation(armJoints)
    
    # Slack controls.
    gQdds = guess.getgAcceleration(scalingQdds)
    gQddsCol = guess.getgAccelerationCol()
    gFDt = guess.getgForceDerivative(scalingFDt)
    gFDtCol = guess.getgForceDerivativeCol()
    
    # %% Formulate optimal control problem.
    if solveProblem: 
        #######################################################################
        # Initialize opti instance.
        opti = ca.Opti()
        
        #######################################################################
        # Static parameters.
        # Final time.
        finalTime = opti.variable()
        opti.subject_to(opti.bounded(lbFinalTime.iloc[0]['time'],
                                     finalTime,
                                     ubFinalTime.iloc[0]['time']))
        opti.set_initial(finalTime, gFinalTime)
        assert lbFinalTime.iloc[0]['time'] <= gFinalTime, (
            "Error lower bound final time")
        assert ubFinalTime.iloc[0]['time'] >= gFinalTime, (
            "Error upper bound final time")
        
        #######################################################################
        # States.
        # Muscle activation at mesh points.
        a = opti.variable(nMuscles, N+1)
        opti.subject_to(opti.bounded(lbAk, ca.vec(a), ubAk))
        opti.set_initial(a, gA.to_numpy().T)
        assert np.alltrue(lbAk <= ca.vec(gA.to_numpy().T).full()), (
            "Error lower bound muscle activation")
        assert np.alltrue(ubAk >= ca.vec(gA.to_numpy().T).full()), (
            "Error upper bound muscle activation")
        # Muscle activation at collocation points.
        a_col = opti.variable(nMuscles, d*N)
        opti.subject_to(opti.bounded(lbAj, ca.vec(a_col), ubAj))
        opti.set_initial(a_col, gACol.to_numpy().T)
        assert np.alltrue(lbAj <= ca.vec(gACol.to_numpy().T).full()), (
            "Error lower bound muscle activation collocation points")
        assert np.alltrue(ubAj >= ca.vec(gACol.to_numpy().T).full()), (
            "Error upper bound muscle activation collocation points")
        # Muscle force at mesh points
        normF = opti.variable(nMuscles, N+1)
        opti.subject_to(opti.bounded(lbFk, ca.vec(normF), ubFk))
        opti.set_initial(normF, gF.to_numpy().T)
        assert np.alltrue(lbFk <= ca.vec(gF.to_numpy().T).full()), (
            "Error lower bound muscle force")
        assert np.alltrue(ubFk >= ca.vec(gF.to_numpy().T).full()), (
            "Error upper bound muscle force")
        # Muscle force at collocation points.
        normF_col = opti.variable(nMuscles, d*N)
        opti.subject_to(opti.bounded(lbFj, ca.vec(normF_col), ubFj))
        opti.set_initial(normF_col, gFCol.to_numpy().T)
        assert np.alltrue(lbFj <= ca.vec(gFCol.to_numpy().T).full()), (
            "Error lower bound muscle force collocation points")
        assert np.alltrue(ubFj >= ca.vec(gFCol.to_numpy().T).full()), (
            "Error upper bound muscle force collocation points")
        # Joint position at mesh points.
        Qs = opti.variable(nJoints, N+1)
        opti.subject_to(opti.bounded(lbQsk, ca.vec(Qs), ubQsk))
        opti.set_initial(Qs, gQs.to_numpy().T)
        if not guessType == 'quasiRandom':
            assert np.alltrue(lbQsk <= ca.vec(gQs.to_numpy().T).full()), (
                "Error lower bound joint position")
            assert np.alltrue(ubQsk >= ca.vec(gQs.to_numpy().T).full()), (
                "Error upper bound joint position")
        # Joint position at collocation points.
        Qs_col = opti.variable(nJoints, d*N)
        opti.subject_to(opti.bounded(lbQsj, ca.vec(Qs_col), ubQsj))
        opti.set_initial(Qs_col, gQsCol.to_numpy().T)
        if not guessType == 'quasiRandom':
            assert np.alltrue(lbQsj <= ca.vec(gQsCol.to_numpy().T).full()), (
                "Error lower bound joint position collocation points")
            assert np.alltrue(ubQsj >= ca.vec(gQsCol.to_numpy().T).full()), (
                "Error upper bound joint position collocation points")
        # Joint velocity at mesh points.
        Qds = opti.variable(nJoints, N+1)
        opti.subject_to(opti.bounded(lbQdsk, ca.vec(Qds), ubQdsk))
        opti.set_initial(Qds, gQds.to_numpy().T)
        assert np.alltrue(lbQdsk <= ca.vec(gQds.to_numpy().T).full()), (
            "Error lower bound joint velocity")
        assert np.alltrue(ubQdsk >= ca.vec(gQds.to_numpy().T).full()), (
            "Error upper bound joint velocity")
        # Joint velocity at collocation points.
        Qds_col = opti.variable(nJoints, d*N)
        opti.subject_to(opti.bounded(lbQdsj, ca.vec(Qds_col), ubQdsj))
        opti.set_initial(Qds_col, gQdsCol.to_numpy().T)
        assert np.alltrue(lbQdsj <= ca.vec(gQdsCol.to_numpy().T).full()), (
            "Error lower bound joint velocity collocation points")
        assert np.alltrue(ubQdsj >= ca.vec(gQdsCol.to_numpy().T).full()), (
            "Error upper bound joint velocity collocation points")
        # Arm activation at mesh points.
        aArm = opti.variable(nArmJoints, N+1)
        opti.subject_to(opti.bounded(lbArmAk, ca.vec(aArm), ubArmAk))
        opti.set_initial(aArm, gArmA.to_numpy().T)
        assert np.alltrue(lbArmAk <= ca.vec(gArmA.to_numpy().T).full()), (
            "Error lower bound arm activation")
        assert np.alltrue(ubArmAk >= ca.vec(gArmA.to_numpy().T).full()), (
            "Error upper bound arm activation")
        # Arm activation at collocation points.
        aArm_col = opti.variable(nArmJoints, d*N)
        opti.subject_to(opti.bounded(lbArmAj, ca.vec(aArm_col), ubArmAj))
        opti.set_initial(aArm_col, gArmACol.to_numpy().T)
        assert np.alltrue(lbArmAj <= ca.vec(gArmACol.to_numpy().T).full()), (
            "Error lower bound arm activation collocation points")
        assert np.alltrue(ubArmAj >= ca.vec(gArmACol.to_numpy().T).full()), (
            "Error upper bound arm activation collocation points")
        
        #######################################################################
        # Controls.
        # Muscle activation derivative at mesh points.
        aDt = opti.variable(nMuscles, N)
        opti.subject_to(opti.bounded(lbADtk, ca.vec(aDt), ubADtk))
        opti.set_initial(aDt, gADt.to_numpy().T)
        assert np.alltrue(lbADtk <= ca.vec(gADt.to_numpy().T).full()), (
            "Error lower bound muscle activation derivative")
        assert np.alltrue(ubADtk >= ca.vec(gADt.to_numpy().T).full()), (
            "Error upper bound muscle activation derivative")
        # Arm excitation at mesh points.
        eArm = opti.variable(nArmJoints, N)
        opti.subject_to(opti.bounded(lbArmEk, ca.vec(eArm), ubArmEk))
        opti.set_initial(eArm, gArmE.to_numpy().T)
        assert np.alltrue(lbArmEk <= ca.vec(gArmE.to_numpy().T).full()), (
            "Error lower bound arm excitation")
        assert np.alltrue(ubArmEk >= ca.vec(gArmE.to_numpy().T).full()), (
            "Error upper bound arm excitation")
        
        #######################################################################
        # Slack controls.
        # Muscle force derivative at collocation points.
        normFDt_col = opti.variable(nMuscles, d*N)
        opti.subject_to(opti.bounded(lbFDtj, ca.vec(normFDt_col), ubFDtj))
        opti.set_initial(normFDt_col, gFDtCol.to_numpy().T)
        assert np.alltrue(lbFDtj <= ca.vec(gFDtCol.to_numpy().T).full()), (
            "Error lower bound muscle force derivative")
        assert np.alltrue(ubFDtj >= ca.vec(gFDtCol.to_numpy().T).full()), (
            "Error upper bound muscle force derivative")
        # Joint velocity derivative (acceleration) at collocation points.
        Qdds_col = opti.variable(nJoints, d*N)
        opti.subject_to(opti.bounded(lbQddsj, ca.vec(Qdds_col),
                                     ubQddsj))
        opti.set_initial(Qdds_col, gQddsCol.to_numpy().T)
        assert np.alltrue(lbQddsj <= ca.vec(gQddsCol.to_numpy().T).full()), (
            "Error lower bound joint velocity derivative")
        assert np.alltrue(ubQddsj >= ca.vec(gQddsCol.to_numpy().T).full()), (
            "Error upper bound joint velocity derivative")
            
        ####################################################################### 
        # Parallel formulation - initialize variables.
        # Static parameters.
        tf = ca.MX.sym('tf')
        # States.
        ak = ca.MX.sym('ak', nMuscles)
        aj = ca.MX.sym('aj', nMuscles, d)    
        akj = ca.horzcat(ak, aj)    
        normFk = ca.MX.sym('normFk', nMuscles)
        normFj = ca.MX.sym('normFj', nMuscles, d)
        normFkj = ca.horzcat(normFk, normFj)       
        Qsk = ca.MX.sym('Qsk', nJoints)
        Qsj = ca.MX.sym('Qsj', nJoints, d)
        Qskj = ca.horzcat(Qsk, Qsj)    
        Qdsk = ca.MX.sym('Qdsk', nJoints)
        Qdsj = ca.MX.sym('Qdsj', nJoints, d)
        Qdskj = ca.horzcat(Qdsk, Qdsj)    
        aArmk = ca.MX.sym('aArmk', nArmJoints)
        aArmj = ca.MX.sym('aArmj', nArmJoints, d)
        aArmkj = ca.horzcat(aArmk, aArmj)
        # Controls.
        aDtk = ca.MX.sym('aDtk', nMuscles)    
        eArmk = ca.MX.sym('eArmk', nArmJoints)
        # Slack controls.
        normFDtj = ca.MX.sym('normFDtj', nMuscles, d);
        Qddsj = ca.MX.sym('Qddsj', nJoints, d)
        
        #######################################################################
        # Time step.
        h = tf / N
        
        #######################################################################
        # Collocation matrices.
        tau = ca.collocation_points(d,'radau');
        [C,D] = ca.collocation_interpolators(tau);
        # Missing matrix B, add manually.
        B = [-8.88178419700125e-16, 0.376403062700467, 0.512485826188421, 
             0.111111111111111]
        
        #######################################################################
        # Initialize cost function and constraint vectors.
        J = 0
        eq_constr = []
        ineq_constr1 = []
        ineq_constr2 = []
        ineq_constr3 = []
        ineq_constr4 = []
        ineq_constr5 = [] 
        ineq_constr6 = [] 
            
        #######################################################################
        # Loop over collocation points.
        for j in range(d):
            ###################################################################
            # Unscale variables.
            # States.
            normFkj_nsc = normFkj * (scalingF.to_numpy().T * np.ones((1, d+1)))
            Qskj_nsc = Qskj * (scalingQs.to_numpy().T * np.ones((1, d+1)))
            Qdskj_nsc = Qdskj * (scalingQds.to_numpy().T * np.ones((1, d+1)))
            # Controls.
            aDtk_nsc = aDtk * (scalingADt.to_numpy().T)
            # Slack controls.
            normFDtj_nsc = normFDtj * (
                scalingFDt.to_numpy().T * np.ones((1, d)))
            Qddsj_nsc = Qddsj * (scalingQdds.to_numpy().T * np.ones((1, d))) 
            # Qs and Qds are intertwined in external function.
            QsQdskj_nsc = ca.MX(nJoints*2, d+1)
            QsQdskj_nsc[::2, :] = Qskj_nsc
            QsQdskj_nsc[1::2, :] = Qdskj_nsc   
            
            ###################################################################
            # Polynomial approximations.
            # Left side.
            Qsinj_l = Qskj_nsc[leftPolynomialJointIndices, j+1]
            Qdsinj_l = Qdskj_nsc[leftPolynomialJointIndices, j+1]
            [lMTj_l, vMTj_l, dMj_l] = f_polynomial(Qsinj_l, Qdsinj_l)
            # Right side.
            Qsinj_r = Qskj_nsc[rightPolynomialJointIndices, j+1]
            Qdsinj_r = Qdskj_nsc[rightPolynomialJointIndices, j+1]
            [lMTj_r, vMTj_r, dMj_r] = f_polynomial(Qsinj_r, Qdsinj_r)
            # Muscle-tendon lengths and velocities.        
            lMTj_lr = ca.vertcat(lMTj_l[leftPolynomialMuscleIndices], 
                                 lMTj_r[rightPolynomialMuscleIndices])
            vMTj_lr = ca.vertcat(vMTj_l[leftPolynomialMuscleIndices], 
                                 vMTj_r[rightPolynomialMuscleIndices])
            # Moment arms.
            dMj = {}
            # Left side.
            for joint in leftPolynomialJoints:
                if ((joint != 'mtp_angle_l') and 
                    (joint != 'lumbar_extension') and
                    (joint != 'lumbar_bending') and 
                    (joint != 'lumbar_rotation')):
                        dMj[joint] = dMj_l[momentArmIndices[joint], 
                                           leftPolynomialJoints.index(joint)]
            # Right side.
            for joint in rightPolynomialJoints:
                if ((joint != 'mtp_angle_r') and 
                    (joint != 'lumbar_extension') and
                    (joint != 'lumbar_bending') and 
                    (joint != 'lumbar_rotation')):
                        # We need to adjust momentArmIndices for the right side
                        # since the polynomial indices are 'one-sided'. We 
                        # subtract by the number of side muscles.
                        c_ma = [
                            i - nSideMuscles for i in momentArmIndices[joint]]
                        dMj[joint] = dMj_r[c_ma,
                                           rightPolynomialJoints.index(joint)]
            # Trunk.
            for joint in trunkJoints:
                dMj[joint] = dMj_l[trunkMomentArmPolynomialIndices, 
                                   leftPolynomialJoints.index(joint)]            
            
            ###################################################################
            # Hill-equilibrium.       
            [hillEquilibriumj, Fj, activeFiberForcej, passiveFiberForcej,
             normActiveFiberLengthForcej, normFiberLengthj, fiberVelocityj] = (
             f_hillEquilibrium(akj[:, j+1], lMTj_lr, vMTj_lr, 
                               normFkj_nsc[:, j+1], normFDtj_nsc[:, j])) 
            
            ###################################################################
            # Metabolic energy rate.
            metabolicEnergyRatej = f_metabolicsBhargava(
                akj[:, j+1], akj[:, j+1], normFiberLengthj, fiberVelocityj, 
                activeFiberForcej, passiveFiberForcej, 
                normActiveFiberLengthForcej)[5]
            
            ###################################################################
            # Passive joint torques.
            passiveTorque_j = {}
            passiveTorquesj = ca.MX(nPassiveTorqueJoints, 1)
            for cj, joint in enumerate(passiveTorqueJoints):
                passiveTorque_j[joint] = f_passiveTorque[joint](
                    Qskj_nsc[joints.index(joint), j+1], 
                    Qdskj_nsc[joints.index(joint), j+1])
                passiveTorquesj[cj, 0] = passiveTorque_j[joint]
                
            linearPassiveTorqueArms_j = {}
            for joint in armJoints:
                linearPassiveTorqueArms_j[joint] = f_linearPassiveArmTorque(
                    Qskj_nsc[joints.index(joint), j+1],
                    Qdskj_nsc[joints.index(joint), j+1])
                
            linearPassiveTorqueMtp_j = {}
            for joint in mtpJoints:
                linearPassiveTorqueMtp_j[joint] = f_linearPassiveMtpTorque(
                    Qskj_nsc[joints.index(joint), j+1],
                    Qdskj_nsc[joints.index(joint), j+1])
            
            ###################################################################
            # Cost function.
            metEnergyRateTerm = (f_NMusclesSum2(metabolicEnergyRatej) / 
                                       modelMass)
            activationTerm = f_NMusclesSum2(akj[:, j+1])
            armExcitationTerm = f_nArmJointsSum2(eArmk)             
            jointAccelerationTerm = (
                    f_nNoArmJointsSum2(Qddsj[idxNoArmJoints, j]))                
            passiveTorqueTerm = (
                    f_nPassiveTorqueJointsSum2(passiveTorquesj))       
            activationDtTerm = f_NMusclesSum2(aDtk)
            forceDtTerm = f_NMusclesSum2(normFDtj[:, j])
            armAccelerationTerm = f_nArmJointsSum2(Qddsj[idxArmJoints, j])
            
            J += ((weights['metabolicEnergyRateTerm'] * metEnergyRateTerm +
                   weights['activationTerm'] * activationTerm + 
                   weights['armExcitationTerm'] * armExcitationTerm + 
                   weights['jointAccelerationTerm'] * jointAccelerationTerm +                
                   weights['passiveTorqueTerm'] * passiveTorqueTerm + 
                   weights['controls'] * (forceDtTerm + activationDtTerm 
                          + armAccelerationTerm)) * h * B[j + 1])
            
            ###################################################################
            # Expression for the state derivatives at the collocation points.
            ap = ca.mtimes(akj, C[j+1])        
            normFp_nsc = ca.mtimes(normFkj_nsc, C[j+1])
            Qsp_nsc = ca.mtimes(Qskj_nsc, C[j+1])
            Qdsp_nsc = ca.mtimes(Qdskj_nsc, C[j+1])        
            aArmp = ca.mtimes(aArmkj, C[j+1])
            # Append collocation equations.
            # Muscle activation dynamics (implicit formulation).
            eq_constr.append((h*aDtk_nsc - ap))
            # Muscle contraction dynamics (implicit formulation)  .
            eq_constr.append((h*normFDtj_nsc[:, j] - normFp_nsc) / 
                            scalingF.to_numpy().T)
            # Skeleton dynamics (implicit formulation).
            # Position derivatives.
            eq_constr.append((h*Qdskj_nsc[:, j+1] - Qsp_nsc) / 
                            scalingQs.to_numpy().T)
            # Velocity derivatives.
            eq_constr.append((h*Qddsj_nsc[:, j] - Qdsp_nsc) / 
                            scalingQds.to_numpy().T)
            # Arm activation dynamics (explicit formulation).
            aArmDtj = f_armActivationDynamics(eArmk, aArmkj[:, j+1])
            eq_constr.append(h*aArmDtj - aArmp)
            
            ###################################################################
            # Path constraints.
            # Call external function (run inverse dynamics).
            Tj = F(ca.vertcat(QsQdskj_nsc[:, j+1], Qddsj_nsc[:, j]))
            
            ###################################################################
            # Null pelvis residuals.
            eq_constr.append(Tj[idxGroundPelvisJoints, 0])
            
            ###################################################################
            # Muscle-driven joint torques.
            for joint in muscleDrivenJoints:                
                FJ_joint = Fj[momentArmIndices[joint]]
                mTj_joint = ca.sum1(dMj[joint]*FJ_joint) 
                diffTj_joint = f_diffTorques(
                    Tj[joints.index(joint)], mTj_joint, passiveTorque_j[joint])
                eq_constr.append(diffTj_joint)
            
            ###################################################################
            # Torque-driven joint torques (arm joints).
            for cj, joint in enumerate(armJoints):
                diffTJ_joint = f_diffTorques(
                    Tj[joints.index(joint)] /
                    scalingArmE.iloc[0][joint],
                    aArmkj[cj, j+1], linearPassiveTorqueArms_j[joint] /
                    scalingArmE.iloc[0][joint])
                eq_constr.append(diffTJ_joint)
                
            ###################################################################
            # Passive joint torques (mtp joints).
            for joint in mtpJoints:
                diffTj_joint = f_diffTorques(
                    Tj[joints.index(joint)] / 
                    scalingMtpE.iloc[0][joint],
                    0, 
                    (passiveTorque_j[joint] + 
                     linearPassiveTorqueMtp_j[joint]) /
                    scalingMtpE.iloc[0][joint])
                eq_constr.append(diffTj_joint)          
            
            ###################################################################
            # Activation dynamics (implicit formulation).
            act1 = aDtk_nsc + akj[:, j+1] / deactivationTimeConstant
            act2 = aDtk_nsc + akj[:, j+1] / activationTimeConstant
            ineq_constr1.append(act1)
            ineq_constr2.append(act2)
            
            ###################################################################
            # Contraction dynamics (implicit formulation).
            eq_constr.append(hillEquilibriumj)
            
            ###################################################################
            # Prevent collision between body parts.
            diffCalcOrs = ca.sumsqr(Tj[idxCalcOr_r] - Tj[idxCalcOr_l])
            ineq_constr3.append(diffCalcOrs)
            diffFemurHandOrs_r = ca.sumsqr(Tj[idxFemurOr_r] - Tj[idxHandOr_r])
            ineq_constr4.append(diffFemurHandOrs_r)
            diffFemurHandOrs_l = ca.sumsqr(Tj[idxFemurOr_l] - Tj[idxHandOr_l])
            ineq_constr4.append(diffFemurHandOrs_l)
            diffTibiaOrs = ca.sumsqr(Tj[idxTibiaOr_r] - Tj[idxTibiaOr_l])
            ineq_constr5.append(diffTibiaOrs)
            diffToesOrs = ca.sumsqr(Tj[idxToesOr_r] - Tj[idxToesOr_l])
            ineq_constr6.append(diffToesOrs)
        # End loop over collocation points.
        
        #######################################################################
        # Flatten constraint vectors
        eq_constr = ca.vertcat(*eq_constr)
        ineq_constr1 = ca.vertcat(*ineq_constr1)
        ineq_constr2 = ca.vertcat(*ineq_constr2)
        ineq_constr3 = ca.vertcat(*ineq_constr3)
        ineq_constr4 = ca.vertcat(*ineq_constr4)
        ineq_constr5 = ca.vertcat(*ineq_constr5)
        ineq_constr6 = ca.vertcat(*ineq_constr6)
        # Create function for map construct (parallel computing).
        f_coll = ca.Function('f_coll', [tf, ak, aj, normFk, normFj, Qsk, 
                                        Qsj, Qdsk, Qdsj, aArmk, aArmj,
                                        aDtk, eArmk, normFDtj, Qddsj], 
                [eq_constr, ineq_constr1, ineq_constr2, ineq_constr3, 
                 ineq_constr4, ineq_constr5, ineq_constr6, J])     
        # Create map construct (N mesh intervals).
        f_coll_map = f_coll.map(N, parallelMode, nThreads)   
        # Call function with opti variables and set constraints.
        (coll_eq_constr, coll_ineq_constr1, coll_ineq_constr2,
         coll_ineq_constr3, coll_ineq_constr4, coll_ineq_constr5,
         coll_ineq_constr6, Jall) = f_coll_map(
             finalTime, a[:, :-1], a_col, normF[:, :-1], normF_col, 
             Qs[:, :-1], Qs_col, Qds[:, :-1], Qds_col, 
             aArm[:, :-1], aArm_col, aDt, eArm, normFDt_col, Qdds_col)
        # Set constraints.    
        opti.subject_to(ca.vec(coll_eq_constr) == 0)
        opti.subject_to(ca.vec(coll_ineq_constr1) >= 0)
        opti.subject_to(
            ca.vec(coll_ineq_constr2) <= 1 / activationTimeConstant)    
        opti.subject_to(opti.bounded(0.0081, ca.vec(coll_ineq_constr3), 4))
        opti.subject_to(opti.bounded(0.0324 , ca.vec(coll_ineq_constr4), 4))
        opti.subject_to(opti.bounded(0.0121, ca.vec(coll_ineq_constr5), 4))
        opti.subject_to(opti.bounded(0.01, ca.vec(coll_ineq_constr6), 4))
                
        #######################################################################
        # Equality / continuity constraints.
        # Loop over mesh points.
        for k in range(N):
            akj2 = (ca.horzcat(a[:, k], a_col[:, k*d:(k+1)*d]))
            normFkj2 = (ca.horzcat(normF[:, k], normF_col[:, k*d:(k+1)*d]))
            Qskj2 = (ca.horzcat(Qs[:, k], Qs_col[:, k*d:(k+1)*d]))
            Qdskj2 = (ca.horzcat(Qds[:, k], Qds_col[:, k*d:(k+1)*d]))    
            aArmkj2 = (ca.horzcat(aArm[:, k], aArm_col[:, k*d:(k+1)*d]))
            
            opti.subject_to(a[:, k+1] == ca.mtimes(akj2, D))
            opti.subject_to(normF[:, k+1] == ca.mtimes(normFkj2, D))    
            opti.subject_to(Qs[:, k+1] == ca.mtimes(Qskj2, D))
            opti.subject_to(Qds[:, k+1] == ca.mtimes(Qdskj2, D))    
            opti.subject_to(aArm[:, k+1] == ca.mtimes(aArmkj2, D)) 
            
        #######################################################################
        # Periodic constraints on states.
        # Joint positions and velocities.
        opti.subject_to(Qs[idxPeriodicQsJointsA ,-1] - 
                        Qs[idxPeriodicQsJointsB, 0] == 0)
        opti.subject_to(Qds[idxPeriodicQdsJointsA ,-1] - 
                        Qds[idxPeriodicQdsJointsB, 0] == 0)
        opti.subject_to(Qs[idxPeriodicOppositeJoints ,-1] + 
                        Qs[idxPeriodicOppositeJoints, 0] == 0)
        opti.subject_to(Qds[idxPeriodicOppositeJoints ,-1] + 
                        Qds[idxPeriodicOppositeJoints, 0] == 0)
        # Muscle activations.
        opti.subject_to(a[:, -1] - a[idxPeriodicMuscles, 0] == 0)
        # Muscle-tendon forces.
        opti.subject_to(normF[:, -1] - normF[idxPeriodicMuscles, 0] == 0)
        # Arm activations.
        opti.subject_to(aArm[:, -1] - aArm[idxPeriodicArmJoints, 0] == 0)
        
        #######################################################################
        # Average speed constraint.
        Qs_nsc = Qs * (scalingQs.to_numpy().T * np.ones((1, N+1)))
        distTraveled =  (Qs_nsc[joints.index('pelvis_tx'), -1] - 
                                Qs_nsc[joints.index('pelvis_tx'), 0])
        averageSpeed = distTraveled / finalTime
        opti.subject_to(averageSpeed - targetSpeed == 0)
        
        #######################################################################
        # Scale cost function with distance traveled.
        Jall_sc = ca.sum2(Jall)/distTraveled  
        
        #######################################################################
        # Create NLP solver.
        opti.minimize(Jall_sc)
                
        #######################################################################
        # Solve problem.
        from variousFunctions import solve_with_bounds
        w_opt, stats = solve_with_bounds(opti, tol)
        if saveResults:               
            np.save(os.path.join(pathResults, 'w_opt.npy'), w_opt)
            np.save(os.path.join(pathResults, 'stats.npy'), stats)
        
    # %% Analyze results.
    if analyzeResults:
        if loadResults:
            w_opt = np.load(os.path.join(pathResults, 'w_opt.npy'))
            stats = np.load(os.path.join(pathResults, 'stats.npy'), 
                            allow_pickle=True).item()  
            
        # %% Extractoptimal results.
        NParameters = 1    
        finalTime_opt = w_opt[:NParameters]
        starti = NParameters    
        a_opt = (np.reshape(w_opt[starti:starti+nMuscles*(N+1)],
                                  (N+1, nMuscles))).T
        starti = starti + nMuscles*(N+1)
        a_col_opt = (np.reshape(w_opt[starti:starti+nMuscles*(d*N)],
                                      (d*N, nMuscles))).T    
        starti = starti + nMuscles*(d*N)
        normF_opt = (np.reshape(w_opt[starti:starti+nMuscles*(N+1)],
                                      (N+1, nMuscles))  ).T  
        starti = starti + nMuscles*(N+1)
        normF_col_opt = (np.reshape(w_opt[starti:starti+nMuscles*(d*N)],
                                          (d*N, nMuscles))).T
        starti = starti + nMuscles*(d*N)
        Qs_opt = (np.reshape(w_opt[starti:starti+nJoints*(N+1)],
                                   (N+1, nJoints))  ).T  
        starti = starti + nJoints*(N+1)    
        Qs_col_opt = (np.reshape(w_opt[starti:starti+nJoints*(d*N)],
                                       (d*N, nJoints))).T
        starti = starti + nJoints*(d*N)
        Qds_opt = (np.reshape(w_opt[starti:starti+nJoints*(N+1)],
                                      (N+1, nJoints)) ).T   
        starti = starti + nJoints*(N+1)    
        Qds_col_opt = (np.reshape(w_opt[starti:starti+nJoints*(d*N)],
                                          (d*N, nJoints))).T
        starti = starti + nJoints*(d*N)    
        aArm_opt = (np.reshape(w_opt[starti:starti+nArmJoints*(N+1)],
                                     (N+1, nArmJoints))).T
        starti = starti + nArmJoints*(N+1)    
        aArm_col_opt = (np.reshape(w_opt[starti:starti+nArmJoints*(d*N)],
                                         (d*N, nArmJoints))).T
        starti = starti + nArmJoints*(d*N)
        aDt_opt = (np.reshape(w_opt[starti:starti+nMuscles*N],
                              (N, nMuscles))).T
        starti = starti + nMuscles*N
        eArm_opt = (np.reshape(w_opt[starti:starti+nArmJoints*N],
                               (N, nArmJoints))).T
        starti = starti + nArmJoints*N 
        normFDt_col_opt = (np.reshape(w_opt[starti:starti+nMuscles*(d*N)],
                                            (d*N, nMuscles))).T
        starti = starti + nMuscles*(d*N)
        Qdds_col_opt = (np.reshape(w_opt[starti:starti+nJoints*(d*N)],
                                             (d*N, nJoints))).T
        starti = starti + nJoints*(d*N)
        assert (starti == w_opt.shape[0]), "error when extracting results"
            
        # %% Unscale results.
        normF_opt_nsc = normF_opt * (scalingF.to_numpy().T * np.ones((1, N+1)))
        normF_col_opt_nsc = (
            normF_col_opt * (scalingF.to_numpy().T * np.ones((1, d*N))))  
        Qs_opt_nsc = Qs_opt * (scalingQs.to_numpy().T * np.ones((1, N+1)))
        Qs_col_opt_nsc = (
            Qs_col_opt * (scalingQs.to_numpy().T * np.ones((1, d*N))))
        Qds_opt_nsc = Qds_opt * (scalingQds.to_numpy().T * np.ones((1, N+1)))
        Qds_col_opt_nsc = (
            Qds_col_opt * (scalingQds.to_numpy().T * np.ones((1, d*N))))
        aDt_opt_nsc = aDt_opt * (scalingADt.to_numpy().T * np.ones((1, N)))
        Qdds_col_opt_nsc = (
            Qdds_col_opt * (scalingQdds.to_numpy().T * np.ones((1, d*N))))
        normFDt_col_opt_nsc = (
            normFDt_col_opt * (scalingFDt.to_numpy().T * np.ones((1, d*N))))
        normFDt_opt_nsc = normFDt_col_opt_nsc[:,d-1::d]
        aArm_opt_nsc = aArm_opt * scalingArmE.iloc[0]['arm_rot_r']
        
        # %% Assert target speed.
        distTraveled_opt = (Qs_opt_nsc[joints.index('pelvis_tx'), -1] - 
                            Qs_opt_nsc[joints.index('pelvis_tx'), 0])
        averageSpeed = distTraveled_opt / finalTime_opt
        assert (np.abs(averageSpeed - targetSpeed) < 10**(-tol)), (
            "Error: Target speed constraint not satisfied")
        
        # %% Extract joint torques.
        # linearpassiveTorque_mtp_angle_l_opt = np.zeros((1, N+1))
        # linearpassiveTorque_mtp_angle_r_opt = np.zeros((1, N+1))        
        # linearpassiveTorque_arm_flex_l_opt = np.zeros((1, N+1))
        # linearpassiveTorque_arm_flex_r_opt = np.zeros((1, N+1))        
        # linearpassiveTorque_arm_add_l_opt = np.zeros((1, N+1))
        # linearpassiveTorque_arm_add_r_opt = np.zeros((1, N+1))        
        # linearpassiveTorque_arm_rot_l_opt = np.zeros((1, N+1))
        # linearpassiveTorque_arm_rot_r_opt = np.zeros((1, N+1))        
        # linearpassiveTorque_elbow_flex_l_opt = np.zeros((1, N+1))
        # linearpassiveTorque_elbow_flex_r_opt = np.zeros((1, N+1))
        
        # passiveTorque_mtp_angle_l_opt = np.zeros((1, N+1))
        # passiveTorque_mtp_angle_r_opt = np.zeros((1, N+1))
        
        linearPassiveTorqueArms_opt = np.zeros((nArmJoints, N+1))        
        for k in range(N+1):
            for cj, joint in armJoints:
                linearPassiveTorqueArms_opt[cj, k] = f_linearPassiveArmTorque(
                    Qs_opt_nsc[joints.index(joint), k],
                    Qds_opt_nsc[joints.index(joint), k])
                
        linearPassiveTorqueMtp_opt = np.zeros((nMtpJoints, N+1))        
        for k in range(N+1):
            for cj, joint in mtpJoints:
                linearPassiveTorqueMtp_opt[cj, k] = f_linearPassiveMtpTorque(
                    Qs_opt_nsc[joints.index(joint), k],
                    Qds_opt_nsc[joints.index(joint), k])
                
        passiveTorqueMtp_opt = np.zeros((nMtpJoints, N+1))
        for k in range(N+1):
            for cj, joint in mtpJoints:
                passiveTorqueMtp_opt[cj, k] = f_passiveTorque[joint](
                    Qs_opt_nsc[joints.index(joint), k], 
                    Qds_opt_nsc[joints.index(joint), k])
            
            
            # linearpassiveTorque_mtp_angle_l_opt[0, k] = f_linearPassiveMtpTorque(
            #         Qs_opt_nsc[joints.index('mtp_angle_l'), k],
            #         Qds_opt_nsc[joints.index('mtp_angle_l'), k])
            # linearpassiveTorque_mtp_angle_r_opt[0, k] = f_linearPassiveMtpTorque(
            #         Qs_opt_nsc[joints.index('mtp_angle_r'), k],
            #         Qds_opt_nsc[joints.index('mtp_angle_r'), k])            
            # linearpassiveTorque_arm_flex_l_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_flex_l'), k],
            #         Qds_opt_nsc[joints.index('arm_flex_l'), k])
            # linearpassiveTorque_arm_flex_r_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_flex_r'), k],
            #         Qds_opt_nsc[joints.index('arm_flex_r'), k])            
            # linearpassiveTorque_arm_add_l_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_add_l'), k],
            #         Qds_opt_nsc[joints.index('arm_add_l'), k])
            # linearpassiveTorque_arm_add_r_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_add_r'), k],
            #         Qds_opt_nsc[joints.index('arm_add_r'), k])            
            # linearpassiveTorque_arm_rot_l_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_rot_l'), k],
            #         Qds_opt_nsc[joints.index('arm_rot_l'), k])
            # linearpassiveTorque_arm_rot_r_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('arm_rot_r'), k],
            #         Qds_opt_nsc[joints.index('arm_rot_r'), k])            
            # linearpassiveTorque_elbow_flex_l_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('elbow_flex_l'), k],
            #         Qds_opt_nsc[joints.index('elbow_flex_l'), k])
            # linearpassiveTorque_elbow_flex_r_opt[0, k] = f_linearPassiveArmTorque(
            #         Qs_opt_nsc[joints.index('elbow_flex_r'), k],
            #         Qds_opt_nsc[joints.index('elbow_flex_r'), k])
            
            # passiveTorque_mtp_angle_l_opt[0, k] = f_passiveTorque['mtp_angle_l'](
            #         Qs_opt_nsc[joints.index('mtp_angle_l'), k],
            #         Qds_opt_nsc[joints.index('mtp_angle_l'), k])
            # passiveTorque_mtp_angle_r_opt[0, k] = f_passiveTorque['mtp_angle_r'](
            #         Qs_opt_nsc[joints.index('mtp_angle_r'), k],
            #         Qds_opt_nsc[joints.index('mtp_angle_r'), k]) 
            
        # %% Extract ground reaction forces.
        # Also do some sanity checks with non muscle-driven joints.
        QsQds_opt_nsc = np.zeros((nJoints*2, N+1))
        QsQds_opt_nsc[::2, :] = Qs_opt_nsc
        QsQds_opt_nsc[1::2, :] = Qds_opt_nsc
        Qdds_opt = Qdds_col_opt_nsc[:,d-1::d]
        F1_out = np.zeros((NF1_out, N))
        armT = np.zeros((nArmJoints, N))
        mtpT = np.zeros((nMtpJoints, N))
        for k in range(N):    
            Tj = F1(ca.vertcat(QsQds_opt_nsc[:, k+1], Qdds_opt[:, k]))
            F1_out[:, k] = Tj.full().T
            
            for cj, joint in enumerate(armJoints):
                armT[cj, k] = f_diffTorques(
                    F1_out[joints.index(joint), k] / 
                    scalingArmE.iloc[0][joint],
                    aArm_opt[cj, k+1], 
                    linearPassiveTorqueArms_opt[cj, k+1] /
                    scalingArmE.iloc[0][joint])
                
            for cj, joint in enumerate(mtpJoints):
                mtpT[cj, k] = f_diffTorques(
                    F1_out[joints.index(joint), k] / 
                    scalingMtpE.iloc[0][joint],
                    0, 
                    (linearPassiveTorqueMtp_opt[cj, k+1] + 
                     passiveTorqueMtp_opt[cj, k+1]) /
                    scalingArmE.iloc[0][joint])
                           
            # armT[0, k] = f_diffTorques(F1_out[joints.index('arm_flex_l'), k] / 
            #         scalingArmE.iloc[0]['arm_flex_l'], aArm_opt[0, k+1], 
            #         linearpassiveTorque_arm_flex_l_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_flex_l'])                 
            # armT[1, k] = f_diffTorques(F1_out[joints.index('arm_add_l'), k] / 
            #         scalingArmE.iloc[0]['arm_add_l'], aArm_opt[1, k+1], 
            #         linearpassiveTorque_arm_add_l_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_add_l'])              
            # armT[2, k] = f_diffTorques(F1_out[joints.index('arm_rot_l'), k] / 
            #         scalingArmE.iloc[0]['arm_rot_l'], aArm_opt[2, k+1], 
            #         linearpassiveTorque_arm_rot_l_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_rot_l'])              
            # armT[3, k] = f_diffTorques(F1_out[joints.index('arm_flex_r'), k] / 
            #         scalingArmE.iloc[0]['arm_flex_r'], aArm_opt[3, k+1], 
            #         linearpassiveTorque_arm_flex_r_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_flex_r'])              
            # armT[4, k] = f_diffTorques(F1_out[joints.index('arm_add_r'), k] / 
            #         scalingArmE.iloc[0]['arm_add_r'], aArm_opt[4, k+1], 
            #         linearpassiveTorque_arm_add_r_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_add_r'])             
            # armT[5, k] = f_diffTorques(F1_out[joints.index('arm_rot_r'), k] / 
            #         scalingArmE.iloc[0]['arm_rot_r'], aArm_opt[5, k+1], 
            #         linearpassiveTorque_arm_rot_r_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['arm_rot_r'])              
            # armT[6, k] = f_diffTorques(F1_out[joints.index('elbow_flex_l'), k]/ 
            #         scalingArmE.iloc[0]['elbow_flex_l'], aArm_opt[6, k+1], 
            #         linearpassiveTorque_elbow_flex_l_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['elbow_flex_l'])             
            # armT[7, k] = f_diffTorques(F1_out[joints.index('elbow_flex_r'), k]/ 
            #         scalingArmE.iloc[0]['elbow_flex_r'], aArm_opt[7, k+1], 
            #         linearpassiveTorque_elbow_flex_r_opt[0, k+1] / 
            #         scalingArmE.iloc[0]['elbow_flex_r'])
            
            # mtpT[0, k] = f_diffTorques(F1_out[joints.index('mtp_angle_l'), k] / 
            #         scalingMtpE.iloc[0]['mtp_angle_l'], 0, 
            #         (linearpassiveTorque_mtp_angle_l_opt[0, k+1] + 
            #          passiveTorque_mtp_angle_l_opt[0, k+1])/
            #         scalingMtpE.iloc[0]['mtp_angle_l'])        
            # mtpT[1, k] = f_diffTorques(F1_out[joints.index('mtp_angle_r'), k] / 
            #         scalingMtpE.iloc[0]['mtp_angle_r'], 0, 
            #         (linearpassiveTorque_mtp_angle_r_opt[0, k+1] + 
            #          passiveTorque_mtp_angle_r_opt[0, k+1]) /
            #         scalingMtpE.iloc[0]['mtp_angle_r'])
            
        # Ground reaction forces.
        GRF_opt = F1_out[idxGRF, :]
        # Sanity checks.
        assert np.alltrue(np.abs(armT) < 10**(-tol)), (
            "Error arm torques balance")
        assert np.alltrue(np.abs(mtpT) < 10**(-tol)), (
            "error mtp torques balance")   
        
        # %% Decompose optimal cost.
        if decomposeCost:     
            # Missing matrix B, add manually (again in case only analyzing).
            B = [-8.88178419700125e-16, 0.376403062700467, 0.512485826188421, 
                 0.111111111111111]
            metabolicEnergyRateTerm_opt_all = 0
            activationTerm_opt_all = 0
            armExcitationTerm_opt_all = 0
            jointAccelerationTerm_opt_all = 0
            passiveTorqueTerm_opt_all = 0
            activationDtTerm_opt_all = 0
            forceDtTerm_opt_all = 0
            armAccelerationTerm_opt_all = 0
            h_opt = finalTime_opt / N
            for k in range(N):
                # States.
                akj_opt = (ca.horzcat(a_opt[:, k], a_col_opt[:, k*d:(k+1)*d]))
                normFkj_opt = (
                    ca.horzcat(normF_opt[:, k], normF_col_opt[:, k*d:(k+1)*d]))
                normFkj_opt_nsc = (
                    normFkj_opt * (scalingF.to_numpy().T * np.ones((1, d+1)))) 
                Qskj_opt = (
                    ca.horzcat(Qs_opt[:, k], Qs_col_opt[:, k*d:(k+1)*d]))
                Qskj_opt_nsc = (
                    Qskj_opt * (scalingQs.to_numpy().T * np.ones((1, d+1))))
                Qdskj_opt = (
                    ca.horzcat(Qds_opt[:, k], Qds_col_opt[:, k*d:(k+1)*d]))
                Qdskj_opt_nsc = (
                    Qdskj_opt * (scalingQds.to_numpy().T * np.ones((1, d+1))))
                # Controls.
                aDtk_opt = aDt_opt[:, k]
                aDtk_opt_nsc = aDt_opt_nsc[:, k]
                eArmk_opt = eArm_opt[:, k]
                # Slack controls.
                Qddsj_opt = Qdds_col_opt[:, k*d:(k+1)*d]
                Qddsj_opt_nsc = (
                    Qddsj_opt * (scalingQdds.to_numpy().T * np.ones((1, d))))
                normFDtj_opt = normFDt_col_opt[:, k*d:(k+1)*d] 
                normFDtj_opt_nsc = (
                    normFDtj_opt * (scalingFDt.to_numpy().T * np.ones((1, d))))
                # Qs and Qds are intertwined in external function.
                QsQdskj_opt_nsc = ca.DM(nJoints*2, d+1)
                QsQdskj_opt_nsc[::2, :] = Qskj_opt_nsc
                QsQdskj_opt_nsc[1::2, :] = Qdskj_opt_nsc
                
                
                
                for j in range(d):
                    # Passive joint torques.
                    passiveTorquesj_opt = np.zeros((nPassiveTorqueJoints, 1))
                    for cj, joint in enumerate(passiveTorqueJoints):
                        passiveTorquesj_opt[cj, 0] = f_passiveTorque[joint](
                            Qskj_opt_nsc[joints.index(joint), j+1], 
                            Qdskj_opt_nsc[joints.index(joint), j+1])


                               
                    # passiveTorque_hip_flexion_rj_opt = f_passiveTorque_hip_flexion(
                    #         Qskj_opt_nsc[joints.index('hip_flexion_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_flexion_r'), j+1])
                    # passiveTorque_hip_flexion_lj_opt = f_passiveTorque_hip_flexion(
                    #         Qskj_opt_nsc[joints.index('hip_flexion_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_flexion_l'), j+1])   
                    # passiveTorque_hip_adduction_rj_opt = f_passiveTorque_hip_adduction(
                    #         Qskj_opt_nsc[joints.index('hip_adduction_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_adduction_r'), j+1])
                    # passiveTorque_hip_adduction_lj_opt = f_passiveTorque_hip_adduction(
                    #         Qskj_opt_nsc[joints.index('hip_adduction_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_adduction_l'), j+1])   
                    # passiveTorque_hip_rotation_rj_opt = f_passiveTorque_hip_rotation(
                    #         Qskj_opt_nsc[joints.index('hip_rotation_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_rotation_r'), j+1])
                    # passiveTorque_hip_rotation_lj_opt = f_passiveTorque_hip_rotation(
                    #         Qskj_opt_nsc[joints.index('hip_rotation_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('hip_rotation_l'), j+1])   
                    # passiveTorque_knee_angle_rj_opt = f_passiveTorque_knee_angle(
                    #         Qskj_opt_nsc[joints.index('knee_angle_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('knee_angle_r'), j+1])
                    # passiveTorque_knee_angle_lj_opt = f_passiveTorque_knee_angle(
                    #         Qskj_opt_nsc[joints.index('knee_angle_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('knee_angle_l'), j+1])        
                    # passiveTorque_ankle_angle_rj_opt = f_passiveTorque_ankle_angle(
                    #         Qskj_opt_nsc[joints.index('ankle_angle_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('ankle_angle_r'), j+1])
                    # passiveTorque_ankle_angle_lj_opt = f_passiveTorque_ankle_angle(
                    #         Qskj_opt_nsc[joints.index('ankle_angle_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('ankle_angle_l'), j+1]) 
                    # passiveTorque_subtalar_angle_rj_opt = f_passiveTorque_subtalar_angle(
                    #         Qskj_opt_nsc[joints.index('subtalar_angle_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('subtalar_angle_r'), j+1])
                    # passiveTorque_subtalar_angle_lj_opt = f_passiveTorque_subtalar_angle(
                    #         Qskj_opt_nsc[joints.index('subtalar_angle_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('subtalar_angle_l'), j+1])
                    # passiveTorque_mtp_angle_rj_opt = f_passiveTorque_mtp_angle(
                    #         Qskj_opt_nsc[joints.index('mtp_angle_r'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('mtp_angle_r'), j+1])
                    # passiveTorque_mtp_angle_lj_opt = f_passiveTorque_mtp_angle(
                    #         Qskj_opt_nsc[joints.index('mtp_angle_l'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('mtp_angle_l'), j+1])
                    # passiveTorque_lumbar_extensionj_opt = (
                    #         f_passiveTorque_lumbar_extension(
                    #         Qskj_opt_nsc[joints.index('lumbar_extension'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('lumbar_extension'), j+1])) 
                    # passiveTorque_lumbar_bendingj_opt = (
                    #         f_passiveTorque_lumbar_bending(
                    #         Qskj_opt_nsc[joints.index('lumbar_bending'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('lumbar_bending'), j+1]))
                    # passiveTorque_lumbar_rotationj_opt = (
                    #         f_passiveTorque_lumbar_rotation(
                    #         Qskj_opt_nsc[joints.index('lumbar_rotation'), j+1], 
                    #         Qdskj_opt_nsc[joints.index('lumbar_rotation'), j+1]))                    
                    # passiveTorquesj_opt = ca.vertcat(
                    #     passiveTorque_hip_flexion_rj_opt,
                    #     passiveTorque_hip_flexion_lj_opt,
                    #     passiveTorque_hip_adduction_rj_opt,
                    #     passiveTorque_hip_adduction_lj_opt,
                    #     passiveTorque_hip_rotation_rj_opt,
                    #     passiveTorque_hip_rotation_lj_opt,
                    #     passiveTorque_knee_angle_rj_opt,
                    #     passiveTorque_knee_angle_lj_opt,
                    #     passiveTorque_ankle_angle_rj_opt,
                    #     passiveTorque_ankle_angle_lj_opt,
                    #     passiveTorque_subtalar_angle_rj_opt,
                    #     passiveTorque_subtalar_angle_lj_opt,
                    #     passiveTorque_lumbar_extensionj_opt,
                    #     passiveTorque_lumbar_bendingj_opt,
                    #     passiveTorque_lumbar_rotationj_opt,
                    #     passiveTorque_mtp_angle_rj_opt,
                    #     passiveTorque_mtp_angle_lj_opt,)
                    
                    ###########################################################
                    # Polynomial approximations
                    # Left leg
                    Qsinj_opt_l = Qskj_opt_nsc[leftPolynomialJointIndices, j+1]
                    Qdsinj_opt_l = Qdskj_opt_nsc[leftPolynomialJointIndices, j+1]
                    [lMTj_opt_l, vMTj_opt_l, _] = f_polynomial(Qsinj_opt_l, Qdsinj_opt_l)       
                    # Right leg
                    Qsinj_opt_r = Qskj_opt_nsc[rightPolynomialJointIndices, j+1]
                    Qdsinj_opt_r = Qdskj_opt_nsc[rightPolynomialJointIndices, j+1]
                    [lMTj_opt_r, vMTj_opt_r, _] = f_polynomial(Qsinj_opt_r, Qdsinj_opt_r)
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
                    jointAccelerationTerm_opt = f_nNoArmJointsSum2(Qddsj_opt[idxNoArmJoints, j])          
                    passiveTorqueTerm_opt = f_nPassiveTorqueJointsSum2(passiveTorquesj_opt)     
                    activationDtTerm_opt = f_NMusclesSum2(aDtk_opt)
                    forceDtTerm_opt = f_NMusclesSum2(normFDtj_opt[:, j])
                    armAccelerationTerm_opt = f_nArmJointsSum2(Qddsj_opt[idxArmJoints, j])
                    armExcitationTerm_opt = f_nArmJointsSum2(eArmk_opt) 
                    metabolicEnergyRateTerm_opt = (f_NMusclesSum2(metabolicEnergyRatej_opt) / modelMass)
                    
                    metabolicEnergyRateTerm_opt_all += weights['metabolicEnergyRateTerm'] * metabolicEnergyRateTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    activationTerm_opt_all += weights['activationTerm'] * activationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    armExcitationTerm_opt_all += weights['armExcitationTerm'] * armExcitationTerm_opt * h_opt * B[j + 1] / distTraveled_opt
                    jointAccelerationTerm_opt_all += weights['jointAccelerationTerm'] * jointAccelerationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    passiveTorqueTerm_opt_all += weights['passiveTorqueTerm'] * passiveTorqueTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    activationDtTerm_opt_all += weights['controls'] * activationDtTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    forceDtTerm_opt_all += weights['controls'] * forceDtTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
                    armAccelerationTerm_opt_all += weights['controls'] * armAccelerationTerm_opt * h_opt * B[j + 1] / distTraveled_opt 
        
        objective_terms = {"metabolicEnergyRateTerm": metabolicEnergyRateTerm_opt_all.full(),
                           "activationTerm": activationTerm_opt_all.full(),
                           "armExcitationTerm": armExcitationTerm_opt_all.full(),
                           "jointAccelerationTerm": jointAccelerationTerm_opt_all.full(),
                           "passiveTorqueTerm": passiveTorqueTerm_opt_all.full(),
                           "activationDtTerm": activationDtTerm_opt_all.full(),
                           "forceDtTerm": forceDtTerm_opt_all.full(),
                           "armAccelerationTerm": armAccelerationTerm_opt_all.full()} 
        objective_terms_nsc = {"metabolicEnergyRateTerm": metabolicEnergyRateTerm_opt_all.full() / weights['metabolicEnergyRateTerm'],
                               "activationTerm": activationTerm_opt_all.full() / weights['activationTerm'],
                               "armExcitationTerm": armExcitationTerm_opt_all.full() / weights['armExcitationTerm'],
                               "jointAccelerationTerm": jointAccelerationTerm_opt_all.full() / weights['jointAccelerationTerm'],
                               "passiveTorqueTerm": passiveTorqueTerm_opt_all.full() / weights['passiveTorqueTerm'],
                               "activationDtTerm": activationDtTerm_opt_all.full() / weights['controls'],
                               "forceDtTerm": forceDtTerm_opt_all.full() / weights['controls'],
                               "armAccelerationTerm": armAccelerationTerm_opt_all.full() / weights['controls']}
        
        JAll_opt = (metabolicEnergyRateTerm_opt_all.full() +
                     activationTerm_opt_all.full() + 
                     armExcitationTerm_opt_all.full() +
                     jointAccelerationTerm_opt_all.full() + 
                     passiveTorqueTerm_opt_all.full() + 
                     activationDtTerm_opt_all.full() + 
                     forceDtTerm_opt_all.full() + 
                     armAccelerationTerm_opt_all.full())
        objective_terms["mtpExcitationTerm"] = 0
        
        if stats['success'] == True:
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
        Qs_GC = np.zeros((nJoints, 2*N))
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
        Qds_GC = np.zeros((nJoints, 2*N))
        Qds_GC[:, :N-idxIC_s[0]] = Qds_opt_nsc[:, idxIC_s[0]:-1]
        Qds_GC[idxPeriodicQdsJointsA, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                Qds_opt_nsc[idxPeriodicQdsJointsB, :-1])
        Qds_GC[idxPeriodicQdsJointsA, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                Qds_opt_nsc[idxPeriodicQdsJointsB, :-1])
        Qds_GC[idxPeriodicOppositeJoints, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                -Qds_opt_nsc[idxPeriodicOppositeJoints, :-1])
        Qds_GC[:, N-idxIC_s[0]+N:2*N] = Qds_opt_nsc[:,:idxIC_s[0]] 
        if legIC == "left":
            Qds_GC[idxPeriodicQdsJointsA, :] = Qds_GC[idxPeriodicQdsJointsB, :]
            Qds_GC[idxPeriodicOppositeJoints, :] = -Qds_GC[idxPeriodicOppositeJoints, :]
        Qds_GC[idxRotationalJoints, :] = Qds_GC[idxRotationalJoints, :] * 180 / np.pi
        
        # Joint accelerations
        # TODO: don;t think it is correct.
        Qdds_GC = np.zeros((nJoints, 2*N))
        Qdds_GC[:, :N-idxIC_c[0]] = Qdds_opt[:, idxIC_c[0]:]
        Qdds_GC[idxPeriodicQdsJointsA, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                Qdds_opt[idxPeriodicQdsJointsB, :])
        Qdds_GC[idxPeriodicOppositeJoints, N-idxIC_c[0]:N-idxIC_c[0]+N] = (
                -Qdds_opt[idxPeriodicOppositeJoints, :])
        Qdds_GC[:, N-idxIC_c[0]+N:2*N] = Qdds_opt[:,:idxIC_c[0]] 
        if legIC == "left":
            Qdds_GC[idxPeriodicQdsJointsA, :] = Qdds_GC[idxPeriodicQdsJointsB, :]
            Qdds_GC[idxPeriodicOppositeJoints, :] = -Qdds_GC[idxPeriodicOppositeJoints, :]
        Qdds_GC[idxRotationalJoints, :] = Qdds_GC[idxRotationalJoints, :] * 180 / np.pi
        
        # Muscle activations
        A_GC = np.zeros((nMuscles, 2*N))
        A_GC[:, :N-idxIC_s[0]] = a_opt[:, idxIC_s[0]:-1]
        A_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = a_opt[idxPeriodicMuscles, :-1]
        A_GC[:, N-idxIC_s[0]+N:2*N] = a_opt[:,:idxIC_s[0]] 
        if legIC == "left":
            A_GC = A_GC[idxPeriodicMuscles, :]
            
        # Muscle force
        F_GC = np.zeros((nMuscles, 2*N))
        F_GC[:, :N-idxIC_s[0]] = normF_opt_nsc[:, idxIC_s[0]:-1]
        F_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = normF_opt_nsc[idxPeriodicMuscles, :-1]
        F_GC[:, N-idxIC_s[0]+N:2*N] = normF_opt_nsc[:,:idxIC_s[0]] 
        if legIC == "left":
            F_GC = F_GC[idxPeriodicMuscles, :]
            
        # Muscle force derivative
        FDt_GC = np.zeros((nMuscles, 2*N))
        FDt_GC[:, :N-idxIC_c[0]] = normFDt_opt_nsc[:, idxIC_c[0]:]
        FDt_GC[:, N-idxIC_c[0]:N-idxIC_c[0]+N] = normFDt_opt_nsc[idxPeriodicMuscles, :]
        FDt_GC[:, N-idxIC_c[0]+N:2*N] = normFDt_opt_nsc[:,:idxIC_c[0]] 
        if legIC == "left":
            FDt_GC = FDt_GC[idxPeriodicMuscles, :]
            
        # Arm actuator activations
        aArm_GC = np.zeros((nArmJoints, 2*N))
        aArm_GC[:, :N-idxIC_s[0]] = aArm_opt_nsc[:, idxIC_s[0]:-1]
        aArm_GC[:, N-idxIC_s[0]:N-idxIC_s[0]+N] = (
                aArm_opt_nsc[idxPeriodicArmJoints, :-1])
        aArm_GC[:, N-idxIC_s[0]+N:2*N] = aArm_opt_nsc[:,:idxIC_s[0]] 
        if legIC == "left":
            aArm_GC = aArm_GC[idxPeriodicArmJoints, :]
            
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
        Qs_GC_rad = Qs_GC.copy()        
        Qs_GC_rad[idxRotationalJoints, :] = (
            Qs_GC_rad[idxRotationalJoints, :] * np.pi / 180)
        Qds_GC_rad = Qds_GC.copy()        
        Qds_GC_rad[idxRotationalJoints, :] = (
            Qds_GC_rad[idxRotationalJoints, :] * np.pi / 180)     
        basal_coef = 1.2
        basal_exp = 1    
        metabolicEnergyRatePerMuscle = np.zeros((nMuscles,2*N))
        totalMetabolicEnergyRate = np.zeros((1,2*N))
        activationHeatRate = np.zeros((1,2*N))
        maintenanceHeatRate = np.zeros((1,2*N))
        shorteningHeatRate = np.zeros((1,2*N))
        mechanicalWorkRate = np.zeros((1,2*N))
        normFiberLength_GC = np.zeros((nMuscles,2*N))
        fiberVelocity_GC = np.zeros((nMuscles,2*N))
        activationHeatRate_GC = np.zeros((nMuscles,2*N))
        maintenanceHeatRate_GC = np.zeros((nMuscles,2*N))
        shorteningHeatRate_GC = np.zeros((nMuscles,2*N))
        mechanicalWorkRate_GC = np.zeros((nMuscles,2*N))
        for k in range(2*N):
            ###################################################################
            # Polynomial approximations
            # Left leg
            Qsk_GC_l = Qs_GC_rad[leftPolynomialJointIndices, k]
            Qdsk_GC_l = Qds_GC_rad[leftPolynomialJointIndices, k]
            [lMTk_GC_l, vMTk_GC_l, _] = f_polynomial(Qsk_GC_l, Qdsk_GC_l)       
            # Right leg
            Qsk_GC_r = Qs_GC_rad[rightPolynomialJointIndices, k]
            Qdsk_GC_r = Qds_GC_rad[rightPolynomialJointIndices, k]
            [lMTk_GC_r, vMTk_GC_r, _] = f_polynomial(Qsk_GC_r, Qdsk_GC_r)
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
                 
            if stats['success'] == True:
                assert np.alltrue(
                    np.abs(hillEquilibriumk_GC.full()) <= 1e-4), "Hill_" + case
                 
            normFiberLength_GC[:,k] = normFiberLengthk_GC.full().flatten()
            fiberVelocity_GC[:,k] = fiberVelocityk_GC.full().flatten()
            
            ###################################################################
            # Get metabolic energy rate
            [activationHeatRatek_GC, maintenanceHeatRatek_GC, 
             shorteningHeatRatek_GC, mechanicalWorkRatek_GC, _, 
             metabolicEnergyRatek_GC] = f_metabolicsBhargava(
                 A_GC[:, k], A_GC[:, k], normFiberLengthk_GC, 
                 fiberVelocityk_GC, activeFiberForcek_GC,
                 passiveFiberForcek_GC, normActiveFiberLengthForcek_GC)
                 
            metabolicEnergyRatePerMuscle[:, k:k+1] = (
                metabolicEnergyRatek_GC.full())
                 
            # Sum over all muscles                     
            metabolicEnergyRatek_allMuscles = np.sum(
                metabolicEnergyRatek_GC.full())
            activationHeatRatek_allMuscles = np.sum(
                activationHeatRatek_GC.full())
            maintenanceHeatRatek_allMuscles = np.sum(
                maintenanceHeatRatek_GC.full())
            shorteningHeatRatek_allMuscles = np.sum(
                shorteningHeatRatek_GC.full())
            mechanicalWorkRatek_allMuscles = np.sum(
                mechanicalWorkRatek_GC.full())
            
            activationHeatRate_GC[:,k] = activationHeatRatek_GC.full().flatten()
            maintenanceHeatRate_GC[:,k] = maintenanceHeatRatek_GC.full().flatten()
            shorteningHeatRate_GC[:,k] = shorteningHeatRatek_GC.full().flatten()
            mechanicalWorkRate_GC[:,k] = mechanicalWorkRatek_GC.full().flatten()
            # Add basal rate
            basalRatek = basal_coef*modelMass**basal_exp
            totalMetabolicEnergyRate[0, k] = (metabolicEnergyRatek_allMuscles 
                                              + basalRatek) 
            activationHeatRate[0, k] = activationHeatRatek_allMuscles
            maintenanceHeatRate[0, k] = maintenanceHeatRatek_allMuscles
            shorteningHeatRate[0, k] = shorteningHeatRatek_allMuscles
            mechanicalWorkRate[0, k] = mechanicalWorkRatek_allMuscles     
            
        # Integrate
        metabolicEnergyRatePerMuscle_int = np.trapz(
            metabolicEnergyRatePerMuscle, tgrid_GC)
        totalMetabolicEnergyRate_int = np.trapz(totalMetabolicEnergyRate,
                                                tgrid_GC)
        activationHeatRate_int = np.trapz(activationHeatRate,
                                                tgrid_GC)
        maintenanceHeatRate_int = np.trapz(maintenanceHeatRate,
                                                tgrid_GC)
        shorteningHeatRate_int = np.trapz(shorteningHeatRate,
                                                tgrid_GC)
        mechanicalWorkRate_int = np.trapz(mechanicalWorkRate,
                                                tgrid_GC)
        # Total distance traveled
        distTraveled_opt_GC = (Qs_GC_rad[joints.index('pelvis_tx'),-1] - 
                               Qs_GC_rad[joints.index('pelvis_tx'),0])
        # Cost of transport (COT)
        COT_GC = totalMetabolicEnergyRate_int / modelMass / distTraveled_opt_GC
        COT_activation_GC = activationHeatRate_int / modelMass / distTraveled_opt_GC
        COT_maintenance_GC = maintenanceHeatRate_int / modelMass / distTraveled_opt_GC
        COT_shortening_GC = shorteningHeatRate_int / modelMass / distTraveled_opt_GC
        COT_mechanical_GC = mechanicalWorkRate_int / modelMass / distTraveled_opt_GC        
        COT_perMuscle_GC = metabolicEnergyRatePerMuscle_int / modelMass / distTraveled_opt_GC        
        
        # %% Compute stride length and width.
        QsQds_opt_nsc_GC = np.zeros((nJoints*2, N*2))
        QsQds_opt_nsc_GC[::2, :] = Qs_GC_rad
        QsQds_opt_nsc_GC[1::2, :] = Qds_GC_rad        
        Qdds_GC_rad = Qdds_GC.copy()
        Qdds_GC_rad[idxRotationalJoints, :] = (
            Qdds_GC_rad[idxRotationalJoints, :] * np.pi / 180)        
        F1_GC = np.zeros((NF1_out, N*2))
        for k_GC in range(N*2):
            Tk_GC = F1(ca.vertcat(QsQds_opt_nsc_GC[:,k_GC],
                                  Qdds_GC_rad[:, k_GC]))
            F1_GC[:, k_GC] = Tk_GC.full().T
        
        stride_length_GC = ca.norm_2(F1_GC[idxCalcOr3D_r, 0] - 
                                     F1_GC[idxCalcOr3D_r, -1]).full()[0][0]
        GRF_GC2 = F1_GC[idxGRF, :]
        GRM_GC2 = F1_GC[idxGRM, :]        
        
        # # 
        torques_GC2 = F1_GC[getJointIndices(joints, joints), :] 
        # temp = GRF_GC - GRF_GC2
        # temp2 = np.max(np.abs(torques_GC - torques_GC2))
        
        
        # %% Write GRF file for visualization in OpenSim GUI    
        # COP and free moment
        from variousFunctions import getCOP
        COPr_GC, freeTr_GC = getCOP(GRF_GC2[:3,:], GRM_GC2[:3,:])
        COPl_GC, freeTl_GC = getCOP(GRF_GC2[3:,:], GRM_GC2[3:,:])        
        COP_GC = np.concatenate((COPr_GC, COPl_GC))
        freeT_GC = np.concatenate((freeTr_GC, freeTl_GC))        
        
        if writeMotion:     
            
            # post-processing
            GRF_GC_toPrint = np.copy(GRF_GC2)
            COP_GC_toPrint = np.copy(COP_GC)
            freeT_GC_toPrint = np.copy(freeT_GC)
            
            idx_r = np.argwhere(GRF_GC_toPrint[1, :] < 30)
            for tr in range(idx_r.shape[0]):
                GRF_GC_toPrint[:3, idx_r[tr, 0]] = 0
                COP_GC_toPrint[:3, idx_r[tr, 0]] = 0
                freeT_GC_toPrint[:3, idx_r[tr, 0]] = 0
            idx_l = np.argwhere(GRF_GC_toPrint[4, :] < 30)
            for tl in range(idx_l.shape[0]):
                GRF_GC_toPrint[3:, idx_l[tl, 0]] = 0
                COP_GC_toPrint[3:, idx_l[tl, 0]] = 0
                freeT_GC_toPrint[3:, idx_l[tl, 0]] = 0            
            
            grf_cop_Labels = ['r_ground_force_vx', 'r_ground_force_vy', 'r_ground_force_vz',
                              'r_ground_force_px', 'r_ground_force_py', 'r_ground_force_pz',
                              'l_ground_force_vx', 'l_ground_force_vy', 'l_ground_force_vz',
                              'l_ground_force_px', 'l_ground_force_py', 'l_ground_force_pz']
            
            grmLabels = ['r_ground_torque_x', 'r_ground_torque_y', 'r_ground_torque_z',
                          'l_ground_torque_x', 'l_ground_torque_y', 'l_ground_torque_z']
                         
            grLabels = grf_cop_Labels + grmLabels      
            labels = ['time'] + grLabels
            data = np.concatenate(
                (tgrid_GC.T, GRF_GC_toPrint[:3,:].T, COP_GC_toPrint[:3,:].T, 
                 GRF_GC_toPrint[3:,:].T, COP_GC_toPrint[3:,:].T, 
                 freeT_GC_toPrint.T), axis=1)             
            from variousFunctions import numpy2storage
            numpy2storage(labels, data, os.path.join(pathResults,'GRF.mot'))
            
        # %% Save trajectories for further analysis
        if saveTrajectories: 
            if not os.path.exists(os.path.join(pathTrajectories,
                                               'optimaltrajectories.npy')): 
                    optimaltrajectories = {}
            else:  
                optimaltrajectories = np.load(
                        os.path.join(pathTrajectories,
                                     'optimaltrajectories.npy'),
                        allow_pickle=True)   
                optimaltrajectories = optimaltrajectories.item()  
                
            GC_percent = np.linspace(1, 100, 2*N)
            
            optimaltrajectories[case] = {
                                'coordinate_values': Qs_GC, 
                                'coordinate_speeds': Qds_GC, 
                                'coordinate_accelerations': Qdds_GC,
                                'muscle_activations': A_GC,
                                'arm_activations': aArm_GC,
                                'joint_torques': torques_GC2,
                                'GRF': GRF_GC2,
                                'time': tgrid_GC,
                                'norm_fiber_lengths': normFiberLength_GC,
                                'fiber_velocity': fiberVelocity_GC,
                                'joints': joints,
                                'muscles': bothSidesMuscles,
                                'mtp_joints': mtpJoints,
                                'GRF_labels': GRFNames,
                                'COT': COT_GC[0],
                                'COT_perMuscle_GC': COT_perMuscle_GC,
                                'GC_percent': GC_percent,
                                'objective': stats['iterations']['obj'][-1],
                                'objective_terms': objective_terms,
                                'objective_terms_nsc': objective_terms_nsc,
                                'iter_count': stats['iter_count'],
                                "stride_length_GC": stride_length_GC}              
            np.save(os.path.join(pathTrajectories, 'optimaltrajectories.npy'),
                    optimaltrajectories)
           
        # %% Error message
        if not stats['success'] == True:
            print("WARNING: PROBLEM DID NOT CONVERGE - " 
                  + stats['return_status']) 
            