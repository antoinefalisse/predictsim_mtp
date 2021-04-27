import os
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

# cases = [str(i) for i in range(44, 52)]
cases = ['55']

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
    
    if 'metabolicEnergyRateTerm' in settings[case]:
        weights['metabolicEnergyRateTerm'] = settings[case]['metabolicEnergyRateTerm']
        
    if 'activationTerm' in settings[case]:
        weights['activationTerm'] = settings[case]['activationTerm']

    # Other settings
    tol = settings[case]['tol']
    N = settings[case]['N']
    NThreads = 8
    d = 3
    parallelMode = "thread"
    contactConfiguration = settings[case]['contactConfiguration']
    guessType = settings[case]['guessType']
    targetSpeed = settings[case]['targetSpeed']
        
    idxSubject = "1"
    if 'idxSubject' in settings[case]:
        idxSubject = settings[case]['idxSubject'] 
    subject = 'subject' + idxSubject + '_no_mtp'
    
    polynomial_type = 'nominal'
    if 'polynomial_type' in settings[case]:
        polynomial_type = settings[case]['polynomial_type']
        
    shorterKneeExtMA = False
    if 'shorterKneeExtMA' in settings[case]:
        shorterKneeExtMA = settings[case]['shorterKneeExtMA']
        perc_shorter = 0
        if 'perc_shorter' in settings[case]:
            perc_shorter = settings[case]['perc_shorter'] / 100
    shorterKneeExtMT = False
    if 'shorterKneeExtMT' in settings[case]:
        shorterKneeExtMT = settings[case]['shorterKneeExtMT']
        perc_shorter = 0
        if 'perc_shorter' in settings[case]:
            perc_shorter = settings[case]['perc_shorter'] / 100 
    shorterKneePol = False
    if 'shorterKneePol' in settings[case]:
        shorterKneePol = settings[case]['shorterKneePol']
        perc_shorter = 0
        if 'perc_shorter' in settings[case]:
            perc_shorter = settings[case]['perc_shorter'] / 100
          
    # Paths
    pathMain = os.getcwd()
    pathData = os.path.join(pathMain, 'OpenSimModel', subject)
    if idxSubject == '1':
        pathModel = os.path.join(pathData, 'Model', subject + ".osim")
    elif idxSubject == '2':
        pathModel = os.path.join(pathData, 'Model', 'subject' + idxSubject + 
                                 '_withoutMTP_scaled.osim')        
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
    if subject == 'subject1_no_mtp':
        if contactConfiguration == 'generic':
            F = ca.external('F','PredSim_no_mtpPin_cm0.dll')
            if analyzeResults:
                F1 = ca.external('F','PredSim_no_mtpPin_pp_cm0.dll')
        elif contactConfiguration == 'specific':
            F = ca.external('F','PredSim_no_mtpPin_cm3.dll')
            if analyzeResults:
                F1 = ca.external('F','PredSim_no_mtpPin_pp_cm3.dll')
        elif contactConfiguration == 'generic_cm5':
            F = ca.external('F','PredSim_no_mtpPin_cm5.dll')
            if analyzeResults:
                F1 = ca.external('F','PredSim_no_mtpPin_pp_cm5.dll')
        elif contactConfiguration == 'generic_cm6':
            F = ca.external('F','PredSim_no_mtpPin_cm6.dll')
            if analyzeResults:
                F1 = ca.external('F','PredSim_no_mtpPin_pp_cm6.dll')
    elif subject == 'subject2_no_mtp':
        if contactConfiguration == 'generic':
            F = ca.external('F','s2_withoutMTP_ge.dll')
            if analyzeResults:
                F1 = ca.external('F','s2_withoutMTP_ge_pp.dll')
        elif contactConfiguration == 'generic_low':
            F = ca.external('F','s2_withoutMTP_gl.dll')
            if analyzeResults:
                F1 = ca.external('F','s2_withoutMTP_gl_pp.dll')
        elif contactConfiguration == 'specific':
            F = ca.external('F','s2_withoutMTP_ss.dll')
            if analyzeResults:
                F1 = ca.external('F','s2_withoutMTP_ss_pp.dll')
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
    
    if shorterKneeExtMA or shorterKneeExtMT or shorterKneePol:
        knee_extensors = ['rect_fem_r', 'vas_med_r', 'vas_int_r', 'vas_lat_r']
    
    from muscleData import getMTParameters
    sideMtParameters = getMTParameters(pathModel, rightSideMuscles,
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
    
    if not shorterKneeExtMT:
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
                                      'subject' + idxSubject,
                                      'subject' + idxSubject + '_MuscleAnalysis_')
            
    suffix_pol = '_old'  
    polynomialData = getPolynomialData(loadPolynomialData, pathMTParameters, 
                                       pathCoordinates, pathMuscleAnalysis,
                                       rightPolynomialJoints, muscles,
                                       suffix_pol)        
    if loadPolynomialData:
        polynomialData = polynomialData.item()
        
    suffix_pol = '_FK'    
    polynomialData_old = getPolynomialData(loadPolynomialData, pathMTParameters, 
                                       pathCoordinates, pathMuscleAnalysis,
                                       rightPolynomialJoints, muscles,
                                       suffix_pol)  
    if loadPolynomialData:
        polynomialData_old = polynomialData_old.item()
    
    NPolynomial = len(leftPolynomialJoints)
    f_polynomial = polynomialApproximation(muscles, polynomialData, NPolynomial)
    f_polynomial_old = polynomialApproximation(muscles, polynomialData_old, NPolynomial)
    leftPolynomialJointIndices = getJointIndices(joints, leftPolynomialJoints)
    rightPolynomialJointIndices = getJointIndices(joints, rightPolynomialJoints)
    
    # Test polynomials
    
    range_knee = np.linspace(-120, 10, 1000)
    range_knee *= (np.pi/180)
    Qdotsin_test = np.zeros((len(leftPolynomialJoints),))
    lMT_test = np.zeros((NSideMuscles+3, range_knee.shape[0]))
    vMT_test = np.zeros((NSideMuscles+3, range_knee.shape[0]))
    dM_test = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_knee.shape[0]))
    lMT_test_old = np.zeros((NSideMuscles+3, range_knee.shape[0]))
    vMT_test_old = np.zeros((NSideMuscles+3, range_knee.shape[0]))
    dM_test_old = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_knee.shape[0]))
    for i in range(range_knee.shape[0]):
        Qsin_test = np.zeros((len(leftPolynomialJoints),))
        Qsin_test[leftPolynomialJoints.index('knee_angle_l')] = range_knee[i]
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial(Qsin_test, Qdotsin_test)        
        lMT_test[:, i] = (c_lMT_test.full()).flatten()
        vMT_test[:, i] = (c_vMT_test.full()).flatten()
        dM_test [:,:,i] = (c_dM_test.full())
        
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial_old(Qsin_test, Qdotsin_test)        
        lMT_test_old[:, i] = (c_lMT_test.full()).flatten()
        vMT_test_old[:, i] = (c_vMT_test.full()).flatten()
        dM_test_old [:,:,i] = (c_dM_test.full())
    
    range_knee_deg = range_knee*180/np.pi
    import matplotlib.pyplot as plt  
    fig, axs = plt.subplots(7, 7, sharex=True)    
    fig.suptitle('Muscles')
    for i, ax in enumerate(axs.flat):
        if i < NSideMuscles:
            # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
            for case in cases:
                ax.plot(range_knee_deg, lMT_test[i,:], c='black')            
                ax.plot(range_knee_deg, lMT_test_old[i,:], c='orange')        
            ax.set_title(muscles[i])
            if i < NSideMuscles and i > 41:
                ax.set_xticks([-120,10])
                ax.set_xticklabels(['-120','10'])
                ax.set_xlabel('Knee angle (deg)')
            else:
                ax.set_xticks([-120,10]) 
                ax.set_xticklabels([])
            # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
            handles, labels = ax.get_legend_handles_labels()
            plt.legend(handles, labels, loc='upper right')
    plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    plt.setp(axs[:, 0], ylabel='(m)')
    fig.align_ylabels()
    
    test1 = lMT_test_old / lMT_test
        
    fig, axs = plt.subplots(7, 7, sharex=True)    
    fig.suptitle('Muscles')
    for i, ax in enumerate(axs.flat):
        if i < NSideMuscles:
            # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
            for case in cases:
                ax.plot(range_knee_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:], c='black')
                # ax.plot(range_knee_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.9, c='blue')    
                # ax.plot(range_knee_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.8, c='red')    
                # ax.plot(range_knee_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.7, c='green')    
                ax.plot(range_knee_deg, dM_test_old[i,leftPolynomialJoints.index('knee_angle_l'),:], c='orange')        
            ax.set_title(muscles[i])
            if i < NSideMuscles and i > 41:
                ax.set_xticks([-120,10])
                ax.set_xticklabels(['-120','10'])
                ax.set_xlabel('Knee angle (deg)')
            else:
                ax.set_xticks([-120,10]) 
                ax.set_xticklabels([]) 
            # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
            handles, labels = ax.get_legend_handles_labels()
            plt.legend(handles, labels, loc='upper right')
        
    # plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    plt.setp(axs[:, 0], ylabel='(m)')
    fig.align_ylabels()
    
    test2 = dM_test_old[:,leftPolynomialJoints.index('knee_angle_l'),:] / dM_test[:,leftPolynomialJoints.index('knee_angle_l'),:]
    
    # %% Ankle
    
    # Test polynomials
    
    range_ankle = np.linspace(-50, 40, 1000)
    range_ankle *= (np.pi/180)
    Qdotsin_test = np.zeros((len(leftPolynomialJoints),))
    lMT_test = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    vMT_test = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    dM_test = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_ankle.shape[0]))
    lMT_test_old = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    vMT_test_old = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    dM_test_old = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_ankle.shape[0]))
    for i in range(range_ankle.shape[0]):
        Qsin_test = np.zeros((len(leftPolynomialJoints),))
        Qsin_test[leftPolynomialJoints.index('ankle_angle_l')] = range_ankle[i]
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial(Qsin_test, Qdotsin_test)        
        lMT_test[:, i] = (c_lMT_test.full()).flatten()
        vMT_test[:, i] = (c_vMT_test.full()).flatten()
        dM_test [:,:,i] = (c_dM_test.full())
        
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial_old(Qsin_test, Qdotsin_test)        
        lMT_test_old[:, i] = (c_lMT_test.full()).flatten()
        vMT_test_old[:, i] = (c_vMT_test.full()).flatten()
        dM_test_old [:,:,i] = (c_dM_test.full())
    
    range_ankle_deg = range_ankle*180/np.pi
    import matplotlib.pyplot as plt  
    fig, axs = plt.subplots(7, 7, sharex=True)    
    fig.suptitle('Muscles')
    for i, ax in enumerate(axs.flat):
        if i < NSideMuscles:
            # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
            for case in cases:
                ax.plot(range_ankle_deg, lMT_test[i,:], c='black')            
                ax.plot(range_ankle_deg, lMT_test_old[i,:], c='orange')        
            ax.set_title(muscles[i])
            if i < NSideMuscles and i > 41:
                ax.set_xticks([-120,10])
                ax.set_xticklabels(['-120','10'])
                ax.set_xlabel('Knee angle (deg)')
            else:
                ax.set_xticks([-120,10]) 
                ax.set_xticklabels([])
            # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
            handles, labels = ax.get_legend_handles_labels()
            plt.legend(handles, labels, loc='upper right')
    plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    plt.setp(axs[:, 0], ylabel='(m)')
    fig.align_ylabels()
    
    test1 = lMT_test_old / lMT_test
        
    fig, axs = plt.subplots(7, 7, sharex=True)    
    fig.suptitle('Muscles')
    for i, ax in enumerate(axs.flat):
        if i < NSideMuscles:
            # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
            for case in cases:
                ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('ankle_angle_l'),:], c='black')
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.9, c='blue')    
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.8, c='red')    
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.7, c='green')    
                ax.plot(range_ankle_deg, dM_test_old[i,leftPolynomialJoints.index('ankle_angle_l'),:], c='orange')        
            ax.set_title(muscles[i])
            if i < NSideMuscles and i > 41:
                ax.set_xticks([-50,40])
                ax.set_xticklabels(['-50','40'])
                ax.set_xlabel('Ankle angle (deg)')
            else:
                ax.set_xticks([-50,40]) 
                ax.set_xticklabels([]) 
            # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
            handles, labels = ax.get_legend_handles_labels()
            plt.legend(handles, labels, loc='upper right')
        
    # plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    plt.setp(axs[:, 0], ylabel='(m)')
    fig.align_ylabels()
    
    # %% Subtalar
    
    # Test polynomials
    
    range_ankle = np.linspace(-45, 45, 1000)
    range_ankle *= (np.pi/180)
    Qdotsin_test = np.zeros((len(leftPolynomialJoints),))
    lMT_test = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    vMT_test = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    dM_test = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_ankle.shape[0]))
    lMT_test_old = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    vMT_test_old = np.zeros((NSideMuscles+3, range_ankle.shape[0]))
    dM_test_old = np.zeros((NSideMuscles+3, len(leftPolynomialJoints), range_ankle.shape[0]))
    for i in range(range_ankle.shape[0]):
        Qsin_test = np.zeros((len(leftPolynomialJoints),))
        Qsin_test[leftPolynomialJoints.index('subtalar_angle_l')] = range_ankle[i]
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial(Qsin_test, Qdotsin_test)        
        lMT_test[:, i] = (c_lMT_test.full()).flatten()
        vMT_test[:, i] = (c_vMT_test.full()).flatten()
        dM_test [:,:,i] = (c_dM_test.full())
        
        [c_lMT_test, c_vMT_test, c_dM_test] = f_polynomial_old(Qsin_test, Qdotsin_test)        
        lMT_test_old[:, i] = (c_lMT_test.full()).flatten()
        vMT_test_old[:, i] = (c_vMT_test.full()).flatten()
        dM_test_old [:,:,i] = (c_dM_test.full())
    
    # range_ankle_deg = range_ankle*180/np.pi
    # import matplotlib.pyplot as plt  
    # fig, axs = plt.subplots(7, 7, sharex=True)    
    # fig.suptitle('Muscles')
    # for i, ax in enumerate(axs.flat):
    #     if i < NSideMuscles:
    #         # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
    #         for case in cases:
    #             ax.plot(range_ankle_deg, lMT_test[i,:], c='black')            
    #             ax.plot(range_ankle_deg, lMT_test_old[i,:], c='orange')        
    #         ax.set_title(muscles[i])
    #         if i < NSideMuscles and i > 41:
    #             ax.set_xticks([-120,10])
    #             ax.set_xticklabels(['-120','10'])
    #             ax.set_xlabel('Knee angle (deg)')
    #         else:
    #             ax.set_xticks([-120,10]) 
    #             ax.set_xticklabels([])
    #         # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
    #         handles, labels = ax.get_legend_handles_labels()
    #         plt.legend(handles, labels, loc='upper right')
    # plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    # plt.setp(axs[:, 0], ylabel='(m)')
    # fig.align_ylabels()
    
    # test1 = lMT_test_old / lMT_test
        
    fig, axs = plt.subplots(7, 7, sharex=True)    
    fig.suptitle('Muscles')
    for i, ax in enumerate(axs.flat):
        if i < NSideMuscles:
            # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
            for case in cases:
                ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('subtalar_angle_l'),:], c='black')
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.9, c='blue')    
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.8, c='red')    
                # ax.plot(range_ankle_deg, dM_test[i,leftPolynomialJoints.index('knee_angle_l'),:]*0.7, c='green')    
                ax.plot(range_ankle_deg, dM_test_old[i,leftPolynomialJoints.index('subtalar_angle_l'),:], c='orange')        
            ax.set_title(muscles[i])
            if i < NSideMuscles and i > 41:
                ax.set_xticks([-45,45])
                ax.set_xticklabels(['-45','45'])
                ax.set_xlabel('Subtalar angle (deg)')
            else:
                ax.set_xticks([-45,45]) 
                ax.set_xticklabels([]) 
            # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
            handles, labels = ax.get_legend_handles_labels()
            plt.legend(handles, labels, loc='upper right')
        
    # plt.setp(axs[-1, :], xlabel='Knee angle (deg)')
    plt.setp(axs[:, 0], ylabel='(m)')
    fig.align_ylabels()