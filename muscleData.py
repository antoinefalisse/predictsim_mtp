from sys import path
import os
import numpy as np

def getMTParameters(pathModel, muscles, loadMTParameters, pathMTParameters=0):
    
    if loadMTParameters:        
        mtParameters = np.load(os.path.join(pathMTParameters, 'mtParameters.npy'), 
                            allow_pickle=True)     
        
    else:
        import opensim
        model = opensim.Model(pathModel)
        mtParameters = np.zeros([5,len(muscles)])
        model_muscles = model.getMuscles()
        for i in range(len(muscles)):
           muscle = model_muscles.get(muscles[i])
           mtParameters[0,i] = muscle.getMaxIsometricForce()
           mtParameters[1,i] = muscle.getOptimalFiberLength()
           mtParameters[2,i] = muscle.getTendonSlackLength()
           mtParameters[3,i] = muscle.getPennationAngleAtOptimalFiberLength()
           mtParameters[4,i] = muscle.getMaxContractionVelocity()*muscle.getOptimalFiberLength()
        if pathMTParameters != 0:
           np.save(os.path.join(pathMTParameters, 'mtParameters.npy'), mtParameters)
       
    return mtParameters  

def getPolynomialData(loadPolynomialData, pathPolynomialData, pathCoordinates='', pathMuscleAnalysis='', joints=[], muscles=[]):
    
    if loadPolynomialData:
        polynomialData = np.load(os.path.join(pathPolynomialData, 'polynomialData.npy'), 
                            allow_pickle=True) 
        
    else:       
        from polynomials import getPolynomialCoefficients
        polynomialData = getPolynomialCoefficients(pathCoordinates, pathMuscleAnalysis, joints, muscles)
        if pathPolynomialData != 0:
            np.save(os.path.join(pathPolynomialData, 'polynomialData.npy'), polynomialData)
           
    return polynomialData

def tendonCompliance(NSideMuscles):
    tendonCompliance = np.full((1, NSideMuscles), 35)
    
    return tendonCompliance

def tendonShift(NSideMuscles):
    tendonShift = np.full((1, NSideMuscles), 0)
    
    return tendonShift

def specificTension_2D(muscles):    
    
    sigma = {'hamstrings_r' : 0.62222,
             'bifemsh_r': 1.00500, 
             'glut_max_r': 0.74455, 
             'iliopsoas_r': 1.5041, 
             'rect_fem_r': 0.74936, 
             'vasti_r': 0.55263, 
             'gastroc_r': 0.69865, 
             'soleus_r': 0.62703, 
             'tib_ant_r': 0.75417}
    
    specificTension = np.empty((1, len(muscles)))    
    for count, muscle in enumerate(muscles):
        specificTension[0, count] = sigma[muscle]
    
    return specificTension

def slowTwitchRatio_2D(muscles):    
    
    sigma = {'hamstrings_r' : 0.5425,
             'bifemsh_r': 0.529, 
             'glut_max_r': 0.55, 
             'iliopsoas_r': 0.50, 
             'rect_fem_r': 0.3865, 
             'vasti_r': 0.543, 
             'gastroc_r': 0.566, 
             'soleus_r': 0.803, 
             'tib_ant_r': 0.70}
    
    slowTwitchRatio = np.empty((1, len(muscles)))    
    for count, muscle in enumerate(muscles):
        slowTwitchRatio[0, count] = sigma[muscle]
    
    return slowTwitchRatio   

def specificTension_3D(muscles):    
    
    sigma = {'glut_med1_r' : 0.74455,
             'glut_med2_r': 0.75395, 
             'glut_med3_r': 0.75057, 
             'glut_min1_r': 0.75, 
             'glut_min2_r': 0.75, 
             'glut_min3_r': 0.75116, 
             'semimem_r': 0.62524, 
             'semiten_r': 0.62121, 
             'bifemlh_r': 0.62222,
             'bifemsh_r': 1.00500, 
             'sar_r': 0.74286,
             'add_long_r': 0.74643, 
             'add_brev_r': 0.75263,
             'add_mag1_r': 0.55217,
             'add_mag2_r': 0.55323, 
             'add_mag3_r': 0.54831, 
             'tfl_r': 0.75161,
             'pect_r': 0.76000, 
             'grac_r': 0.73636, 
             'glut_max1_r': 0.75395, 
             'glut_max2_r': 0.74455, 
             'glut_max3_r': 0.74595, 
             'iliacus_r': 1.2477,
             'psoas_r': 1.5041,
             'quad_fem_r': 0.74706, 
             'gem_r': 0.74545, 
             'peri_r': 0.75254, 
             'rect_fem_r': 0.74936, 
             'vas_med_r': 0.49961, 
             'vas_int_r': 0.55263, 
             'vas_lat_r': 0.50027,
             'med_gas_r': 0.69865, 
             'lat_gas_r': 0.69694, 
             'soleus_r': 0.62703,
             'tib_post_r': 0.62520, 
             'flex_dig_r': 0.5, 
             'flex_hal_r': 0.50313,
             'tib_ant_r': 0.75417, 
             'per_brev_r': 0.62143,
             'per_long_r': 0.62450, 
             'per_tert_r': 1.0,
             'ext_dig_r': 0.75294,
             'ext_hal_r': 0.73636, 
             'ercspn_r': 0.25, 
             'intobl_r': 0.25, 
             'extobl_r': 0.25}
    
    specificTension = np.empty((1, len(muscles)))    
    for count, muscle in enumerate(muscles):
        specificTension[0, count] = sigma[muscle]
    
    return specificTension

def slowTwitchRatio_3D(muscles):    
    
    sigma = {'glut_med1_r' : 0.55,
             'glut_med2_r': 0.55, 
             'glut_med3_r': 0.55, 
             'glut_min1_r': 0.55, 
             'glut_min2_r': 0.55, 
             'glut_min3_r': 0.55, 
             'semimem_r': 0.4925, 
             'semiten_r': 0.425, 
             'bifemlh_r': 0.5425,
             'bifemsh_r': 0.529, 
             'sar_r': 0.50,
             'add_long_r': 0.50, 
             'add_brev_r': 0.50,
             'add_mag1_r': 0.552,
             'add_mag2_r': 0.552, 
             'add_mag3_r': 0.552, 
             'tfl_r': 0.50,
             'pect_r': 0.50, 
             'grac_r': 0.50, 
             'glut_max1_r': 0.55, 
             'glut_max2_r': 0.55, 
             'glut_max3_r': 0.55, 
             'iliacus_r': 0.50,
             'psoas_r': 0.50,
             'quad_fem_r': 0.50, 
             'gem_r': 0.50, 
             'peri_r': 0.50, 
             'rect_fem_r': 0.3865, 
             'vas_med_r': 0.503, 
             'vas_int_r': 0.543, 
             'vas_lat_r': 0.455,
             'med_gas_r': 0.566, 
             'lat_gas_r': 0.507, 
             'soleus_r': 0.803,
             'tib_post_r': 0.60, 
             'flex_dig_r': 0.60, 
             'flex_hal_r': 0.60,
             'tib_ant_r': 0.70, 
             'per_brev_r': 0.60,
             'per_long_r': 0.60, 
             'per_tert_r': 0.75,
             'ext_dig_r': 0.75,
             'ext_hal_r': 0.75, 
             'ercspn_r': 0.60,
             'intobl_r': 0.56, 
             'extobl_r': 0.58}
    
    slowTwitchRatio = np.empty((1, len(muscles)))    
    for count, muscle in enumerate(muscles):
        slowTwitchRatio[0, count] = sigma[muscle]
    
    return slowTwitchRatio

def passiveJointTorqueData_3D(joint):    
    
    kAll = {'hip_flexion_r' : [-2.44, 5.05, 1.51, -21.88],
            'hip_adduction_r': [-0.03, 14.94, 0.03, -14.94], 
            'hip_rotation_r': [-0.03, 14.94, 0.03, -14.94],
            'knee_angle_r': [-6.09, 33.94, 11.03, -11.33],
            'ankle_angle_r': [-2.03, 38.11, 0.18, -12.12],
            'subtalar_angle_r': [-60.21, 16.32, 60.21, -16.32],
            'mtp_angle_r': [-0.9, 14.87, 0.18, -70.08],            
            'lumbar_extension': [-0.35, 30.72, 0.25, -20.36],
            'lumbar_bending': [-0.25, 20.36, 0.25, -20.36],
            'lumbar_rotation': [-0.25, 20.36, 0.25, -20.36]}
    
    thetaAll = {'hip_flexion_r' : [-0.6981, 1.81],
                'hip_adduction_r': [-0.5, 0.5], 
                'hip_rotation_r': [-0.92, 0.92],
                'knee_angle_r': [-2.4, 0.13],
                'ankle_angle_r': [-0.74, 0.52],
                'subtalar_angle_r': [-0.65, 0.65],
                'mtp_angle_r': [0, 1.134464013796314],
                'lumbar_extension': [-0.5235987755982988, 0.17],
                'lumbar_bending': [-0.3490658503988659, 0.3490658503988659],
                'lumbar_rotation': [-0.3490658503988659, 0.3490658503988659]}
    
    k = kAll[joint] 
    theta = thetaAll[joint]
    
    return k, theta  


    