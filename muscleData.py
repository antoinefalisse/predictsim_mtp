from sys import path
import os
import numpy as np

def getMTParameters(pathOS, pathModel, muscles, loadMTParameters, pathMTParameters=0):
    
    if loadMTParameters:        
        mtParameters = np.load(os.path.join(pathMTParameters, 'mtParameters.npy'), 
                            allow_pickle=True)     
        
    else: 
        path.insert(0, pathOS)
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
            
# TODO not longer used because parameters can now be directly extracted from
# model
def mtParameters_2D():
    
    mtParameters = np.array([
            [2700, 804, 1944, 2342, 1169, 5000, 2500, 5137, 3000],
            [0.109059287708541, 0.171872304562285, 0.154778393881481, 
             0.0974995850024712, 0.114154396713697, 0.107708763662917, 
             0.0869260793110567, 0.0481705511245028, 0.0937899168938178],
             [0.326177319201692, 0.0884198561042967, 0.0723088353824893, 
              0.155999336003954, 0.310419850712686, 0.116768379298115, 
              0.347704317244227, 0.240852755622514, 0.213419912931851],
              [0,	0.40142573,	0,	0.13962634,	0.08726646,
               0.05235988,	0.29670597,	0.43633231,	0.08726646],
               [1.09059287708541, 1.71872304562285, 1.54778393881481,
                0.974995850024712, 1.14154396713697, 1.07708763662917, 
                0.869260793110567, 0.481705511245028, 0.937899168938178]])
        
    return mtParameters 

def tendonCompliance_2D(NSideMuscles):
    tendonCompliance = np.full((1, NSideMuscles), 35)
    
    return tendonCompliance

def tendonShift_2D(NSideMuscles):
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

# TODO not longer used because parameters can now be directly extracted from
# model
def mtParameters_3D():
    
    mtParameters = np.array([
            [819, 573, 653, 270, 285, 323, 1288, 410, 896, 804, 156, 627, 429,
             381, 343, 488, 233, 266, 162, 573, 819, 552, 1073, 1113, 381, 164,
             444, 1169, 1294, 1365, 1871, 1558, 683, 3549, 1588, 310, 322, 905,
             435, 943, 180, 512, 162, 2500, 900, 900], 
             [0.0520776466291754, 0.0823999283675263, 0.0632190293747345, 
              0.0665743872254168, 0.0549421208310944, 0.0373512946690306, 
              0.0800925765489541, 0.200513601757488, 0.109059287708541, 
              0.171872304562285, 0.515800451495399, 0.138312421924500, 
              0.133193044656658, 0.0875366938287895, 0.122225183391540, 
              0.131955204523298, 0.0944201526072643, 0.0997850929935035, 
              0.351729725497981, 0.138993580345969, 0.144758451755550, 
              0.142117802941823, 0.0976783826340393, 0.0974995850024712, 
              0.0534125956263058, 0.0237032199612196, 0.0254922255791636, 
              0.114154396713697, 0.0894440221887840, 0.0875762844735864, 
              0.0843607720996998, 0.0579507195407045, 0.0617978232068280, 
              0.0481705511245028, 0.0297570603130644, 0.0320252816344382, 
              0.0404119969590143, 0.0937899168938178, 0.0478795464373982, 
              0.0466397693044547, 0.0753718253866636, 0.0966430161097304, 
              0.104902499371277, 0.115203171511584, 0.0968336708901757, 
              0.114315693448030], 
              [0.0759262885434707, 0.0516827953074425, 0.0518670055241629, 
               0.0156645617000981, 0.0255088418144367, 0.0501293691610674, 
               0.359415437263432, 0.254881717656906, 0.326177319201692, 
               0.0884198561042967, 0.0991923945183459, 0.110249031968805, 
               0.0200290292716779, 0.0603701336750272, 0.121215057908965, 
               0.250815617758025, 0.422405945874603, 0.0329290806878562, 
               0.125903254013482, 0.122353503825677, 0.125063424305815, 
               0.143104732128919, 0.0976783826340393, 0.155999336003954, 
               0.0237389313894692, 0.0385177324369819, 0.112754074677070, 
               0.310419850712686, 0.126628615682998, 0.136900858487445, 
               0.157674300233963, 0.376679677014579, 0.366924575290541, 
               0.240852755622514, 0.297570603130644, 0.376768019228685, 
               0.357129275451754, 0.213419912931851, 0.154172139528422, 
               0.328382049184426, 0.0954073739071692, 0.326880789782912,
               0.288245606380536, 0.0288007928778960, 0.0968336708901757,
               0.133368309022702], 
               [0.139626340000000, 0, 0.331612560000000, 0.174532930000000, 
                0, 0.0174532900000000, 0.261799390000000, 0.0872664600000000, 
                0, 0.401425730000000, 0, 0.104719760000000, 0, 
                0.0872664600000000, 0.0523598800000000, 0.0872664600000000, 
                0.0523598800000000, 0, 0.0523598800000000, 0.0872664600000000, 
                0, 0.0872664600000000, 0.122173050000000, 0.139626340000000, 0, 
                0, 0.174532930000000, 0.0872664600000000, 0.0872664600000000,
                0.0523598800000000, 0.0872664600000000, 0.296705970000000, 
                0.139626340000000, 0.436332310000000, 0.209439510000000, 
                0.122173050000000, 0.174532930000000, 0.0872664600000000, 
                0.0872664600000000, 0.174532930000000, 0.226892800000000, 
                0.139626340000000, 0.104719760000000, 0, 0, 0], 
                [0.520776466291754, 0.823999283675263, 0.632190293747345, 
                 0.665743872254168, 0.549421208310944, 0.373512946690306, 
                 0.800925765489541, 2.00513601757488, 1.09059287708541, 
                 1.71872304562285, 5.15800451495399, 1.38312421924500, 
                 1.33193044656658, 0.875366938287895, 1.22225183391540, 
                 1.31955204523298, 0.944201526072643, 0.997850929935035, 
                 3.51729725497981, 1.38993580345969, 1.44758451755550, 
                 1.42117802941823, 0.976783826340393, 0.974995850024712,
                 0.534125956263058, 0.237032199612196, 0.254922255791636,
                 1.14154396713697, 0.894440221887840, 0.875762844735864,
                 0.843607720996998, 0.579507195407045, 0.617978232068280,
                 0.481705511245028, 0.297570603130644, 0.320252816344382,
                 0.404119969590143, 0.937899168938178, 0.478795464373982,
                 0.466397693044547, 0.753718253866636, 0.966430161097304, 
                 1.04902499371277, 1.15203171511584, 0.968336708901757, 
                 1.14315693448030]])
        
    return mtParameters    

def tendonCompliance_3D():
    tendonCompliance = np.full((1, 46), 35)
    
    return tendonCompliance

def tendonShift_3D():
    tendonShift = np.full((1, 46), 0)
    
    return tendonShift

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

def specificTension_2D_muscle(muscles):    
    
    sigma = {'hamstrings_r' : 0.62222,
             'bifemsh_r': 1.00500, 
             'glut_max_r': 0.74455, 
             'iliopsoas_r': 1.5041, 
             'rect_fem_r': 0.74936, 
             'vasti_r': 0.55263, 
             'gastroc_r': 0.69865, 
             'soleus_r': 0.62703, 
             'tib_ant_r': 0.75417,
             'ext_dig_r': 0.75,
             'ext_hal_r': 0.75, 
             'flex_dig_r': 0.60, 
             'flex_hal_r': 0.60,
             'ercspn_r': 0.60,
             'intobl_r': 0.56, 
             'extobl_r': 0.58}
    
    specificTension = np.empty((1, len(muscles)))    
    for count, muscle in enumerate(muscles):
        specificTension[0, count] = sigma[muscle]
    
    return specificTension
    