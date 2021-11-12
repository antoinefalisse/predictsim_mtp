import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from sklearn.metrics import mean_squared_error, r2_score

from utilities import getJointIndices

# %% Settings
labels = ['Old model - low contact spheres - without toes', 
          'New model - low contact spheres - without toes',
          'New model - high contact spheres - without toes',
          'Old model - low contact spheres - with toes',
          'New model - high contact spheres - with toes']

cases = ['31', '32', '28', '27', '4']
case_4exp = '4'

colors=['black', '#984ea3','#4daf4a','#377eb8','#ff7f00'] 
linestyles=['solid','dashed','dashdot','solid','dashdot']
linewidth_s = 3
fontsize_tick = 14
fontsize_label = 15
fontsize_title = 17

# %% Fixed settings
pathMain = os.getcwd()
# Load results
pathTrajectories = os.path.join(pathMain, 'Results')
optimaltrajectories = np.load(os.path.join(pathTrajectories, 
                                           'optimalTrajectories.npy'),
                              allow_pickle=True).item()
# Load experimental data
pathData = os.path.join(pathMain, 'OpenSimModel', 'new_model')
experimentalData = np.load(os.path.join(pathData, 'experimentalData.npy'),
                           allow_pickle=True).item()
subject = 'subject2' # TODO
threshold = 5

metrics = {}
metrics['RMSE'] = {}
metrics['R2'] = {}

# %% Kinematics
jointsToAnalyze = ['knee_angle_r',  'ankle_angle_r']
metrics['RMSE']['kinematics'] = {}
metrics['R2']['kinematics'] = {}

for i, joint in enumerate(jointsToAnalyze):
    metrics['RMSE']['kinematics'][joint] = {}
    metrics['R2']['kinematics'][joint] = {}
    for c, case in enumerate(cases):    
            
        c_joints = optimaltrajectories[case]['joints']
        c_joint_idx = c_joints.index(joint)
        
        # Reference data
        c_ref = experimentalData[subject]["kinematics"]["positions"]["mean"][joint].to_numpy()
        c_ref_t = experimentalData[subject]["kinematics"]["positions"]["GC_percent"]
        # Find stance-swing transition
        c_ref_vGRF = experimentalData[subject]["GRF"]["mean"]['GRF_y_r'].to_numpy()
        c_ref_idx_tr = np.argwhere(c_ref_vGRF < threshold)[0][0]
        # Select stance phase
        c_ref_stance = c_ref[:c_ref_idx_tr]
        # Interpolate over 100 data points
        c_ref_vec = np.linspace(0, c_ref_stance.shape[0]-1, c_ref_stance.shape[0])
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, 100)
        set_interp = interp1d(c_ref_vec, c_ref_stance)
        c_ref_inter = set_interp(c_ref_vec_N)
        
        # Simulated data
        c_sim = optimaltrajectories[case]['coordinate_values'][c_joint_idx:c_joint_idx+1, :].flatten()
        c_sim_t = optimaltrajectories[case]['GC_percent']
        # Find stance-swing transition
        c_sim_vGRF = optimaltrajectories[case]['GRF'][1, :].T
        c_sim_idx_tr = np.argwhere(c_sim_vGRF < threshold)[0][0]
        # Select stance phase
        c_sim_stance = c_sim[:c_sim_idx_tr]
        # Interpolate over 100 data points
        c_sim_vec = np.linspace(0, c_sim_stance.shape[0]-1, c_sim_stance.shape[0])
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, 100)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinematics'][joint][case] = mean_squared_error(c_ref_inter, c_sim_inter, squared=False)
        metrics['R2']['kinematics'][joint][case] = r2_score(c_ref_inter, c_sim_inter)
        
# %% Kinetics
metrics['RMSE']['kinetics'] = {}
metrics['R2']['kinetics'] = {}

for i, joint in enumerate(jointsToAnalyze):
    metrics['RMSE']['kinetics'][joint] = {}
    metrics['R2']['kinetics'][joint] = {}
    for c, case in enumerate(cases):    
            
        c_joints = optimaltrajectories[case]['joints']
        c_joint_idx = c_joints.index(joint)
        
        # Reference data
        c_ref = experimentalData[subject]["kinetics"]["mean"][joint].to_numpy()
        c_ref_t = experimentalData[subject]["kinetics"]["GC_percent"]
        # Find stance-swing transition
        c_ref_vGRF = experimentalData[subject]["GRF"]["mean"]['GRF_y_r'].to_numpy()
        c_ref_idx_tr = np.argwhere(c_ref_vGRF < threshold)[0][0]
        # Select stance phase
        c_ref_stance = c_ref[:c_ref_idx_tr]
        # Interpolate over 100 data points
        c_ref_vec = np.linspace(0, c_ref_stance.shape[0]-1, c_ref_stance.shape[0])
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, 100)
        set_interp = interp1d(c_ref_vec, c_ref_stance)
        c_ref_inter = set_interp(c_ref_vec_N)
        
        # Simulated data
        c_sim = optimaltrajectories[case]['joint_torques'][c_joint_idx:c_joint_idx+1, :].flatten()
        c_sim_t = optimaltrajectories[case]['GC_percent']
        # Find stance-swing transition
        c_sim_vGRF = optimaltrajectories[case]['GRF'][1, :].T
        c_sim_idx_tr = np.argwhere(c_sim_vGRF < threshold)[0][0]
        # Select stance phase
        c_sim_stance = c_sim[:c_sim_idx_tr]
        # Interpolate over 100 data points
        c_sim_vec = np.linspace(0, c_sim_stance.shape[0]-1, c_sim_stance.shape[0])
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, 100)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinetics'][joint][case] = mean_squared_error(c_ref_inter, c_sim_inter, squared=False)
        metrics['R2']['kinetics'][joint][case] = r2_score(c_ref_inter, c_sim_inter)

# %% Analysis results
print('Influence of the mass distribution')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_mass = 100 - (metrics['RMSE']['kinematics']['knee_angle_r']['32'] / metrics['RMSE']['kinematics']['knee_angle_r']['31']) * 100
rmse_change_knee_kinetics_mass = 100 - (metrics['RMSE']['kinetics']['knee_angle_r']['32'] / metrics['RMSE']['kinetics']['knee_angle_r']['31']) * 100
print('RMSE kinematics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinematics_mass), round(metrics['RMSE']['kinematics']['knee_angle_r']['31'],1), round(metrics['RMSE']['kinematics']['knee_angle_r']['32'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinetics_mass), round(metrics['RMSE']['kinetics']['knee_angle_r']['31'],1), round(metrics['RMSE']['kinetics']['knee_angle_r']['32'],1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_mass = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r']['32'] / metrics['RMSE']['kinematics']['ankle_angle_r']['31']) * 100
rmse_change_ankle_kinetics_mass = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r']['32'] / metrics['RMSE']['kinetics']['ankle_angle_r']['31']) * 100
print('RMSE kinematics decreased by {}, from {} to {}'.format(round(rmse_change_ankle_kinematics_mass), round(metrics['RMSE']['kinematics']['ankle_angle_r']['31'],1), round(metrics['RMSE']['kinematics']['ankle_angle_r']['32'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_ankle_kinetics_mass), round(metrics['RMSE']['kinetics']['ankle_angle_r']['31'],1), round(metrics['RMSE']['kinetics']['ankle_angle_r']['32'],1)))

print('Influence of the sphere position')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_sphere = 100 - (metrics['RMSE']['kinematics']['knee_angle_r']['28'] / metrics['RMSE']['kinematics']['knee_angle_r']['32']) * 100
rmse_change_knee_kinetics_sphere = 100 - (metrics['RMSE']['kinetics']['knee_angle_r']['28'] / metrics['RMSE']['kinetics']['knee_angle_r']['32']) * 100
print('RMSE kinematics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinematics_sphere), round(metrics['RMSE']['kinematics']['knee_angle_r']['32'],1), round(metrics['RMSE']['kinematics']['knee_angle_r']['28'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinetics_sphere), round(metrics['RMSE']['kinetics']['knee_angle_r']['32'],1), round(metrics['RMSE']['kinetics']['knee_angle_r']['28'],1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_sphere = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r']['28'] / metrics['RMSE']['kinematics']['ankle_angle_r']['32']) * 100
rmse_change_ankle_kinetics_sphere = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r']['28'] / metrics['RMSE']['kinetics']['ankle_angle_r']['32']) * 100
print('RMSE kinematics decreased by {} %, from {} to {}'.format(round(rmse_change_ankle_kinematics_sphere), round(metrics['RMSE']['kinematics']['ankle_angle_r']['32'],1), round(metrics['RMSE']['kinematics']['ankle_angle_r']['28'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_ankle_kinetics_sphere), round(metrics['RMSE']['kinetics']['ankle_angle_r']['32'],1), round(metrics['RMSE']['kinetics']['ankle_angle_r']['28'],1)))

print('Influence of the toes')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_toes = 100 - (metrics['RMSE']['kinematics']['knee_angle_r']['4'] / metrics['RMSE']['kinematics']['knee_angle_r']['28']) * 100
rmse_change_knee_kinetics_toes = 100 - (metrics['RMSE']['kinetics']['knee_angle_r']['4'] / metrics['RMSE']['kinetics']['knee_angle_r']['28']) * 100
print('RMSE kinematics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinematics_toes), round(metrics['RMSE']['kinematics']['knee_angle_r']['28'],1), round(metrics['RMSE']['kinematics']['knee_angle_r']['4'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_knee_kinetics_toes), round(metrics['RMSE']['kinetics']['knee_angle_r']['28'],1), round(metrics['RMSE']['kinetics']['knee_angle_r']['4'],1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_toes = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r']['4'] / metrics['RMSE']['kinematics']['ankle_angle_r']['28']) * 100
rmse_change_ankle_kinetics_toes = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r']['4'] / metrics['RMSE']['kinetics']['ankle_angle_r']['28']) * 100
print('RMSE kinematics decreased by {} %, from {} to {}'.format(round(rmse_change_ankle_kinematics_toes), round(metrics['RMSE']['kinematics']['ankle_angle_r']['28'],1), round(metrics['RMSE']['kinematics']['ankle_angle_r']['4'],1)))
print('RMSE kinetics decreased by {} %, from {} to {}'.format(round(rmse_change_ankle_kinetics_toes), round(metrics['RMSE']['kinetics']['ankle_angle_r']['28'],1), round(metrics['RMSE']['kinetics']['ankle_angle_r']['4'],1)))

# %% First peak GRF
# Experimental
c_ref_vGRF_max = np.max(experimentalData[subject]["GRF"]["mean"]['GRF_y_r'].to_numpy())
# Simulation with toes
c_sim_toes_vGRF = np.max(optimaltrajectories['4']['GRF'][1, :].T)
# Simulation without toes
c_sim_noToes_vGRF = np.max(optimaltrajectories['28']['GRF'][1, :].T)

peak_GRF_change = 100 - (c_sim_toes_vGRF/c_sim_noToes_vGRF*100)
print('Peak decreased by {} %, from {} to {}'.format(round(peak_GRF_change), round(c_sim_noToes_vGRF), round(c_sim_toes_vGRF)))

# %%
# COT comparison
COT_withToes = optimaltrajectories['4']['COT_perMuscle']
COT_withoutToes = optimaltrajectories['28']['COT_perMuscle']

COT_diff = COT_withToes-COT_withoutToes
sort_COT_idx = np.flip(np.argsort(COT_diff))
sort_COT = np.flip(np.sort(ratio_COT))
muscles_names = optimaltrajectories['4']['muscles']
sort_COT_muscles = []
for idx in sort_COT_idx:    
    sort_COT_muscles.append(muscles_names[idx])
sort_COT_diff = COT_diff[sort_COT_idx]
COT_quads_added = np.sum(sort_COT_diff[0:8])

COT_withToes_sum = np.sum(COT_withToes)
COT_withoutToes_sum = np.sum(COT_withoutToes)

COT_quads_added / (COT_withToes_sum-COT_withoutToes_sum)*100