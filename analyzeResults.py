'''
    This script computes RMSEs for kinematics and kinetics, and compares the
    results across cases. Focus is on studying the influence of modeling
    choices on the simulations.
'''

# %% Import packages
import os
import numpy as np
from scipy.interpolate import interp1d
from sklearn.metrics import mean_squared_error, r2_score

# %% Settings
# higher damping (d=2)
cases = ['31', '32', '28', '66', '40']
# lower damping (d=0.4)
# cases = ['31', '32', '28', '27', '4']

old_model_without_toes = '31'
new_model_without_toes = '32'
new_model_without_toes_high = '28'
old_model_with_toes = '66'
new_model_with_toes = '40'

# Effect of Achilles tendon stiffness
# cases = ['40','53','54','56','59','60','62','64']

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
subject = 'new_model'
threshold = 5 # vGRF threshold for stance-swing transition
N = 100 # # data points for interpolation

metrics = {}
metrics['RMSE'] = {}
metrics['R2'] = {}
signal_range = {}

for case in cases:
    print('COT case {}: {}'.format(case, np.round(optimaltrajectories[case]['COT'],2)))
    print('# iterations case {}: {}'.format(case, optimaltrajectories[case]['iter_count']))

# %% Kinematics
jointsToAnalyze = ['knee_angle_r',  'ankle_angle_r']
metrics['RMSE']['kinematics'] = {}
metrics['R2']['kinematics'] = {}
signal_range['kinematics'] = {}
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
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, N)
        set_interp = interp1d(c_ref_vec, c_ref_stance)
        c_ref_inter = set_interp(c_ref_vec_N)        
        signal_range['kinematics'][joint] = np.max(c_ref_inter) - np.min(c_ref_inter) 
        
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
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, N)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinematics'][joint][case] = mean_squared_error(c_ref_inter, c_sim_inter, squared=False)
        metrics['R2']['kinematics'][joint][case] = r2_score(c_ref_inter, c_sim_inter)
        
# %% Kinetics
metrics['RMSE']['kinetics'] = {}
metrics['R2']['kinetics'] = {}
signal_range['kinetics'] = {}
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
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, N)
        set_interp = interp1d(c_ref_vec, c_ref_stance)
        c_ref_inter = set_interp(c_ref_vec_N)
        signal_range['kinetics'][joint] = np.max(c_ref_inter) - np.min(c_ref_inter) 
        
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
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, N)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinetics'][joint][case] = mean_squared_error(c_ref_inter, c_sim_inter, squared=False)
        metrics['R2']['kinetics'][joint][case] = r2_score(c_ref_inter, c_sim_inter)
        
# %% Percent change as a function of signal range with respect to previous case
variables = ['kinematics', 'kinetics']
joints = ['knee_angle_r', 'ankle_angle_r']
changes = {}
changes_baseline = {}
for variable in variables:
    changes[variable] = {}
    changes_baseline[variable] = {}
    for joint in joints:
        changes[variable][joint] = {}
        changes_baseline[variable][joint] = {}
        for c, case in enumerate(cases[1:]):            
            RMSE_change = metrics['RMSE'][variable][joint][case] - metrics['RMSE'][variable][joint][cases[c]]
            changes[variable][joint][case] = np.round(RMSE_change / signal_range[variable][joint] * 100, 1)
            
            RMSE_change_baseline = metrics['RMSE'][variable][joint][case] - metrics['RMSE'][variable][joint][cases[0]]
            changes_baseline[variable][joint][case] = np.round(RMSE_change_baseline / signal_range[variable][joint] * 100, 1)   

# %% Analysis results
print('Influence of the mass distribution')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_mass = 100 - (metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes] / metrics['RMSE']['kinematics']['knee_angle_r'][old_model_without_toes]) * 100
rmse_change_knee_kinetics_mass = 100 - (metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes] / metrics['RMSE']['kinetics']['knee_angle_r'][old_model_without_toes]) * 100
rmse_change_knee_kinematics_mass_signal = np.abs(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes] - metrics['RMSE']['kinematics']['knee_angle_r'][old_model_without_toes])/signal_range['kinematics']['knee_angle_r']*100
rmse_change_knee_kinetics_mass_signal = np.abs(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes] - metrics['RMSE']['kinetics']['knee_angle_r'][old_model_without_toes])/signal_range['kinetics']['knee_angle_r']*100
print('RMSE kinematics decreased by {}%, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinematics_mass), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][old_model_without_toes],1), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes],1), 
    round(rmse_change_knee_kinematics_mass_signal,1)))
print('RMSE kinetics decreased by {}%, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinetics_mass), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][old_model_without_toes],1), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes],1),
    round(rmse_change_knee_kinetics_mass_signal,1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_mass = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes] / metrics['RMSE']['kinematics']['ankle_angle_r'][old_model_without_toes]) * 100
rmse_change_ankle_kinetics_mass = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes] / metrics['RMSE']['kinetics']['ankle_angle_r'][old_model_without_toes]) * 100
rmse_change_ankle_kinematics_mass_signal = np.abs(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes] - metrics['RMSE']['kinematics']['ankle_angle_r'][old_model_without_toes])/signal_range['kinematics']['ankle_angle_r']*100
rmse_change_ankle_kinetics_mass_signal = np.abs(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes] - metrics['RMSE']['kinetics']['ankle_angle_r'][old_model_without_toes])/signal_range['kinetics']['ankle_angle_r']*100
print('RMSE kinematics decreased by {}%, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinematics_mass), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][old_model_without_toes],1), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes],1), 
    round(rmse_change_ankle_kinematics_mass_signal,1)))
print('RMSE kinetics decreased by {}%, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinetics_mass), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][old_model_without_toes],1), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes],1),
    round(rmse_change_ankle_kinetics_mass_signal,1)))

print('Influence of the sphere position')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_sphere = 100 - (metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high] / metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes]) * 100
rmse_change_knee_kinetics_sphere = 100 - (metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high] / metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes]) * 100
rmse_change_knee_kinematics_signal = np.abs(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high] - metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes])/signal_range['kinematics']['knee_angle_r']*100
rmse_change_knee_kinetics_sphere_signal = np.abs(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high] - metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes])/signal_range['kinetics']['knee_angle_r']*100
print('RMSE kinematics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinematics_sphere), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes],1), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high],1),
    round(rmse_change_knee_kinematics_signal,1)))
print('RMSE kinetics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinetics_sphere), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes],1), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high],1),
    round(rmse_change_knee_kinetics_sphere_signal,1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_sphere = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high] / metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes]) * 100
rmse_change_ankle_kinetics_sphere = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high] / metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes]) * 100
rmse_change_ankle_kinematics_signal = np.abs(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high] - metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes])/signal_range['kinematics']['ankle_angle_r']*100
rmse_change_ankle_kinetics_sphere_signal = np.abs(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high] - metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes])/signal_range['kinetics']['ankle_angle_r']*100
print('RMSE kinematics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinematics_sphere), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes],1), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high],1),
    round(rmse_change_ankle_kinematics_signal,1)))
print('RMSE kinetics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinetics_sphere), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes],1), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high],1),
    round(rmse_change_ankle_kinetics_sphere_signal,1)))

print('Influence of the toes')
print('Knee kinematics and kinetics')
rmse_change_knee_kinematics_toes = 100 - (metrics['RMSE']['kinematics']['knee_angle_r'][new_model_with_toes] / metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high]) * 100
rmse_change_knee_kinetics_toes = 100 - (metrics['RMSE']['kinetics']['knee_angle_r'][new_model_with_toes] / metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high]) * 100
rmse_change_knee_kinematics_toes_signal = np.abs(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_with_toes] - metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high])/signal_range['kinematics']['knee_angle_r']*100
rmse_change_knee_kinetics_toes_signal = np.abs(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_with_toes] - metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high])/signal_range['kinetics']['knee_angle_r']*100

print('RMSE kinematics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinematics_toes), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_without_toes_high],1), 
    round(metrics['RMSE']['kinematics']['knee_angle_r'][new_model_with_toes],1),
    round(rmse_change_knee_kinematics_toes_signal,1)))
print('RMSE kinetics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_knee_kinetics_toes), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_without_toes_high],1), 
    round(metrics['RMSE']['kinetics']['knee_angle_r'][new_model_with_toes],1),
    round(rmse_change_knee_kinetics_toes_signal,1)))
print('Ankle kinematics and kinetics')
rmse_change_ankle_kinematics_toes = 100 - (metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_with_toes] / metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high]) * 100
rmse_change_ankle_kinetics_toes = 100 - (metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_with_toes] / metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high]) * 100
rmse_change_ankle_kinematics_toes_signal = np.abs(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_with_toes] - metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high])/signal_range['kinematics']['ankle_angle_r']*100
rmse_change_ankle_kinetics_toes_signal = np.abs(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_with_toes] - metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high])/signal_range['kinetics']['ankle_angle_r']*100

print('RMSE kinematics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinematics_toes), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_without_toes_high],1), 
    round(metrics['RMSE']['kinematics']['ankle_angle_r'][new_model_with_toes],1),
    round(rmse_change_ankle_kinematics_toes_signal,1)))
print('RMSE kinetics decreased by {} %, from {} to {}, change is {}% of signal range'.format(
    round(rmse_change_ankle_kinetics_toes), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_without_toes_high],1), 
    round(metrics['RMSE']['kinetics']['ankle_angle_r'][new_model_with_toes],1),
    round(rmse_change_ankle_kinetics_toes_signal,1)))

# %% First peak GRF
# Experimental
c_ref_vGRF_max = np.max(experimentalData[subject]["GRF"]["mean"]['GRF_y_r'].to_numpy())
# Simulation with toes
c_sim_toes_vGRF = np.max(optimaltrajectories[new_model_with_toes]['GRF'][1, :].T)
# Simulation without toes
c_sim_noToes_vGRF = np.max(optimaltrajectories[new_model_without_toes_high]['GRF'][1, :].T)

peak_GRF_change = 100 - (c_sim_toes_vGRF/c_sim_noToes_vGRF*100)
print('Peak decreased by {} %, from {} to {}'.format(round(peak_GRF_change), round(c_sim_noToes_vGRF), round(c_sim_toes_vGRF)))

# %% COT comparison
COT_withToes = optimaltrajectories[new_model_with_toes]['COT_perMuscle']
COT_withoutToes = optimaltrajectories[new_model_without_toes_high]['COT_perMuscle']

COT_diff = COT_withToes-COT_withoutToes
sort_COT_idx = np.flip(np.argsort(COT_diff))
muscles_names = optimaltrajectories[new_model_with_toes]['muscles']
sort_COT_muscles = []
for idx in sort_COT_idx:    
    sort_COT_muscles.append(muscles_names[idx])
sort_COT_diff = COT_diff[sort_COT_idx]
COT_quads_added = np.sum(sort_COT_diff[0:8])

COT_withToes_sum = np.sum(COT_withToes)
COT_withoutToes_sum = np.sum(COT_withoutToes)

COT_quads_added / (COT_withToes_sum-COT_withoutToes_sum)*100

np.mean([2045/1248, 2499/849, 2894/1860, 2782/894])
np.mean([4043/1248, 3967/849, 3860/1860, 3783/894])
