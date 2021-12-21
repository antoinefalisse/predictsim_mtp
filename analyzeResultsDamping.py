'''
    This script computes RMSEs for kinematics and kinetics, and compares the
    results across cases. Focus is on studying the influence of the damping
    coefficients on the simulations.
'''

# %% Import packages
import os
import numpy as np
from scipy.interpolate import interp1d
from sklearn.metrics import mean_squared_error, r2_score

# %% Settings
cases = ['4', '38', '40', '34', '37']

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
threshold = 5 # vGRF threshold for stance-swing transition

metrics = {}
metrics['RMSE'] = {}
metrics['R2'] = {}
signal_range = {}

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
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, 100)
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
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, 100)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinematics'][joint][case] = np.round(mean_squared_error(c_ref_inter, c_sim_inter, squared=False), 2)
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
        c_ref_vec_N = np.linspace(0, c_ref_stance.shape[0]-1, 100)
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
        c_sim_vec_N = np.linspace(0, c_sim_stance.shape[0]-1, 100)
        set_interp = interp1d(c_sim_vec, c_sim_stance)
        c_sim_inter = set_interp(c_sim_vec_N)
        
        # Compute RMSE
        metrics['RMSE']['kinetics'][joint][case] = np.round(mean_squared_error(c_ref_inter, c_sim_inter, squared=False), 2)
        metrics['R2']['kinetics'][joint][case] = r2_score(c_ref_inter, c_sim_inter)
        
        
# %% Percent change as a function of signal range
variables = ['kinematics', 'kinetics']
joints = ['knee_angle_r', 'ankle_angle_r']
changes = {}
for variable in variables:
    changes[variable] = {}
    for joint in joints:
        changes[variable][joint] = {}
        for c, case in enumerate(cases[1:]):            
            RMSE_change = metrics['RMSE'][variable][joint][case] - metrics['RMSE'][variable][joint][cases[c]]
            changes[variable][joint][case] = np.round(RMSE_change / signal_range[variable][joint] * 100, 1)
            