import os
import numpy as np
import matplotlib.pyplot as plt  

# %% Settings 
cases_mtp = ['7', '0']
cases_no_mtp = ['3', '0']
subject = "subject1"
model_mtp = "mtp"
model_no_mtp = "no_mtp"

# %% Fixed settings
pathMain = os.getcwd()
# Load results
mainName = "predictsim_mtp"
pathTrajectories = os.path.join(pathMain, 'Results', mainName)
optimaltrajectories = np.load(os.path.join(pathTrajectories, 
                                           'optimalTrajectories.npy'),
                              allow_pickle=True).item()
# Load experimental data
pathData = os.path.join(pathMain, 'OpenSimModel', subject + "_" + model_mtp)
experimentalData_mtp = np.load(os.path.join(pathData, 'experimentalData.npy'),
                               allow_pickle=True).item()
# Load results
mainName = "predictsim_no_mtp"
pathTrajectories = os.path.join(pathMain, 'Results', mainName)
optimaltrajectories_no_mtp = np.load(os.path.join(pathTrajectories, 
                                           'optimalTrajectories.npy'),
                              allow_pickle=True).item()
# Load experimental data
pathData = os.path.join(pathMain, 'OpenSimModel', subject + "_" + model_no_mtp)
experimentalData_no_mtp = np.load(os.path.join(pathData, 
                                               'experimentalData.npy'),
                                  allow_pickle=True).item()
    
# %% Visualize results
plt.close('all')

# %% Joint coordinates
# kinematic_ylim_ub = [20, 1, 1, 50, 50, 20, 20, 30, 30, 60, 60, 20]
# kinematic_ylim_lb = [-20, -1, 0.8, -30, -30, -80, -80, -30, -30, -20, -20, -20]
joints = optimaltrajectories[cases_mtp[0]]['joints']
jointsToPlot = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 
               'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 
               'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
               'knee_angle_r',  'ankle_angle_r', 
               'subtalar_angle_r', 'mtp_angle_r',
               'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
               'arm_flex_r', 'arm_add_r', 'arm_rot_r', 'elbow_flex_r']
joints_no_mtp = optimaltrajectories_no_mtp[cases_no_mtp[0]]['joints']
jointsToPlot_no_mtp = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 
               'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 
               'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
               'knee_angle_r',  'ankle_angle_r', 
               'subtalar_angle_r',
               'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
               'arm_flex_r', 'arm_add_r', 'arm_rot_r', 'elbow_flex_r']
from variousFunctions import getJointIndices
idxJointsToPlot = getJointIndices(joints, jointsToPlot)
idxJointsToPlot_no_mtp = getJointIndices(joints_no_mtp, jointsToPlot_no_mtp)

NJointsToPlot = len(jointsToPlot)  
NJointsToPlot_no_mtp = len(jointsToPlot_no_mtp)   
fig, axs = plt.subplots(4, 6, sharex=True)    
fig.suptitle('Joint coordinates')
count = 0 
for i, ax in enumerate(axs.flat):
    color_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))  
    color_no_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_no_mtp))))  
    if i < NJointsToPlot:
        for case in cases_mtp:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['coordinate_values'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T, c=next(color_mtp), label='case_' + case + '_mtp')
            ax.fill_between(experimentalData_mtp[subject]["kinematics"]["positions"]["GC_percent"],
                        experimentalData_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] + 2*experimentalData_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                        experimentalData_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] - 2*experimentalData_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                        facecolor='blue', alpha=0.5)                  
        if not i == jointsToPlot.index("mtp_angle_r"):
            for case in cases_no_mtp:
                ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
                        optimaltrajectories_no_mtp[case]['coordinate_values'][idxJointsToPlot_no_mtp[count]:idxJointsToPlot_no_mtp[count]+1, :].T, c=next(color_no_mtp), linestyle='dashed', label='case_' + case + '_no_mtp')   
                ax.fill_between(experimentalData_no_mtp[subject]["kinematics"]["positions"]["GC_percent"],
                        experimentalData_no_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot_no_mtp[count]] + 2*experimentalData_no_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot_no_mtp[count]],
                        experimentalData_no_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot_no_mtp[count]] - 2*experimentalData_no_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot_no_mtp[count]],
                        facecolor='orange', alpha=0.5)
            count += 1                
        ax.set_title(joints[idxJointsToPlot[i]])
        # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Coordinate values (deg or m)')
fig.align_ylabels()

# %% Muscle activations
muscles = optimaltrajectories[cases_mtp[0]]['muscles']
musclesToPlot = ['glut_med1_r', 'glut_med2_r', 'glut_med3_r', 'glut_min1_r', 
                 'glut_min2_r', 'glut_min3_r', 'semimem_r', 'semiten_r', 
                 'bifemlh_r', 'bifemsh_r', 'sar_r', 'add_long_r', 'add_brev_r',
                 'add_mag1_r', 'add_mag2_r', 'add_mag3_r', 'tfl_r', 'pect_r',
                 'grac_r', 'glut_max1_r', 'glut_max2_r', 'glut_max3_r',
                 'iliacus_r', 'psoas_r', 'quad_fem_r', 'gem_r', 'peri_r',
                 'rect_fem_r', 'vas_med_r', 'vas_int_r', 'vas_lat_r',
                 'med_gas_r', 'lat_gas_r', 'soleus_r', 'tib_post_r',
                 'flex_dig_r', 'flex_hal_r', 'tib_ant_r', 'per_brev_r',
                 'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r',
                 'ercspn_r', 'intobl_r', 'extobl_r',]
NMusclesToPlot = len(musclesToPlot)
idxMusclesToPlot = getJointIndices(muscles, musclesToPlot)
mappingEMG = {'glut_med1_r': 'GluMed_r', 
              'glut_med2_r': 'GluMed_r', 
              'glut_med3_r': 'GluMed_r',
              'semimem_r': 'HamM_r',
              'semiten_r': 'HamM_r',
              'bifemlh_r': 'HamL_r',
              'bifemsh_r': 'HamL_r',
              'add_long_r': 'AddL_r',
              'tfl_r': 'TFL_r',
              'rect_fem_r': 'RF_r',
              'vas_med_r': 'VM_r',
              'vas_int_r': 'VL_r',
              'vas_lat_r': 'VL_r',
              'med_gas_r': 'GM_r',
              'lat_gas_r': 'GL_r',
              'soleus_r': 'Sol_r',
              'tib_ant_r': 'TA_r',
              'per_brev_r': 'PerB_l',
              'per_long_r': 'PerL_l',
              'glut_med1_l': 'GluMed_l', 
              'glut_med2_l': 'GluMed_l', 
              'glut_med3_l': 'GluMed_l',
              'semimem_l': 'HamM_l',
              'semiten_l': 'HamM_l',
              'bifemlh_l': 'HamL_l',
              'bifemsh_l': 'HamL_l',
              'add_long_l': 'AddL_l',
              'tfl_l': 'TFL_l',
              'rect_fem_l': 'RF_l',
              'vas_med_l': 'VM_l',
              'vas_int_l': 'VL_l',
              'vas_lat_l': 'VL_l',
              'med_gas_l': 'GM_l',
              'lat_gas_l': 'GL_l',
              'soleus_l': 'Sol_l',
              'tib_ant_l': 'TA_l',
              'per_brev_l': 'PerB_l',
              'per_long_l': 'PerL_l'}

fig, axs = plt.subplots(8, 6, sharex=True)    
fig.suptitle('Muscle activations')
for i, ax in enumerate(axs.flat):
    color_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))  
    color_no_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_no_mtp))))  
    if i < NMusclesToPlot:
        for case in cases_mtp:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=next(color_mtp), label='case_' + case + '_mtp')  
            if musclesToPlot[i] in mappingEMG:                
                # Normalize EMG such that peak mean EMG = peak activation             
                exp_mean = experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]]
                exp_mean_peak = np.max(exp_mean)
                sim = optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i], :].T
                sim_peak = np.max(sim)
                scaling_emg = sim_peak / exp_mean_peak
                ax.fill_between(experimentalData_mtp[subject]["EMG"]["GC_percent"],
                        experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg + 2*experimentalData_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg - 2*experimentalData_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        facecolor='blue', alpha=0.5)            
            
        for case in cases_no_mtp:
            ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
                    optimaltrajectories_no_mtp[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=next(color_no_mtp), linestyle='dashed', label='case_' + case + '_no_mtp')            
        
            if musclesToPlot[i] in mappingEMG:                
                # Normalize EMG such that peak mean EMG = peak activation             
                exp_mean = experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]]
                exp_mean_peak = np.max(exp_mean)
                sim = optimaltrajectories_no_mtp[case]['muscle_activations'][idxMusclesToPlot[i], :].T
                sim_peak = np.max(sim)
                scaling_emg = sim_peak / exp_mean_peak
                ax.fill_between(experimentalData_no_mtp[subject]["EMG"]["GC_percent"],
                        experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg + 2*experimentalData_no_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg - 2*experimentalData_no_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        facecolor='orange', alpha=0.5)      
        
        ax.set_title(muscles[idxMusclesToPlot[i]])
        ax.set_ylim((0,1))
    handles, labels = ax.get_legend_handles_labels()
    plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Activation []')
fig.align_ylabels()

# %% Kinetics
# kinetic_ylim_ub = [40,40,40,60,60,80,80,20,20,10,10,60]
# kinetic_ylim_lb = [-50,-50,-50,-80,-80,-50,-50,-100,-100,-50,-50,-20]
fig, axs = plt.subplots(4, 6, sharex=True)    
fig.suptitle('Joint kinetics')
count = 0 
for i, ax in enumerate(axs.flat):
    color_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))  
    color_no_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_no_mtp))))  
    if i < NJointsToPlot:
        for case in cases_mtp:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['joint_torques'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T, c=next(color_mtp), label='case_' + case + '_mtp')  
            ax.fill_between(experimentalData_mtp[subject]["kinetics"]["GC_percent"],
                            experimentalData_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] + 2*experimentalData_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                            experimentalData_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] - 2*experimentalData_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                            facecolor='blue', alpha=0.5)            
        if not i == jointsToPlot.index("mtp_angle_r"):
            for case in cases_no_mtp:
                ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
                        optimaltrajectories_no_mtp[case]['joint_torques'][idxJointsToPlot_no_mtp[count]:idxJointsToPlot_no_mtp[count]+1, :].T, c=next(color_no_mtp), linestyle='dashed', label='case_' + case + '_no_mtp')            
                ax.fill_between(experimentalData_no_mtp[subject]["kinetics"]["GC_percent"],
                            experimentalData_no_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] + 2*experimentalData_no_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                            experimentalData_no_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] - 2*experimentalData_no_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                            facecolor='orange', alpha=0.5)      
            count += 1  
        ax.set_title(joints[idxJointsToPlot[i]])
        # ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Torques [Nm]')
fig.align_ylabels()

# %% Ground reaction forces
GRF_labels = optimaltrajectories[cases_mtp[0]]['GRF_labels']
GRFToPlot = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r', 'GRF_x_l','GRF_y_l', 'GRF_z_l']
NGRFToPlot = len(GRFToPlot)
idxGRFToPlot = getJointIndices(GRF_labels, GRFToPlot)
fig, axs = plt.subplots(2, 3, sharex=True)    
fig.suptitle('Ground reaction forces')
for i, ax in enumerate(axs.flat):
    color_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))  
    color_no_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_no_mtp))))  
    if i < NJointsToPlot:
        for case in cases_mtp:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=next(color_mtp), label='case_' + case + '_mtp')  
            ax.fill_between(experimentalData_mtp[subject]["GRF"]["GC_percent"],
                            experimentalData_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] + 2*experimentalData_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
                            experimentalData_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] - 2*experimentalData_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
                            facecolor='blue', alpha=0.5)
        for case in cases_no_mtp:
            ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
                    optimaltrajectories_no_mtp[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=next(color_no_mtp), linestyle='dashed', label='case_' + case + '_no_mtp')  
            ax.fill_between(experimentalData_no_mtp[subject]["GRF"]["GC_percent"],
                            experimentalData_no_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] + 2*experimentalData_no_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
                            experimentalData_no_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] - 2*experimentalData_no_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
                            facecolor='blue', alpha=0.5)
        ax.set_title(GRF_labels[idxGRFToPlot[i]])
        # ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Ground reaction forces [N]')
fig.align_ylabels()
