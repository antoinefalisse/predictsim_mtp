import os
import numpy as np
import matplotlib.pyplot as plt  

# %% Settings 
cases = ['0','4']
mainName = "predictsim_mtp"

# %% Fixed settings
pathMain = os.getcwd()
# Load results from the IOC
pathTrajectories = os.path.join(pathMain, 'Results', mainName)
optimaltrajectories = np.load(os.path.join(pathTrajectories, 
                                           'optimalTrajectories.npy'),
                              allow_pickle=True).item()
    
# %% Visualize results
plt.close('all')

# %% Joint coordinates
# kinematic_ylim_ub = [20, 1, 1, 50, 50, 20, 20, 30, 30, 60, 60, 20]
# kinematic_ylim_lb = [-20, -1, 0.8, -30, -30, -80, -80, -30, -30, -20, -20, -20]
joints = optimaltrajectories[cases[0]]['joints']
jointToPlot = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 
               'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 
               'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 
               'knee_angle_r',  'ankle_angle_r', 
               'subtalar_angle_r', 'mtp_angle_r', 
               'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
               'arm_flex_r', 'arm_add_r', 'arm_rot_r', 'elbow_flex_r']
from variousFunctions import getJointIndices
idxJointsToPlot = getJointIndices(joints, jointToPlot)
NJointsToPlot = len(jointToPlot)    
fig, axs = plt.subplots(4, 6, sharex=True)    
fig.suptitle('Joint coordinates')
for i, ax in enumerate(axs.flat):
    if i < NJointsToPlot:
        color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
        for case in cases:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['coordinate_values'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T, c=next(color), label='case_' + case)            
        ax.set_title(joints[idxJointsToPlot[i]])
        # ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Coordinate values (deg or m)')
fig.align_ylabels()

# %% Muscle activations
muscles = optimaltrajectories[cases[0]]['muscles']
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
fig, axs = plt.subplots(8, 6, sharex=True)    
fig.suptitle('Muscle activations')
for i, ax in enumerate(axs.flat):
    color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
    if i < NMusclesToPlot:
        for case in cases:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=next(color), label='case_' + case)
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
for i, ax in enumerate(axs.flat):
    color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
    if i < NJointsToPlot:
        for case in cases:
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['joint_torques'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T, c=next(color), label='case_' + case)
        ax.set_title(joints[idxJointsToPlot[i]])
        # ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Torques [Nm]')
fig.align_ylabels()

# %% Contact forces 
# contact_ylim_ub = [300, 1500, 300, 1200]
# contact_ylim_lb = [-300, 0, -300, 0]
GRF_labels = optimaltrajectories[cases[0]]['GRF_labels']
GRFToPlot = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r', 'GRF_x_l','GRF_y_l', 'GRF_z_l']
NGRFToPlot = len(GRFToPlot)
idxGRFToPlot = getJointIndices(GRF_labels, GRFToPlot)
fig, axs = plt.subplots(2, 3, sharex=True)    
fig.suptitle('Ground reaction forces')
#color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))
for i, ax in enumerate(axs.flat):
    color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
    for case in cases:
        ax.plot(optimaltrajectories[case]['GC_percent'],
                optimaltrajectories[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=next(color), label='case_' + case)        
    ax.set_title(GRF_labels[idxGRFToPlot[i]])
    # ax.set_ylim((contact_ylim_lb[i],contact_ylim_ub[i]))
    handles, labels = ax.get_legend_handles_labels()
    plt.legend(handles, labels, loc='upper right')
plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[:, 0], ylabel='Ground reaction forces (N)')
fig.align_ylabels()

# # %% Kinematic tracking    
# # Half gait cycle
# kinematic_ylim_ub = [20, 1, 1, 50, 50, 20, 20, 30, 30, 60, 60, 20]
# kinematic_ylim_lb = [-20, -1, 0.8, -30, -30, -80, -80, -30, -30, -20, -20, -20]
# if settings[idx_settings]['measuredReference']: 
#     joints = referenceResults['case_' + str(idxSettingsReference[0])][idxTrials[0]]['joints']
# else:
#     joints = iocResults['reference']['joints']
    
# fig, axs = plt.subplots(3, 4, sharex=True)    
# fig.suptitle('Joint kinematics')
# #color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))   
# for i, ax in enumerate(axs.flat):
#     if settings[idx_settings]['measuredReference']:  
#         for w in idxSettingsReference:            
#             if not settingsReference[str(w)]['trackPrediction']:            
#                 trial = settingsReference[str(w)]['trials_select'][0]
#             else:
#                 trial = (settingsReference[str(w)]['trials_select'][0] + "_" 
#                          + settingsReference[str(w)]['f_pred'] + "_"
#                          + settingsReference[str(w)]['idxSpecificInitialPool'] + "_"
#                          + settingsReference[str(w)]['weightOptimizationFormulation'] + "_"
#                          + settingsReference[str(w)]['errorScaling'])
#             if referenceResults['case_' + str(w)][trial]['leg'] == 'left':
#                 if joints[i][-1] == 'l':
#                     j = joints.index(joints[i][:-1] + 'r')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['coordinates'][j:j+1, :].T, c='k', label='case_' + str(w)) 
#                 elif joints[i][-1] == 'r':
#                     j = joints.index(joints[i][:-1] + 'l')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['coordinates'][j:j+1, :].T, c='k', label='case_' + str(w))      
#                 else:
#                     if joints[i] == "pelvis_tx":
#                         y = referenceResults['case_' + str(w)][trial]['coordinates'][i:i+1, :].T
#                         y = y - y[0]
#                         ax.plot(y, c='k', label='case_' + str(w))                     
#                     else:
#                         ax.plot(referenceResults['case_' + str(w)][trial]['coordinates'][i:i+1, :].T, c='k', label='case_' + str(w))  
#             else:
#                 if joints[i] == "pelvis_tx":
#                     y = referenceResults['case_' + str(w)][trial]['coordinates'][i:i+1, :].T
#                     y = y - y[0]
#                     ax.plot(y, c='k', label='case_' + str(w))                
#                 else:                    
#                     ax.plot(referenceResults['case_' + str(w)][trial]['coordinates'][i:i+1, :].T, c='k', label='case_' + str(w))  
#     else:
#         ax.plot(iocResults['reference']['coordinate_values'][i:i+1, :].T, c='k', label='reference')
#     for ioci in idx_settings_ioc:
#         ax.plot(iocResults[ioci]['coordinate_values'][i:i+1, :].T, c='r', label='case_' + ioci)            
#     ax.set_title(joints[i])
#     ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
# #    handles, labels = ax.get_legend_handles_labels()
# #    plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# plt.setp(axs[:, 0], ylabel='Coordinate values (deg or m)')
# fig.align_ylabels()
    
# # %% Contact force tracking  
# contact_ylim_ub = [300, 1500, 300, 1200]
# contact_ylim_lb = [-300, 0, -300, 0]
# contactNames = ['GRF_x_r', 'GRF_y_r', 'GRF_x_l', 'GRF_y_l']
# pltCFi_r = [0, 1, 2, 3]
# pltCFi_l = [2, 3, 0, 1]
# fig, axs = plt.subplots(2, 2, sharex=True)    
# fig.suptitle('Contact forces and torques')
# #color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))
# for i, ax in enumerate(axs.flat):
#     if settings[idx_settings]['measuredReference']: 
#         for w in idxSettingsReference:
#             if not settingsReference[str(w)]['trackPrediction']:            
#                 trial = settingsReference[str(w)]['trials_select'][0]
#             else:
#                 trial = (settingsReference[str(w)]['trials_select'][0] + "_" 
#                          + settingsReference[str(w)]['f_pred'] + "_"
#                          + settingsReference[str(w)]['idxSpecificInitialPool'] + "_"
#                          + settingsReference[str(w)]['weightOptimizationFormulation'] + "_"
#                          + settingsReference[str(w)]['errorScaling'])
#             if referenceResults['case_' + str(w)][trial]['leg'] == 'left':
#                 ax.plot(referenceResults['case_' + str(w)][trial]['contacts'][pltCFi_l[i]:pltCFi_l[i]+1, :].T, c='k', label='case_' + str(w)) 
#             else:
#                 ax.plot(referenceResults['case_' + str(w)][trial]['contacts'][pltCFi_r[i]:pltCFi_r[i]+1, :].T, c='k', label='case_' + str(w))
#     else:
#         ax.plot(iocResults['reference']['contact_forces'][i:i+1, :].T, c='k', label='reference')     
#     for ioci in idx_settings_ioc:
#         ax.plot(iocResults[ioci]['contact_forces'][i:i+1, :].T, c='r', label='case_' + ioci)        
#     ax.set_title(contactNames[pltCFi_r[i]])
#     ax.set_ylim((contact_ylim_lb[i],contact_ylim_ub[i]))
# ##    handles, labels = ax.get_legend_handles_labels()
# ##    plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# plt.setp(axs[:, 0], ylabel='Contact forces and torques (N or Nm)')
# fig.align_ylabels()
# #    
# # %% Kinetics
# kinetic_ylim_ub = [40,40,40,60,60,80,80,20,20,10,10,60]
# kinetic_ylim_lb = [-50,-50,-50,-80,-80,-50,-50,-100,-100,-50,-50,-20]
# fig, axs = plt.subplots(3, 4, sharex=True)    
# fig.suptitle('Joint kinetics')
# #color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))   
# for i, ax in enumerate(axs.flat):
#     if settings[idx_settings]['measuredReference']: 
#         for w in idxSettingsReference:
#             if not settingsReference[str(w)]['trackPrediction']:            
#                 trial = settingsReference[str(w)]['trials_select'][0]
#             else:
#                 trial = (settingsReference[str(w)]['trials_select'][0] + "_" 
#                          + settingsReference[str(w)]['f_pred'] + "_"
#                          + settingsReference[str(w)]['idxSpecificInitialPool'] + "_"
#                          + settingsReference[str(w)]['weightOptimizationFormulation'] + "_"
#                          + settingsReference[str(w)]['errorScaling'])
#             if referenceResults['case_' + str(w)][trial]['leg'] == 'left':
#                 if joints[i][-1] == 'l':
#                     j = joints.index(joints[i][:-1] + 'r')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['torques'][j:j+1, :].T, c='k', label='case_' + str(w)) 
#                 elif joints[i][-1] == 'r':
#                     j = joints.index(joints[i][:-1] + 'l')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['torques'][j:j+1, :].T, c='k', label='case_' + str(w))      
#                 else:
#                     ax.plot(referenceResults['case_' + str(w)][trial]['torques'][i:i+1, :].T, c='k', label='case_' + str(w))  
#             else:                       
#                 ax.plot(referenceResults['case_' + str(w)][trial]['torques'][i:i+1, :].T, c='k', label='case_' + str(w))  
#     else:
#         ax.plot(iocResults['reference']['joint_torques'][i:i+1, :].T, c='k', label='reference') 
#     for ioci in idx_settings_ioc:
#         ax.plot(iocResults[ioci]['joint_torques'][i:i+1, :].T, c='r', label='case_' + ioci)
#     ax.set_title(joints[i])
#     ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
# #    handles, labels = ax.get_legend_handles_labels()
# #    plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# plt.setp(axs[:, 0], ylabel='Torques [Nm]')
# fig.align_ylabels()
# #
# # %% Muscle activations
# if settings[idx_settings]['measuredReference']: 
#     muscles = referenceResults['case_' + str(idxSettingsReference[0])][idxTrials[0]]['muscles']
# else:
#     muscles = iocResults['reference']['muscles']
# fig, axs = plt.subplots(3, 6, sharex=True)    
# fig.suptitle('Muscle activations')
# # color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))  
# for i, ax in enumerate(axs.flat):
#     if settings[idx_settings]['measuredReference']: 
#         for w in idxSettingsReference:
#             if not settingsReference[str(w)]['trackPrediction']:            
#                 trial = settingsReference[str(w)]['trials_select'][0]
#             else:
#                 trial = (settingsReference[str(w)]['trials_select'][0] + "_" 
#                          + settingsReference[str(w)]['f_pred'] + "_"
#                          + settingsReference[str(w)]['idxSpecificInitialPool'] + "_"
#                          + settingsReference[str(w)]['weightOptimizationFormulation'] + "_"
#                          + settingsReference[str(w)]['errorScaling'])          
#             if referenceResults['case_' + str(w)][trial]['leg'] == 'left':
#                 if muscles[i][-1] == 'l':
#                     j = muscles.index(muscles[i][:-1] + 'r')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['activations'][j:j+1, :].T, c='k', label='case_' + str(w)) 
#                 elif muscles[i][-1] == 'r':
#                     j = muscles.index(muscles[i][:-1] + 'l')
#                     ax.plot(referenceResults['case_' + str(w)][trial]['activations'][j:j+1, :].T, c='k', label='case_' + str(w))   
#             else:                       
#                 ax.plot(referenceResults['case_' + str(w)][trial]['activations'][i:i+1, :].T, c='k', label='case_' + str(w))
#     else:
#         ax.plot(iocResults['reference']['muscle_activations'][i:i+1, :].T, c='k', label='reference')
#     for ioci in idx_settings_ioc:
#         ax.plot(iocResults[ioci]['muscle_activations'][i:i+1, :].T, c='r', label='case_' + ioci)
#     ax.set_title(muscles[i])
#     ax.set_ylim((0,1))
# #    handles, labels = ax.get_legend_handles_labels()
# #    plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# plt.setp(axs[:, 0], ylabel='Activation []')
# fig.align_ylabels()
# #    
# ## %% Muscle fiber lengths
# #trial = settingsReference[str(trials[list(trials.keys())[0]]['case'])]['trials_select'][0]  
# #muscles = referenceResults['case_' + str(trials[list(trials.keys())[0]]['case'])][trial]['muscles']
# #fig, axs = plt.subplots(3, 6, sharex=True)    
# #fig.suptitle('Normalized fiber lenghts')
# #color=iter(plt.cm.rainbow(np.linspace(0,1,len(trials))))
# #for count, w in enumerate(idxSettingsReference):
# #    trial = settingsReference[str(w)]['trials_select'][0]    
# #    for i, ax in enumerate(axs.flat):
# #        if referenceResults['case_' + str(w)][trial]['leg'] == 'left':
# #            if muscles[i][-1] == 'l':
# #                j = muscles.index(muscles[i][:-1] + 'r')
# #                ax.plot(referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][j:j+1, :].T, c='k', label='case_' + str(w)) 
# #            elif muscles[i][-1] == 'r':
# #                j = muscles.index(muscles[i][:-1] + 'l')
# #                ax.plot(referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][j:j+1, :].T, c='k', label='case_' + str(w))   
# #        else:   
# #            j = muscles.index(muscles[i])                    
# #            ax.plot(referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][i:i+1, :].T, c='k', label='case_' + str(w))   
# #        ax.set_title(referenceResults['case_' + str(w)][trial]['muscles'][i])
# #        ax.set_ylim((0,2))
# #        ax.hlines(0.4,0,referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][j:j+1, :].shape[1],'k',linestyle='--')
# #        ax.hlines(1.6,0,referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][j:j+1, :].shape[1],'k',linestyle='--')
# #        ax.hlines(1,0,referenceResults['case_' + str(w)][trial]['normalized_fiber_lengths'][j:j+1, :].shape[1],'k',linestyle=':')
# ##    handles, labels = ax.get_legend_handles_labels()
# ##    plt.legend(handles, labels, loc='upper right')
# #    plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# #    plt.setp(axs[:, 0], ylabel='Lengths []')
# #    fig.align_ylabels()