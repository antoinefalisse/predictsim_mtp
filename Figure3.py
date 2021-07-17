import os
import numpy as np
import matplotlib.pyplot as plt  

# %% Settings 
# # Effect of mtp on generic_CMO (N=50)
# cases_mtp = ['7']
# cases_no_mtp = ['3']
# # Effect of mtp on specific_CM3 (N=50)
# cases_mtp = ['0']
# cases_no_mtp = ['0']

# Effect of mtp on generic_CMO (N=100)
# cases_mtp = ['145','151','153','155','157','159','161','163']
cases_mtp = ['145','151','153','156','157','159','162','163']
cases_no_mtp = []
# # Effect of mtp on specific_CM3 (N=100)
# cases_mtp = ['3']
# cases_no_mtp = ['4']

subject = "subject2"
model_mtp = "mtp"
model_no_mtp = "no_mtp"

color_mtp=['blue'] 
linestyle_mtp=[':']
color_no_mtp=['black','orange'] 
linestyle_no_mtp=['-','--']
linewidth_s = 3
fontsize_tick = 14
fontsize_label = 15
fontsize_title = 17

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
jointTitles = ['Pelvis tilt', 'Pelvis list', 'Pelvis rotation', 'Pelvis tx', 
              'Pelvis ty', 'Pelvis tz', 'Hip flexion', 'Hip adduction', 
              'Hip rotation', 'Hip flexion', 'Hip adduction', 
              'Hip rotation', 'Knee', 'Knee', 'Ankle', 
              'Ankle', 'Subtalar', 'Subtalar', 
              'Toe', 'Toe', 
              'Lumbar extension', 'Lumbar bending', 'Lumbar rotation', 
              'Arm flex', 'Arm add', 'Arm rot', 'Arm flex', 'Arm add', 
              'Arm rot', 'Elbow flex', 'Elbow flex']
kinematic_ylim_ub = [20, 30,]
kinematic_ylim_lb = [-80, -30]
joints = optimaltrajectories[cases_mtp[0]]['joints']
jointsToPlot = ['knee_angle_r',  'ankle_angle_r']
# joints_no_mtp = optimaltrajectories_no_mtp[cases_no_mtp[0]]['joints']
# jointsToPlot_no_mtp = ['knee_angle_r',  'ankle_angle_r']

from variousFunctions import getJointIndices
idxJointsToPlot = getJointIndices(joints, jointsToPlot)
# idxJointsToPlot_no_mtp = getJointIndices(joints_no_mtp, jointsToPlot_no_mtp)

NJointsToPlot = len(jointsToPlot)  
# NJointsToPlot_no_mtp = len(jointsToPlot_no_mtp)   
fig, axs = plt.subplots(3, 5, sharex=False)    
# fig.suptitle('Joint coordinates')
count = 0 
for i, ax in enumerate(axs[0, :]):     
    if i < NJointsToPlot:
        color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))   
        # for c_no_mtp, case in enumerate(cases_no_mtp):
        #     ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
        #             optimaltrajectories_no_mtp[case]['coordinate_values'][idxJointsToPlot_no_mtp[count]:idxJointsToPlot_no_mtp[count]+1, :].T, c=color_no_mtp[c_no_mtp], linestyle=linestyle_no_mtp[c_no_mtp], linewidth=linewidth_s, label='case_' + case + '_no_mtp')   
        #     # ax.fill_between(experimentalData_no_mtp[subject]["kinematics"]["positions"]["GC_percent"],
        #     #         experimentalData_no_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot_no_mtp[count]] + 2*experimentalData_no_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot_no_mtp[count]],
        #     #         experimentalData_no_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot_no_mtp[count]] - 2*experimentalData_no_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot_no_mtp[count]],
        #     #         facecolor='orange', alpha=0.4)
        # count += 1 
        for c_mtp, case in enumerate(cases_mtp):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['coordinate_values'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T,color=next(color), linewidth=linewidth_s, label='case_' + case + '_mtp')
        ax.fill_between(experimentalData_mtp[subject]["kinematics"]["positions"]["GC_percent"],
                    experimentalData_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] + 2*experimentalData_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                    experimentalData_mtp[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] - 2*experimentalData_mtp[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                    facecolor='grey', alpha=0.4)                  
            
                           
        ax.set_title(jointTitles[idxJointsToPlot[i]])
        ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
        ax.set_yticks([kinematic_ylim_lb[i],0,kinematic_ylim_ub[i]])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        # if i == NJointsToPlot-1:
        #     ax.set_xticks([0,50,100])
        #     plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
        # else:
        ax.set_xticks([0,50,100]) 
        ax.set_xticklabels([]) 
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        handles, labels = ax.get_legend_handles_labels()
        # plt.legend(handles, labels, loc='upper right')
plt.setp(axs[1, :4], xlabel='Gait cycle (%)')
# plt.setp(axs[1, 2:], xlabel='Gait cycle (%)')
plt.setp(axs[0:1, 0], ylabel='Joint angle (deg)')
fig.align_ylabels()

# %% Muscle activations
muscleTitles = ['glut_med1_r', 'Gluteus medius', 'glut_med3_r', 'Gluteus minimus', 
                'glut_min2_r', 'glut_min3_r', 'semimem_r', 'Semitendinosus', 'bifemlh_r',
                'Biceps femoris short head', 'sar_r', 'add_long_r', 'add_brev_r', 'add_mag1_r',
                'add_mag2_r', 'add_mag3_r', 'tfl_r', 'pect_r', 'grac_r', 
                'glut_max1_r', 'glut_max2_r', 'glut_max3_r', 'iliacus_r', 'psoas_r',
                'quad_fem_r', 'gem_r', 'peri_r', 'rect_fem_r', 'Vastus medialis', 
                'vas_int_r', 'Vastus lateralis', 'Gastrocnemius med', 'lat_gas_r', 'Soleus',
                'tib_post_r', 'flex_dig_r', 'flex_hal_r', 'Tibialis anterior', 'per_brev_r',
                'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r', 'ercspn_r', 
                'intobl_r', 'extobl_r',
                'glut_med1_r', 'Gluteus medius', 'glut_med3_r', 'Gluteus minimus', 
                'glut_min2_r', 'glut_min3_r', 'semimem_r', 'Semitendinosus', 'bifemlh_r',
                'Biceps femoris sh', 'sar_r', 'add_long_r', 'add_brev_r', 'add_mag1_r',
                'add_mag2_r', 'add_mag3_r', 'tfl_r', 'pect_r', 'grac_r', 
                'glut_max1_r', 'glut_max2_r', 'glut_max3_r', 'iliacus_r', 'psoas_r',
                'quad_fem_r', 'gem_r', 'peri_r', 'Rectus femoris', 'Vastus medialis', 
                'vas_int_r', 'Vastus lateralis', 'Gastrocnemius med', 'lat_gas_r', 'Soleus',
                'tib_post_r', 'flex_dig_r', 'flex_hal_r', 'Tibialis anterior', 'per_brev_r',
                'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r', 'ercspn_r', 
                'intobl_r', 'extobl_r']
muscles = optimaltrajectories[cases_mtp[0]]['muscles']
musclesToPlot = ['med_gas_r', 'soleus_r']
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

# fig, axs = plt.subplots(8, 6, sharex=True)    
# fig.suptitle('Muscle activations')
for i, ax in enumerate(axs[0:1,2:].flat):
    # color_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))  
    # color_no_mtp=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_no_mtp))))  
    if i < NMusclesToPlot:
        # for c_no_mtp, case in enumerate(cases_no_mtp):
        #     ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
        #             optimaltrajectories_no_mtp[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=color_no_mtp[c_no_mtp], linestyle=linestyle_no_mtp[c_no_mtp], linewidth=linewidth_s, label='case_' + case + '_no_mtp')
        
        color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))
        for c_mtp, case in enumerate(cases_mtp):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=next(color), linewidth=linewidth_s, label='case_' + case + '_mtp')  
                
            
            if c_mtp == 0:
            
                # scaling_emg = 0
                if musclesToPlot[i] in mappingEMG:                
                    # Normalize EMG such that peak mean EMG = peak activation             
                    exp_mean = experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]]
                    exp_mean_peak = np.max(exp_mean)
                    sim = optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i], :].T
                    sim_peak = np.max(sim)
                    scaling_emg = sim_peak / exp_mean_peak
                    
                    # if scaling_emg_t > scaling_emg:
                    #     scaling_emg = scaling_emg_t
                    
                ax.fill_between(experimentalData_mtp[subject]["EMG"]["GC_percent"],
                        experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg + 2*experimentalData_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        experimentalData_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg - 2*experimentalData_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        facecolor='grey', alpha=0.4)            
            
                    
        
            # if musclesToPlot[i] in mappingEMG:                
            #     # Normalize EMG such that peak mean EMG = peak activation             
            #     exp_mean = experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]]
            #     exp_mean_peak = np.max(exp_mean)
            #     sim = optimaltrajectories_no_mtp[case]['muscle_activations'][idxMusclesToPlot[i], :].T
            #     sim_peak = np.max(sim)
            #     scaling_emg = sim_peak / exp_mean_peak
            #     ax.fill_between(experimentalData_no_mtp[subject]["EMG"]["GC_percent"],
            #             experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg + 2*experimentalData_no_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
            #             experimentalData_no_mtp[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg - 2*experimentalData_no_mtp[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
            #             facecolor='orange', alpha=0.4)      
        
        ax.set_title(muscleTitles[idxMusclesToPlot[i]])
        ax.set_ylim((0,1))
        ax.set_yticks([0,1])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks([0,50,100])
        # if i < NMusclesToPlot and i > 5:
        #     ax.set_xticks([0,50,100])
        #     ax.set_xticklabels(['0','50','100'])
        #     ax.set_xlabel('Gait cycle (%)')
        # else:
        ax.set_xticks([0,50,100]) 
        ax.set_xticklabels([]) 
    # handles, labels = ax.get_legend_handles_labels()
    # plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[0:1, 2], ylabel='Muscle activation (-)')
fig.align_ylabels()

# %% Kinetics
kinetic_ylim_ub = [70,30]
kinetic_ylim_lb = [-50,-110]
# fig, axs = plt.subplots(4, 6, sharex=True)    
# fig.suptitle('Joint kinetics')
count = 0 
for i, ax in enumerate(axs[1,:]):
    if i < NJointsToPlot:
        # for c_no_mtp, case in enumerate(cases_no_mtp):
        #     ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
        #             optimaltrajectories_no_mtp[case]['joint_torques'][idxJointsToPlot_no_mtp[count]:idxJointsToPlot_no_mtp[count]+1, :].T, c=color_no_mtp[c_no_mtp], linestyle=linestyle_no_mtp[c_no_mtp], linewidth=linewidth_s, label='case_' + case + '_no_mtp')            
        #     # ax.fill_between(experimentalData_no_mtp[subject]["kinetics"]["GC_percent"],
        #     #             experimentalData_no_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] + 2*experimentalData_no_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
        #     #             experimentalData_no_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] - 2*experimentalData_no_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
        #     #             facecolor='orange', alpha=0.4)      
        # count += 1  
        color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))
        for c_mtp, case in enumerate(cases_mtp):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['joint_torques'][idxJointsToPlot[i]:idxJointsToPlot[i]+1, :].T, c=next(color), linewidth=linewidth_s, label='case_' + case + '_mtp')  
        ax.fill_between(experimentalData_mtp[subject]["kinetics"]["GC_percent"],
                        experimentalData_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] + 2*experimentalData_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                        experimentalData_mtp[subject]["kinetics"]["mean"][jointsToPlot[i]] - 2*experimentalData_mtp[subject]["kinetics"]["std"][jointsToPlot[i]],
                        facecolor='grey', alpha=0.4)
            
        # ax.set_title(jointTitles[idxJointsToPlot[i]])
        ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
        ax.set_yticks([kinetic_ylim_lb[i],0,kinetic_ylim_ub[i]])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        # if i == NJointsToPlot-1:
        ax.set_xticks([0,50,100])
        plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
        # else:
        #     ax.set_xticks([0,50,100]) 
        #     ax.set_xticklabels([]) 
        # handles, labels = ax.get_legend_handles_labels()
        # plt.legend(handles, labels, loc='upper right')
# plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
plt.setp(axs[1, 0], ylabel='Joint torque (Nm)')
fig.align_ylabels()

# %% Muscle fiber lengths
# NMusclesToPlot = len(musclesToPlot)
# idxMusclesToPlot = getJointIndices(muscles, musclesToPlot)
# fig, axs = plt.subplots(8, 6, sharex=True)    
# fig.suptitle('Normalized muscle fiber lengths')
for i, ax in enumerate(axs[1:2,2:].flat):
    # color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases))))   
    if i < NMusclesToPlot:
        color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))
        for c_mtp, case in enumerate(cases_mtp):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['norm_fiber_lengths'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=next(color), linewidth=linewidth_s, label='case_' + case)
        # ax.set_title(muscleTitles[idxMusclesToPlot[i]])
        ax.set_ylim((0,2))
        ax.set_yticks([0,1,2])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks([0,50,100])
        plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        
        # if i < NMusclesToPlot and i > 5:
        #     ax.set_xticks([0,50,100])
        #     ax.set_xticklabels(['0','50','100'])
        #     ax.set_xlabel('Gait cycle (%)')
        # else:
        #     ax.set_xticks([0,50,100]) 
        #     ax.set_xticklabels(['0','50','100'])
            # ax.set_xticklabels([])
plt.setp(axs[1:2, 2], ylabel='Fiber length (-)')
plt.setp(axs[1, :4], xlabel='Gait cycle (%)')
fig.align_ylabels()

# %% Metabolic cost and cost function value
ax = axs[0,4]    
color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))
for c_mtp, case in enumerate(cases_mtp):
    # print(optimaltrajectories[case]["COT"])
    # print(optimaltrajectories[case]["objective"])
    ax.scatter(c_mtp, optimaltrajectories[case]["COT"], s=80, color=next(color))
# ax.set_title("Metabolic cost of transport")
ax.set_ylabel("COT (J/Kg/m)", fontsize=fontsize_label)
x_locations = np.linspace(0, len(cases_mtp)-1, len(cases_mtp))
ax.set_xticks(x_locations)
xticklabels = ["", "", "", "", "", "", "", "",]
ax.set_xticklabels(xticklabels, rotation = 90)
ax.tick_params(axis='x', labelsize=fontsize_tick)
y_locations = np.linspace(4.1, 4.5, 3)
ax.set_yticks(y_locations)
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
ax.tick_params(axis='y', labelsize=fontsize_tick)
        
# %% Stride lengths
ax = axs[1,4]    
color=iter(plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))))
for c_mtp, case in enumerate(cases_mtp):
    # print(optimaltrajectories[case]["COT"])
    # print(optimaltrajectories[case]["objective"])
    ax.scatter(c_mtp, optimaltrajectories[case]["stride_length_GC"], s=80, color=next(color))
# ax.set_title("Stride length")
ax.set_ylabel("Stride length (m)", fontsize=fontsize_label) 
x_locations = np.linspace(0, len(cases_mtp))
x_locations = np.linspace(0, len(cases_mtp)-1, len(cases_mtp))
ax.set_xticks(x_locations)
xticklabels = ["100%", "90%", "80%", "70%", "60%", "50%", "40%", "30%",]
ax.set_xticklabels(xticklabels, rotation = 90)
ax.tick_params(axis='x', labelsize=fontsize_tick)
y_locations = np.linspace(1.50, 1.70, 3)
ax.set_yticks(y_locations)
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
ax.tick_params(axis='y', labelsize=fontsize_tick)

# %%
for ax in (axs[2,:].flat):
    ax.set_visible(False)
    
# for ax in (axs[:,4].flat):
#     ax.set_visible(False)
    
for ax in axs.flat:
    ax.xaxis.get_label().set_fontsize(fontsize_label)
    ax.yaxis.get_label().set_fontsize(fontsize_label)
    ax.title.set_fontsize(fontsize_title)
    
fig.set_size_inches(16,12)
fig.tight_layout()




# ax.set_xticks(x_locations)
    # xticklabels = ["Case_" + case for case in cases]
    # ax1.set_xticklabels(xticklabels)
    # ax2.set_xticks(x_locations)
    # ax2.set_xticklabels(xticklabels)

# # %% Ground reaction forces
# GRFTitles = ['Fore-aft', 'Vertical', 'Lateral','Fore-aft', 'Vertical', 'Lateral']
# GRF_labels = optimaltrajectories[cases_mtp[0]]['GRF_labels']
# GRFToPlot = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r']
# NGRFToPlot = len(GRFToPlot)
# GRF_ylim_ub = [200,1000,100]
# GRF_ylim_lb = [-200,0,-100]
# NGRFToPlot = len(GRFToPlot)
# idxGRFToPlot = getJointIndices(GRF_labels, GRFToPlot)
# # fig, axs = plt.subplots(2, 3, sharex=True)    
# # fig.suptitle('Ground reaction forces')
# for i, ax in enumerate(axs[1,2:]):
#     if i < NGRFToPlot:
        
#         for c_no_mtp, case in enumerate(cases_no_mtp):
#             ax.plot(optimaltrajectories_no_mtp[case]['GC_percent'],
#                     optimaltrajectories_no_mtp[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=color_no_mtp[c_no_mtp], linestyle=linestyle_no_mtp[c_no_mtp], linewidth=linewidth_s, label='case_' + case + '_no_mtp') 
            
#         for c_mtp, case in enumerate(cases_mtp):
#             ax.plot(optimaltrajectories[case]['GC_percent'],
#                     optimaltrajectories[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=color_mtp[c_mtp], linestyle=linestyle_mtp[c_mtp], linewidth=linewidth_s, label='case_' + case + '_mtp')  
#             ax.fill_between(experimentalData_mtp[subject]["GRF"]["GC_percent"],
#                             experimentalData_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] + 2*experimentalData_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
#                             experimentalData_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] - 2*experimentalData_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
#                             facecolor='grey', alpha=0.4)
         
#             # ax.fill_between(experimentalData_no_mtp[subject]["GRF"]["GC_percent"],
#             #                 experimentalData_no_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] + 2*experimentalData_no_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
#             #                 experimentalData_no_mtp[subject]["GRF"]["mean"][GRFToPlot[i]] - 2*experimentalData_no_mtp[subject]["GRF"]["std"][GRFToPlot[i]],
#             #                 facecolor='black', alpha=0.4)
#         ax.set_title(GRFTitles[idxGRFToPlot[i]])
#         ax.set_ylim((GRF_ylim_lb[i],GRF_ylim_ub[i]))
#         ax.set_yticks([GRF_ylim_lb[i],0,GRF_ylim_ub[i]])
#         plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
#         ax.spines['right'].set_visible(False)
#         ax.spines['top'].set_visible(False)
#         ax.set_xticks([0,50,100]) 
#         plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
#         # handles, labels = ax.get_legend_handles_labels()
#         # plt.legend(handles, labels, loc='upper right')
# # plt.setp(axs[-1, :], xlabel='Gait cycle (%)')
# plt.setp(axs[1, 2], ylabel='(N)')
# fig.align_ylabels()

# for ax in axs.flat:
#     ax.xaxis.get_label().set_fontsize(fontsize_label)
#     ax.yaxis.get_label().set_fontsize(fontsize_label)
#     ax.title.set_fontsize(fontsize_title)


# # # %%
# for ax in (axs[2,:].flat):
#     ax.set_visible(False)
    
# fig.set_size_inches(16,12)
# fig.tight_layout()



# # # %% Metabolic cost and cost function value
# # fig, (ax1, ax2) = plt.subplots(1, 2)
# # color_mtp=plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))) 
# # for count, case in enumerate(cases_mtp):
# #     print(optimaltrajectories[case]["COT"])
# #     ax1.scatter(count, optimaltrajectories[case]["COT"], s=80, c=color_mtp[count, :].reshape(1,-1))
# #     ax2.scatter(count, optimaltrajectories[case]["objective"], s=80, c=color_mtp[count, :].reshape(1,-1))
# # color_no_mtp=plt.cm.rainbow(np.linspace(0,1,len(cases_mtp))) 
# # for count, case in enumerate(cases_no_mtp):
# #     print(optimaltrajectories_no_mtp[case]["COT"])
# #     ax1.scatter(count+len(cases_mtp), optimaltrajectories_no_mtp[case]["COT"], s=80, c=color_mtp[count, :].reshape(1,-1), marker="^")
# #     ax2.scatter(count+len(cases_mtp), optimaltrajectories_no_mtp[case]["objective"], s=80, c=color_no_mtp[count, :].reshape(1,-1), marker="^")      
# # ax1.set_title("Cost of Transport")
# # ax1.set_ylabel("(J/Kg/m)")    
# # ax2.set_title("Optimal cost value")
# # ax2.set_ylabel("()")
# # x_locations = np.linspace(0, len(cases_mtp)+len(cases_no_mtp)-1, len(cases_mtp)+len(cases_no_mtp))
# # ax1.set_xticks(x_locations)
# # xticklabels = ["Case_" + case + "_mtp" for case in cases_mtp] + ["Case_" + case + "_no_mtp" for case in cases_no_mtp]
# # ax1.set_xticklabels(xticklabels)
# # ax2.set_xticks(x_locations)
# # ax2.set_xticklabels(xticklabels)
