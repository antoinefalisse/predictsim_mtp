import os
import numpy as np
import matplotlib.pyplot as plt  

from utilities import getJointIndices

# %% Settings
cases = ['4', '38', '40', '34', '37']
case_4exp = '4'

colors=['black', '#984ea3','#4daf4a','#377eb8','#ff7f00'] 
linestyles=['solid','dashed','dashdot','solid','dashdot']
linewidth_s = 3
fontsize_tick = 14
fontsize_label = 15
fontsize_title = 17

# '984ea3' : purple
# '4daf4a' : green
# '377eb8' : blue
# 'ff7f00' : orange

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
swing = 65
    
# %% Visualize results
plt.close('all')

# %% Joint coordinates
kinematic_ylim_ub = [20, 30,]
kinematic_ylim_lb = [-80, -30]
jointsToPlot = ['knee_angle_r',  'ankle_angle_r']
jointsToPlotTitle = ['Knee',  'Ankle']

NJointsToPlot = len(jointsToPlot)  
fig, axs = plt.subplots(3, 5, sharex=False)
count = 0
for i, ax in enumerate(axs[0, :]):
    plotExperimentalData = True     
    if i < NJointsToPlot:
        for c, case in enumerate(cases):            
            c_joints = optimaltrajectories[case]['joints']
            c_joint_idx = c_joints.index(jointsToPlot[i])            
            # Simulated data
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['coordinate_values'][c_joint_idx:c_joint_idx+1, :].T, linewidth=linewidth_s, color=colors[c], linestyle=linestyles[c])
            if plotExperimentalData:
                # Mean measured data
                ax.plot(experimentalData[subject]["kinematics"]["positions"]["GC_percent"],
                        experimentalData[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]], linewidth=linewidth_s, color='grey')
                # Mean +/- 2 standard deviation
                ax.fill_between(experimentalData[subject]["kinematics"]["positions"]["GC_percent"],
                    experimentalData[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] + 2*experimentalData[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                    experimentalData[subject]["kinematics"]["positions"]["mean"][jointsToPlot[i]] - 2*experimentalData[subject]["kinematics"]["positions"]["std"][jointsToPlot[i]],
                    facecolor='grey', alpha=0.4)
                plotExperimentalData = False  
        ax.vlines(swing, kinematic_ylim_lb[i], kinematic_ylim_ub[i], color='k')              
        ax.set_title(jointsToPlotTitle[i])
        ax.set_ylim((kinematic_ylim_lb[i],kinematic_ylim_ub[i]))
        ax.set_yticks([kinematic_ylim_lb[i],0,kinematic_ylim_ub[i]])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.set_xticks([0,50,100]) 
        ax.set_xticklabels([]) 
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        handles, labels = ax.get_legend_handles_labels()
        plt.legend(handles, labels, loc='upper right')
plt.setp(axs[1, :], xlabel='Gait cycle (%)')
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
               'vas_int_r', 'Vastus lateralis', 'Gastrocnemius medialis', 'lat_gas_r', 'Soleus',
               'tib_post_r', 'flex_dig_r', 'flex_hal_r', 'Tibialis anterior', 'per_brev_r',
               'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r', 'ercspn_r', 
               'intobl_r', 'extobl_r']
muscles = optimaltrajectories[cases[0]]['muscles']
musclesToPlot = ['vas_med_r','med_gas_r', 'soleus_r']
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
for i, ax in enumerate(axs[0:1,2:].flat):
    if i < NMusclesToPlot:
        for c, case in enumerate(cases):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i]:idxMusclesToPlot[i]+1, :].T, c=colors[c], linestyle=linestyles[c], linewidth=linewidth_s, label='case_' + case + '_no_mtp')
            
            if musclesToPlot[i] in mappingEMG and case == case_4exp:                
                # Normalize EMG such that peak mean EMG = peak activation             
                exp_mean = experimentalData[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]]
                exp_mean_peak = np.max(exp_mean)
                sim = optimaltrajectories[case]['muscle_activations'][idxMusclesToPlot[i], :].T
                sim_peak = np.max(sim)
                scaling_emg = sim_peak / exp_mean_peak
                ax.fill_between(experimentalData[subject]["EMG"]["GC_percent"],
                        experimentalData[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg + 2*experimentalData[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        experimentalData[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg - 2*experimentalData[subject]["EMG"]["std"][mappingEMG[musclesToPlot[i]]] * scaling_emg,
                        facecolor='grey', alpha=0.4)
                ax.plot(experimentalData[subject]["EMG"]["GC_percent"],
                        experimentalData[subject]["EMG"]["mean"][mappingEMG[musclesToPlot[i]]] * scaling_emg, color='grey', linewidth=linewidth_s)
        ax.set_title(muscleTitles[idxMusclesToPlot[i]])
        ax.vlines(swing, 0, 1, color='k')  
        ax.set_ylim((0,1))
        ax.set_yticks([0,1])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks([0,50,100])
        if i < NMusclesToPlot and i > 5:
            ax.set_xticks([0,50,100])
            ax.set_xticklabels(['0','50','100'])
            ax.set_xlabel('Gait cycle (%)')
        else:
            ax.set_xticks([0,50,100]) 
            ax.set_xticklabels([])
plt.setp(axs[0:1, 2], ylabel='Muscle activation (-)')
fig.align_ylabels()

# %% Kinetics
kinetic_ylim_ub = [70,30]
kinetic_ylim_lb = [-50,-110]
count = 0 
for i, ax in enumerate(axs[1,:]):
    plotExperimentalData = True
    if i < NJointsToPlot:
        for c, case in enumerate(cases):
            c_joints = optimaltrajectories[case]['joints']
            c_joint_idx = c_joints.index(jointsToPlot[i]) 
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['joint_torques'][c_joint_idx:c_joint_idx+1, :].T, c=colors[c], linestyle=linestyles[c], linewidth=linewidth_s, label='case_' + case + '_no_mtp')
            if plotExperimentalData:                
                ax.plot(experimentalData[subject]["kinetics"]["GC_percent"],
                        experimentalData[subject]["kinetics"]["mean"][jointsToPlot[i]], linewidth=linewidth_s, color='grey')                
                ax.fill_between(experimentalData[subject]["kinetics"]["GC_percent"],
                                experimentalData[subject]["kinetics"]["mean"][jointsToPlot[i]] + 2*experimentalData[subject]["kinetics"]["std"][jointsToPlot[i]],
                                experimentalData[subject]["kinetics"]["mean"][jointsToPlot[i]] - 2*experimentalData[subject]["kinetics"]["std"][jointsToPlot[i]],
                                facecolor='grey', alpha=0.4)
                plotExperimentalData = False    
        ax.vlines(swing, kinetic_ylim_lb[i], kinetic_ylim_ub[i], color='k')  
        ax.set_ylim((kinetic_ylim_lb[i],kinetic_ylim_ub[i]))
        ax.set_yticks([kinetic_ylim_lb[i],0,kinetic_ylim_ub[i]])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks([0,50,100])
        plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
plt.setp(axs[1, 0], ylabel='Joint torque (Nm)')
fig.align_ylabels()

# %% Ground reaction forces
GRFTitles = ['Fore-aft', 'Vertical', 'Lateral','Fore-aft', 'Vertical', 'Lateral']
GRF_labels = optimaltrajectories[cases[0]]['GRF_labels']
GRFToPlot = ['GRF_x_r', 'GRF_y_r', 'GRF_z_r']
NGRFToPlot = len(GRFToPlot)
GRF_ylim_ub = [200,1000,100]
GRF_ylim_lb = [-200,0,-100]
NGRFToPlot = len(GRFToPlot)
idxGRFToPlot = getJointIndices(GRF_labels, GRFToPlot)
for i, ax in enumerate(axs[1,2:]):
    if i < NGRFToPlot:        
        for c, case in enumerate(cases):
            ax.plot(optimaltrajectories[case]['GC_percent'],
                    optimaltrajectories[case]['GRF'][idxGRFToPlot[i]:idxGRFToPlot[i]+1, :].T, c=colors[c], linestyle=linestyles[c], linewidth=linewidth_s)
            if case == case_4exp:
                ax.fill_between(experimentalData[subject]["GRF"]["GC_percent"],
                                experimentalData[subject]["GRF"]["mean"][GRFToPlot[i]] + 2*experimentalData[subject]["GRF"]["std"][GRFToPlot[i]],
                                experimentalData[subject]["GRF"]["mean"][GRFToPlot[i]] - 2*experimentalData[subject]["GRF"]["std"][GRFToPlot[i]],
                                facecolor='grey', alpha=0.4)                
                ax.plot(experimentalData[subject]["GRF"]["GC_percent"],
                        experimentalData[subject]["GRF"]["mean"][GRFToPlot[i]], color='grey', linewidth=linewidth_s)
        ax.vlines(swing, GRF_ylim_lb[i], GRF_ylim_ub[i], color='k')  
        ax.set_title(GRFTitles[idxGRFToPlot[i]])
        ax.set_ylim((GRF_ylim_lb[i],GRF_ylim_ub[i]))
        ax.set_yticks([GRF_ylim_lb[i],0,GRF_ylim_ub[i]])
        plt.setp(ax.get_yticklabels(), fontsize=fontsize_tick)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks([0,50,100]) 
        plt.setp(ax.get_xticklabels(), fontsize=fontsize_tick)
        handles, labels = ax.get_legend_handles_labels()
plt.setp(axs[1, 2], ylabel='Ground reaction force (N)')
fig.align_ylabels()

for ax in axs.flat:
    ax.xaxis.get_label().set_fontsize(fontsize_label)
    ax.yaxis.get_label().set_fontsize(fontsize_label)
    ax.title.set_fontsize(fontsize_title)

# %%
for ax in (axs[2,:].flat):
    ax.set_visible(False)
    
fig.set_size_inches(16,12)
fig.tight_layout()

