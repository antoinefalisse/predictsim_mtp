%% OPENSIM PIPELINE

clear all
close all
clc

batchContact = 1;
batchKS = 0;
batchEL = 0;
batchID = 0;
batchRotateData = 0;
batchKSTransformed = 0;
batchEL_transformed = 0;
batchID_transformed = 1;
batchPointKinematics = 0;

subjects = {'subject1'};
folders = {'subject1_HamnerModel'};
% folders = {'subject1_FullBodyModel'};

subject = subjects{1};
folder = folders{1};

trials.subject1.names       = {'gait_14',   'gait_15',  'gait_23',  'gait_25',  'gait_27',  'gait_60',  'gait_61',  'gait_63',  'gait_64',  'gait_65'}; %,  'gait_67'};
% trials.subject1.timeStart   = [3.05,        2,          1.2,        0.95,       1.75,       1.1,        1.75,       0.8,        1,          0.9]; %,        1.1];
% trials.subject1.timeEnd     = [4.5,         3.4,        2.6,        2.4,        3.1,        2.45,       3,          2,          2.4,        2.3]; %,        2.6];
% Manual detection of end of half gait cycle with contact data (for tracking simulations)
trials.subject1.IC_L2       = [4.26,        NaN,        2.43,       2.21,       NaN,        2.26,       2.88,       NaN,        2.14,       NaN];
trials.subject1.IC_R2       = [NaN,         3.2,        NaN,        NaN,        2.93,       NaN,        NaN,        1.87,       NaN,        2.11];
% Manual detection of end of gait cycle (for reference data, can be that no contact data)
trials.subject1.IC_L3       = [NaN,     3.76,	NaN,	NaN,	3.45,	NaN,	NaN,	2.37,	NaN,	2.63];
trials.subject1.IC_R3       = [4.81,    NaN,	2.97,	2.75,	NaN,	2.78,	3.4,	NaN,	2.66,	NaN];

pathOpenSimModel = pwd;
% [pathOpenSimModel,~,~] = fileparts(pathmain);
% pathC3D = [pathOpenSimModel,'\',subject,'\C3D\'];
pathTRC = [pathOpenSimModel,'\TRC\'];

[pathRepo,~,~] = fileparts(pathOpenSimModel);
pathVariousFunctions = [pathRepo,'\VariousFunctions'];
addpath(genpath(pathVariousFunctions)); 

import org.opensim.modeling.*

%% Extract contacts
if batchContact
    rightID = '1_ground_force_vy';
    leftID = 'ground_force_vy';
    threshold = 20;
    for i = 1:length(subjects)
        for j = 1:length(trials.(subjects{i}).names)
            if strcmp(trials.(subjects{i}).names{j},'run_1')
                trials.(subjects{i}).timeStart(j) = 0.01;
                trials.(subjects{i}).timeEnd(j) = 11;
            else
                pathGRF = [pathOpenSimModel,'\GRF\GRF_',trials.(subjects{i}).names{j},'.mot'];
                GRF = importdata(pathGRF);
                GRF_L_vy = GRF.data(:,strcmp(leftID,strtrim(GRF.colheaders)));
                GRF_R_vy = GRF.data(:,strcmp(rightID,strtrim(GRF.colheaders)));            
                logic_L = GRF_L_vy >= threshold;
                logic_R = GRF_R_vy >= threshold;
                GRF_L_IC = find(diff(logic_L)==1) +1;
                GRF_R_IC = find(diff(logic_R)==1) +1;
                GRF_L_TO = (diff(logic_L)==-1);
                GRF_R_TO = (diff(logic_R)==-1);
                trials.(subjects{i}).IC_L(j) = GRF.data(GRF_L_IC,1);
                trials.(subjects{i}).IC_R(j) = GRF.data(GRF_R_IC,1);
                trials.(subjects{i}).TO_L(j) = GRF.data(GRF_L_TO,1);
                trials.(subjects{i}).TO_R(j) = GRF.data(GRF_R_TO,1); 
                % We want to select the side for which we have GRFs from both
                % sides.
                margin = 0.05; 
                if trials.(subjects{i}).IC_L(j) < trials.(subjects{i}).IC_R(j)
                    trials.(subjects{i}).sideToSelect{j} = 'R';
                    trials.(subjects{i}).halfCycleTimeStart(j) = round(trials.(subjects{i}).IC_R(j),2);
                    trials.(subjects{i}).halfCycleTimeEnd(j) = round(trials.subject1.IC_L2(j),2);       
                    trials.(subjects{i}).timeStart(j) = round(trials.(subjects{i}).IC_L(j),2) - margin;
                    trials.(subjects{i}).timeEnd(j) = trials.subject1.IC_R3(j) + margin;

                else
                    trials.(subjects{i}).sideToSelect{j} = 'L';
                    trials.(subjects{i}).halfCycleTimeStart(j) = round(trials.(subjects{i}).IC_L(j),2);
                    trials.(subjects{i}).halfCycleTimeEnd(j) = round(trials.subject1.IC_R2(j),2);
                    trials.(subjects{i}).timeStart(j) = round(trials.(subjects{i}).IC_R(j),2) - margin;
                    trials.(subjects{i}).timeEnd(j) = trials.subject1.IC_L3(j) + margin;
                end    
                initStartTime = trials.(subjects{i}).timeStart(j);
                % Check that the three first frames do not contain 0. Otherwise,
                % adjust start time.
                % First, by going down if possible
                TRCpath = [pathTRC,trials.(subjects{i}).names{j},'.trc'];            
                TRC = readTRC(fopen(TRCpath));
                idxStart = find(round(TRC.data(:,2),2) == round(trials.(subjects{i}).timeStart(j),2));
                idxStartInit = idxStart;
                frames = TRC.data(idxStart:idxStart+2,3:end);
                descent = 0;
                adjustement = 0;
                while ~isempty(find(~frames, 1)) && idxStart >= 1
                    adjustement = 1;
                    idxStart = idxStart - 1;
                    frames = TRC.data(idxStart:idxStart+2,3:end);
                    if idxStart == 1
                        descent = 1;
                        break
                    end
                end
                if adjustement == 1 && descent == 0
                    trials.(subjects{i}).timeStart(j) = TRC.data(idxStart,2);
                    disp(['Trial ' trials.(subjects{i}).names{j} ' adjusted down. It was ' num2str(initStartTime) '. It is now ' num2str(trials.(subjects{i}).timeStart(j))])
                end             

                % Otherwise, by going up until max IC - 0.01
                if descent
                    idxStart = idxStartInit;
                    frames = TRC.data(idxStart:idxStart+2,3:end);
                    while ~isempty(find(~frames, 1))
                        idxStart = idxStart + 1;
                        frames = TRC.data(idxStart:idxStart+2,3:end);
                        if TRC.data(idxStart,2) >= (trials.(subjects{i}).timeStart(j) + margin - 0.01)
                            error('TRC file is full of 0s')
                        end
                    end
                    trials.(subjects{i}).timeStart(j) = TRC.data(idxStart,2);
                    disp(['Trial ' trials.(subjects{i}).names{j} ' adjusted up. It was ' num2str(initStartTime) '. It is now ' num2str(trials.(subjects{i}).timeStart(j))])
                end   

%                 GRFall = getGRF(pathGRF);
%                 idxStartPlot = find(round(GRFall.time(:,1),3) == trials.(subjects{i}).halfCycleTimeStart(j));
%                 idxEndPlot = find(round(GRFall.time(:,1),3) == trials.(subjects{i}).halfCycleTimeEnd(j));
%                 N = 100;
%                 timeInterp = linspace(trials.(subjects{i}).halfCycleTimeStart(j), trials.(subjects{i}).halfCycleTimeEnd(j), N);
%                 for k = 1:3
%                     if trials.(subjects{i}).sideToSelect{j} == 'R'
%                         figure(1)
%                     else
%                         figure(2)
%                     end              
%                     subplot(3,6,k)
%                     plot(interp1(GRFall.time, GRFall.val.l(:,k), timeInterp))
%                     hold on
%                     subplot(3,6,k+3)
%                     plot(interp1(GRFall.time, GRFall.val.r(:,k), timeInterp))
%                     hold on
%                     subplot(3,6,k+6)
%                     plot(interp1(GRFall.time, GRFall.MorGF.l(:,k), timeInterp))
%                     hold on
%                     subplot(3,6,k+9)
%                     plot(interp1(GRFall.time, GRFall.MorGF.r(:,k), timeInterp))                
%                     hold on
%                     subplot(3,6,k+12)
%                     plot(interp1(GRFall.time, GRFall.pos.l(:,k), timeInterp))
%                     hold on
%                     subplot(3,6,k+15)
%                     plot(interp1(GRFall.time, GRFall.pos.r(:,k), timeInterp))                
%                     hold on
%                 end  
            end
        end
    end
end
% Check that IC1 is before IC2 if not NaN
% [trials.subject1.IC_R;trials.subject1.IC_R2]
% [trials.subject1.IC_L;trials.subject1.IC_L2]
% trials.subject1.timeStart   = [3.05,        2,          1.2,        0.95,       1.75,       1.1,        1.75,       0.8,        1,          0.9]; %,        1.1];
% trials.subject1.timeEnd     = [4.5,         3.4,        2.6,        2.4,        3.1,        2.45,       3,          2,          2.4,        2.3]; %,        2.6];

%% Batch KS
if batchKS
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        temp_results = [];
        count = 1;
        for j = length(trials.(subjects{i}).names)            
            % Generic IK files
            genericSetupForIK = 'Setup_KS_generic.xml';
            genericSetupPath = [pathOpenSimModel,'\',folders{1},'\IK\'];
            ikTool = InverseKinematicsTool([genericSetupPath genericSetupForIK]);
            % Set model
            ikTool.setModel(model);
            % TRC
            TRCpath = [pathTRC,trials.(subjects{i}).names{j},'.trc'];
            ikTool.setMarkerDataFileName(TRCpath);
            ikTool.setName(trials.(subjects{i}).names{j});
            % Time
            ikTool.setStartTime(trials.(subjects{i}).timeStart(j));
            ikTool.setEndTime(trials.(subjects{i}).timeEnd(j))
            % Path results
            results_IK = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'.mot'];
            ikTool.setOutputMotionFileName(results_IK);
            % Save setup file
            PathSetupIK = [genericSetupPath,'Setup_KS_',trials.(subjects{i}).names{j},'.xml'];
            ikTool.print(PathSetupIK);         
            % Run IK
            Command = ['KS' ' -S ' PathSetupIK];
            system(Command);
            
            % Evaluate marker errors
%             trials.(subjects{i}).markerErrors{j} = importdata([genericSetupPath, trials.(subjects{i}).names{j}, '_ks_marker_errors.sto']);
%             trials.(subjects{i}).markerErrors{j}.meanRMS = mean(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_RMS')));
%             trials.(subjects{i}).markerErrors{j}.stdRMS = std(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_RMS')));     
%             trials.(subjects{i}).markerErrors{j}.max = max(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_max')));
            
%             temp_results = [temp_results, [trials.(subjects{i}).markerErrors{j}.meanRMS; trials.(subjects{i}).markerErrors{j}.stdRMS; trials.(subjects{i}).markerErrors{j}.max]];

            trials.(subjects{i}).KS = importdata([genericSetupPath, 'KS_' trials.(subjects{i}).names{j}, '.mot']);
            bounds.max(count,:) = max(trials.(subjects{i}).KS.data);
            bounds.min(count,:) = min(trials.(subjects{i}).KS.data); 
            count = count + 1; 
        end
    end
end

% bounds.maxmax = max(bounds.max,[],1);
% % bounds.minmin = min(bounds.min,[],1);
% bounds.colheaders = trials.(subjects{i}).KS.colheaders;
% 
% model_3D.colheaders = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz','hip_flexion_l','hip_flexion_r',...
%     'hip_adduction_l','hip_adduction_r','hip_rotation_l','hip_rotation_r',...
%     'knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r','subtalar_angle_l','subtalar_angle_r',...
%     'mtp_angle_l','mtp_angle_r','lumbar_extension','lumbar_bending','lumbar_rotation'};
% for i = 1:length(model_3D.colheaders)
%     model_3D.max(i) = bounds.maxmax(strcmp(bounds.colheaders,model_3D.colheaders{i}));
%     model_3D.min(i) = bounds.minmin(strcmp(bounds.colheaders,model_3D.colheaders{i}));
% end
a = -45; b = 30; r = [];
r1 = (b-a).*rand(5000,1) + a;

%% Batch external Loads
if batchEL
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}).names)
            % Generic ExternalLoads files
            genericSetupForEL = 'Setup_EL_generic.xml';
            genericSetupPath = [pathOpenSimModel,'\',folders{1},'\ExternalLoads\'];
            ELTool = ExternalLoads([genericSetupPath genericSetupForEL], true);
            pathGRF = [pathOpenSimModel,'\GRF\GRF_',trials.(subjects{i}).names{j},'.mot'];
            ELTool.setDataFileName(pathGRF);
            results_IK = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'.mot'];
            ELTool.setExternalLoadsModelKinematicsFileName(results_IK);
            ELTool.setName(trials.(subjects{i}).names{j});
            ELTool.setLowpassCutoffFrequencyForLoadKinematics(-1); % no need to filter with KS
            SetupForEL = ['Setup_EL_',trials.(subjects{i}).names{j},'.xml'];
            ELTool.print([genericSetupPath,SetupForEL]); 
        end
    end
end

%% Batch ID
if batchID
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}).names)
            results_ID = [pathOpenSimModel,'\',folders{1},'\ID\'];
            % Generic ID files
            genericSetupForID = 'Setup_ID_generic.xml';
            genericSetupPath = results_ID;
            idTool = InverseDynamicsTool([genericSetupPath genericSetupForID]);
            % Set model
            idTool.setModel(model);
            idTool.setModelFileName([modelFilePath modelFile]);
            idTool.setName([trials.(subjects{i}).names{j}]);
            % time
            idTool.setStartTime(trials.(subjects{i}).timeStart(j));
            idTool.setEndTime(trials.(subjects{i}).timeEnd(j))
            % External Loads and IK
            genericSetupPathForEL = [pathOpenSimModel,'\',folders{1},'\ExternalLoads\'];
            SetupForEL = ['Setup_EL_',trials.(subjects{i}).names{j},'.xml'];
            ExternalLoadspath = [genericSetupPathForEL,SetupForEL];
            idTool.setExternalLoadsFileName(ExternalLoadspath)
            IKpath = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'.mot'];
            idTool.setCoordinatesFileName(IKpath)
            % Cutoff frequency
            idTool.setLowpassCutoffFrequency(-1); % no need to filter with KS
            % Where to save results
            idTool.setResultsDir(results_ID);
            idTool.setOutputGenForceFileName(['ID_',trials.(subjects{i}).names{j},'.sto']);
            % Save setup file
            PathSetupID = [genericSetupPath,'Setup_ID_',trials.(subjects{i}).names{j},'.xml'];        
            idTool.print(PathSetupID);         
            % Run ID
            Command = ['opensim-cmd run-tool ' PathSetupID];
            system(Command);
        end
    end
end

%% Rotate data
if batchRotateData
    for i = 1:length(subjects)
        for j = 1:length(trials.(subjects{i}).names)
            IKpath = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'.mot'];
            IKdata = importdata(IKpath);            
            alpha = atand((IKdata.data(end,strcmp(IKdata.colheaders,'pelvis_tx')) - IKdata.data(1,strcmp(IKdata.colheaders,'pelvis_tx'))) / ...
                (IKdata.data(end,strcmp(IKdata.colheaders,'pelvis_tz')) - IKdata.data(1,strcmp(IKdata.colheaders,'pelvis_tz'))));
            if alpha >= 0
                alphaRot(j) = 90-alpha;
                alphaSense{j} = '+';
            else
                alphaRot(j) = 90+alpha;
                alphaSense{j} = '-';
            end
        end
    end
end

%% Batch KS transformed
if batchKSTransformed
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        temp_results = [];
        for j = 1:length(trials.(subjects{i}).names)            
            % Generic IK files
            genericSetupForIK = 'Setup_KS_generic_transformed.xml'; % slightly adjusted cause one motion was bad
            genericSetupPath = [pathOpenSimModel,'\',folders{1},'\IK\'];
            ikTool = InverseKinematicsTool([genericSetupPath genericSetupForIK]);
            % Set model
            ikTool.setModel(model);
            % TRC
            TRCpath = [pathTRC,trials.(subjects{i}).names{j},'_tr.trc'];
            ikTool.setMarkerDataFileName(TRCpath);
            ikTool.setName(trials.(subjects{i}).names{j});
            % Time
            ikTool.setStartTime(trials.(subjects{i}).timeStart(j));
            ikTool.setEndTime(trials.(subjects{i}).timeEnd(j))
            % Path results
            results_IK = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'_transformed.mot'];
            ikTool.setOutputMotionFileName(results_IK);
            % Save setup file
            PathSetupIK = [genericSetupPath,'Setup_KS_',trials.(subjects{i}).names{j},'_transformed.xml'];
            ikTool.print(PathSetupIK);         
            % Run IK
            Command = ['KS' ' -S ' PathSetupIK];
%             system(Command);
            
            % Evaluate marker errors
            trials.(subjects{i}).markerErrors{j} = importdata([genericSetupPath, trials.(subjects{i}).names{j}, '_ks_marker_errors.sto']);
            trials.(subjects{i}).markerErrors{j}.meanRMS = mean(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_RMS')));
            trials.(subjects{i}).markerErrors{j}.stdRMS = std(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_RMS')));     
            trials.(subjects{i}).markerErrors{j}.max = max(trials.subject1.markerErrors{j}.data(:,strcmp(trials.subject1.markerErrors{j}.colheaders,'marker_error_max')));
            
            temp_results = [temp_results, [trials.(subjects{i}).markerErrors{j}.meanRMS; trials.(subjects{i}).markerErrors{j}.stdRMS; trials.(subjects{i}).markerErrors{j}.max]];
            
            trials.(subjects{i}).KS = importdata([genericSetupPath, 'KS_' trials.(subjects{i}).names{j}, '_transformed.mot']);
            bounds.max(j,:) = max(trials.(subjects{i}).KS.data);
            bounds.min(j,:) = min(trials.(subjects{i}).KS.data);            
        end
    end
end

% bounds.maxmax = max(bounds.max);
% bounds.minmin = min(bounds.min);
% bounds.colheaders = trials.(subjects{i}).KS.colheaders;
% 
% model_2D.colheaders = {'pelvis_tilt','pelvis_tx','pelvis_ty','hip_flexion_l','hip_flexion_r','knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r','mtp_angle_l','mtp_angle_r','lumbar_extension'};
% for i = 1:length(model_2D.colheaders)
%     model_2D.max(i) = bounds.maxmax(strcmp(bounds.colheaders,model_2D.colheaders{i}));
%     model_2D.min(i) = bounds.minmin(strcmp(bounds.colheaders,model_2D.colheaders{i}));
% end

%% Batch external Loads transformed
if batchEL_transformed
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}).names)
            % Generic ExternalLoads files
            genericSetupForEL = 'Setup_EL_generic.xml';
            genericSetupPath = [pathOpenSimModel,'\',folders{1},'\ExternalLoads\'];
            ELTool = ExternalLoads([genericSetupPath genericSetupForEL], true);
            pathGRF = [pathOpenSimModel,'\GRF\GRF_',trials.(subjects{i}).names{j},'_transformed.mot'];
            ELTool.setDataFileName(pathGRF);
            results_IK = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'_transformed.mot'];
            ELTool.setExternalLoadsModelKinematicsFileName(results_IK);
            ELTool.setName(trials.(subjects{i}).names{j});
            ELTool.setLowpassCutoffFrequencyForLoadKinematics(-1); % no need to filter with KS
            SetupForEL = ['Setup_EL_',trials.(subjects{i}).names{j},'_transformed.xml'];
            ELTool.print([genericSetupPath,SetupForEL]); 
        end
    end
end

%% Batch ID transformed
if batchID_transformed
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',folders{1},'\Model\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}).names)
            results_ID = [pathOpenSimModel,'\',folders{1},'\ID\'];
            % Generic ID files
            genericSetupForID = 'Setup_ID_generic.xml';
            genericSetupPath = results_ID;
            idTool = InverseDynamicsTool([genericSetupPath genericSetupForID]);
            % Set model
            idTool.setModel(model);
            idTool.setModelFileName([modelFilePath modelFile]);
            idTool.setName([trials.(subjects{i}).names{j}]);
            % time
            idTool.setStartTime(trials.(subjects{i}).timeStart(j));
            idTool.setEndTime(trials.(subjects{i}).timeEnd(j))
            % External Loads and IK
            genericSetupPathForEL = [pathOpenSimModel,'\',folders{1},'\ExternalLoads\'];
            SetupForEL = ['Setup_EL_',trials.(subjects{i}).names{j},'_transformed.xml'];
            ExternalLoadspath = [genericSetupPathForEL,SetupForEL];
            idTool.setExternalLoadsFileName(ExternalLoadspath)
            IKpath = [pathOpenSimModel,'\',folders{1},'\IK\KS_',trials.(subjects{i}).names{j},'_transformed.mot'];
            idTool.setCoordinatesFileName(IKpath)
            % Cutoff frequency
            idTool.setLowpassCutoffFrequency(-1); % no need to filter with KS
            % Where to save results
            idTool.setResultsDir(results_ID);
            idTool.setOutputGenForceFileName(['ID_',trials.(subjects{i}).names{j},'_transformed.sto']);
            % Save setup file
            PathSetupID = [genericSetupPath,'Setup_ID_',trials.(subjects{i}).names{j},'_transformed.xml'];        
            idTool.print(PathSetupID);         
            % Run ID
            Command = ['opensim-cmd run-tool ' PathSetupID];
            system(Command);
        end
    end
end

%% Point kinematics
if batchPointKinematics
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathOpenSimModel,'\',subject,'\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}).names)
            results_PK = [pathOpenSimModel,'\',subject,'\PointKinematics\'];
            % Get original setup PK 
            genericSetupForPK = 'SetupPointKinematics.xml';
            genericSetupPath = results_PK;
            % Initiate Analysis tool
            PKTool = AnalyzeTool([genericSetupPath genericSetupForPK]);
            % Append model            
            PKTool.setModel(model);
            PKTool.setModelFilename([modelFilePath modelFile]);
            % Change name
            PKTool.setName([trials.(subjects{i}).names{j}]);
            % time
            PKTool.setStartTime(timeIN.(subjects{i})(j));
            PKTool.setFinalTime(timeOUT.(subjects{i})(j))     
            % Set results dir
            PKTool.setResultsDir(results_PK);
            % Get IK
            IKpath = [pathOpenSimModel,'\',subject,'\IK\IK_',trials.(subjects{i}).names{j},'.mot'];
            PKTool.setCoordinatesFileName(IKpath);
            % Print new setup
            PathSetupPK = [results_PK,'Setup_PK_',trials.(subjects{i}).names{j},'.xml'];
            PKTool.print(PathSetupPK); 
            
            % Further editing
            PKSetup = xml_read(PathSetupPK);
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','start_time',timeIN.(subjects{i})(j));
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','end_time',timeOUT.(subjects{i})(j));
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','body_name','ground');
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','relative_to_body_name','calcn_r');
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','point_name','POA');
            PKSetup = setfield(PKSetup,'AnalyzeTool','AnalysisSet','objects',...
                'PointKinematics','point',[0 0 0]);  
            xml_writeOSIM(PathSetupPK,PKSetup,'OpenSimDocument');          
            
            % Run PK
            Command = ['Analyze' ' -S ' PathSetupPK];
            system(Command);
        end
    end    
end