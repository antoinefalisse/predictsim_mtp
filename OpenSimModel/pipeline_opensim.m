%% OPENSIM PIPELINE

clear all
close all
clc

batchIK = 0;
batchEL = 0;
batchID = 0;
batchPointKinematics = 0;
batchAnalyzeIKError = 1;

subjects = {'subject1'};
models = {'no_mtp'};

subject_name = subjects{1};
model_name = [subjects{1}, '_', models{1}];
trials.subject1 = {'gait_58','gait_59','gait_61','gait_63','gait_64','gait_67',...
    'gait_52','gait_53','gait_54','gait_14','gait_15','gait_23','gait_25','gait_27','gait_60','gait_65','run_1'};
timeIN.subject1 = [1.35, 1.55, 1.75, 0.8, 1, 1.25, 1.82, 0.8, 0.5, 3.05, 2, 1.2, 0.95, 1.75, 1.1, 0.9, 0.01]; 
timeOUT.subject1 = [2.7, 2.85, 3, 2, 2.4, 2.6, 3.15, 2.1, 1.8, 4.5, 3.4, 2.6, 2.4, 3.1, 2.45, 2.3, 11 ];

seltrials = [3,4,5,10,11,12,13,14,15,16];
trials.subject1 = trials.subject1(seltrials);
timeIN.subject1 = timeIN.subject1(seltrials);
timeOUT.subject1 = timeOUT.subject1(seltrials);


pathmain = pwd;
pathTRC = [pathmain,'\TRC\'];
pathGRF = [pathmain,'\GRF\'];

import org.opensim.modeling.*

%% Batch IK:
if batchIK
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathmain,'\',model_name,'\Model\'];
        modelFile = [model_name,'.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}))
            results_IK = [pathmain,'\',model_name,'\IK\IK_',trials.(subjects{i}){j},'.mot'];
            % Generic IK files
            genericSetupForIK = 'SetupKS_generic.xml';
            genericSetupPath = [pathmain,'\',model_name,'\IK\'];
            ikTool = InverseKinematicsTool([genericSetupPath genericSetupForIK]);
            % Set model
            ikTool.setModel(model);
            % TRC
            TRCpath = [pathTRC,trials.(subjects{i}){j},'.trc'];
            ikTool.setMarkerDataFileName(TRCpath);
            ikTool.setName(trials.(subjects{i}){j});
            % Time
            ikTool.setStartTime(timeIN.(subjects{i})(j));
            ikTool.setEndTime(timeOUT.(subjects{i})(j))
            % Where to save results
            ikTool.setOutputMotionFileName(results_IK);
            % Save setup file
            PathSetupIK = [genericSetupPath,'SetupKS_',trials.(subjects{i}){j},'_mtp.xml'];
            ikTool.print(PathSetupIK);         
            % Run IK
            Command = ['KS' ' -S ' PathSetupIK];
            system(Command);
        end
    end
end

%% Batch external Loads
if batchEL
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathmain,'\',model_name,'\Model\'];
        modelFile = [model_name,'.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}))
            % Generic ExternalLoads files
            genericSetupForEL = 'ExternalLoads_generic.xml';
            genericSetupPath = [pathmain,'\',model_name,'\ExternalLoads\'];
            ELTool = ExternalLoads([genericSetupPath genericSetupForEL], 1);
            ELTool.setDataFileName([pathGRF,'GRF_',trials.(subjects{i}){j},'.mot']);
            results_IK = [pathmain,'\',model_name,'\IK\IK_',trials.(subjects{i}){j},'.mot'];
            ELTool.setExternalLoadsModelKinematicsFileName(results_IK);
            ELTool.setName(trials.(subjects{i}){j});
            ELTool.setLowpassCutoffFrequencyForLoadKinematics(-1);
            SetupForEL = ['ExternalLoads_',trials.(subjects{i}){j},'.xml'];
            ELTool.print([genericSetupPath,SetupForEL]); 
        end
    end
end

%% Batch ID
if batchID
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathmain,'\',model_name,'\Model\'];
        modelFile = [model_name,'.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}))
            results_ID = [pathmain,'\',model_name,'\ID\'];
            % Generic ID files
            genericSetupForID = 'SetupID_generic.xml';
            genericSetupPath = results_ID;
            idTool = InverseDynamicsTool([genericSetupPath genericSetupForID]);
            % Set model
            idTool.setModel(model);
            idTool.setModelFileName([modelFilePath modelFile]);
            idTool.setName([trials.(subjects{i}){j}]);
            % time
            idTool.setStartTime(timeIN.(subjects{i})(j));
            idTool.setEndTime(timeOUT.(subjects{i})(j))        
            % External Loads and IK
            genericSetupPathForEL = [pathmain,'\',model_name,'\ExternalLoads\'];
            SetupForEL = ['ExternalLoads_',trials.(subjects{i}){j},'.xml'];
            ExternalLoadspath = [genericSetupPathForEL,SetupForEL];
            idTool.setExternalLoadsFileName(ExternalLoadspath)
            IKpath = [pathmain,'\',model_name,'\IK\IK_',trials.(subjects{i}){j},'.mot'];
            idTool.setCoordinatesFileName(IKpath)
            % Cutoff frequency
            idTool.setLowpassCutoffFrequency(-1);
            % Where to save results
            idTool.setResultsDir(results_ID);
            idTool.setOutputGenForceFileName(['ID_',trials.(subjects{i}){j},'.sto']);
            % Save setup file
            PathSetupID = [genericSetupPath,'SetupID_',trials.(subjects{i}){j},'.xml'];        
            idTool.print(PathSetupID);         
            % Run ID
            Command = ['opensim-cmd' ' run-tool ' PathSetupID];
            system(Command);
        end
    end
end

%% Batch analyze IK error:
if batchAnalyzeIKError
    analyzeIKError = zeros(length(trials.(subjects{i})), 4, length(subjects));
    for i = 1:length(subjects)
        for j = 1:length(trials.(subjects{i}))
            pathIKError = [pathmain,'\',model_name,'\IK\',trials.(subjects{i}){j},'_ks_marker_errors.sto'];
            IKError = importdata(pathIKError);
            mean_RMS = mean(IKError.data(:, strcmp(IKError.colheaders, 'marker_error_RMS')));
            max_RMS = max(IKError.data(:, strcmp(IKError.colheaders, 'marker_error_RMS')));            
            mean_max = mean(IKError.data(:, strcmp(IKError.colheaders, 'marker_error_max')));
            max_max = max(IKError.data(:, strcmp(IKError.colheaders, 'marker_error_max')));
            
            analyzeIKError(j,:,i) = [mean_RMS, max_RMS, mean_max, max_max];
            
            
        end
    end
end

%% Point kinematics
if batchPointKinematics
    for i = 1:length(subjects)
        % Model
        modelFilePath = [pathmain,'\',model_name,'\'];
        modelFile = [subjects{i},'_scaled.osim'];
        model = Model([modelFilePath,modelFile]);
        model.initSystem();
        for j = 1:length(trials.(subjects{i}))
            results_PK = [pathmain,'\',model_name,'\PointKinematics\'];
            % Get original setup PK 
            genericSetupForPK = 'SetupPointKinematics.xml';
            genericSetupPath = results_PK;
            % Initiate Analysis tool
            PKTool = AnalyzeTool([genericSetupPath genericSetupForPK]);
            % Append model            
            PKTool.setModel(model);
            PKTool.setModelFilename([modelFilePath modelFile]);
            % Change name
            PKTool.setName([trials.(subjects{i}){j}]);
            % time
            PKTool.setStartTime(timeIN.(subjects{i})(j));
            PKTool.setFinalTime(timeOUT.(subjects{i})(j))     
            % Set results dir
            PKTool.setResultsDir(results_PK);
            % Get IK
            IKpath = [pathmain,'\',model_name,'\IK\IK_',trials.(subjects{i}){j},'.mot'];
            PKTool.setCoordinatesFileName(IKpath);
            % Print new setup
            PathSetupPK = [results_PK,'Setup_PK_',trials.(subjects{i}){j},'.xml'];
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