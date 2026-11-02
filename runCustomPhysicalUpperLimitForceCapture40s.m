%% CAPTURE OBSTACLE 3 UPPER-LIMIT FORCE (ful) ON THE EXISTING DIAGNOSTIC COPY
% This script operates ONLY on the separately saved diagnostic model copy:
%   OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339.slx
%
% It does NOT edit/save the original custom model or the six-obstacle scenario.
% It adds a PS-Simulink Converter and To Workspace block in memory only,
% connects the verified unconnected RConn2 port of Obstacle3SpawnJoint,
% runs a 40 s diagnostic with CaptureErrors='on', and saves the partial output.
% The diagnostic model is closed without saving at the end.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

sourceModelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
diagModelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
diagModelFile = fullfile(projectRoot,[diagModelName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot,'setupOffRoadProject.m');
testDuration = 40.0;

assert(isfile(diagModelFile),'Diagnostic model copy not found: %s',diagModelFile);
assert(isfile(scenarioFile),'Six-obstacle scenario not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    'Original custom model should remain loaded; this script will not load or alter it.');

sourceDirtyBefore = get_param(sourceModelName,'Dirty');
sourceFileBefore = get_param(sourceModelName,'FileName');
wasDiagLoaded = bdIsLoaded(diagModelName);
if ~wasDiagLoaded
    load_system(diagModelFile);
end

cleanupDiag = onCleanup(@() closeDiagnosticWithoutSaving(diagModelName,wasDiagLoaded)); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 3 UPPER-LIMIT FORCE CAPTURE — 40 s\n');
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original model dirty flag: %s\n',sourceDirtyBefore);

%% 1. Restore exactly the workspace/scenario inputs used by the earlier probe
fprintf('\n[1/5] Restoring project workspace and six-obstacle inputs...\n');
run(setupFile);
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from scenario MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Unexpected static/dynamic obstacle roles.');
assert(abs(double(obs(3).spawnTime)-7) < 1e-9 && ...
       abs(double(obs(5).spawnTime)-15) < 1e-9, ...
    'Expected obstacle 3 activation at 7 s and obstacle 5 at 15 s.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalXY = goalPoseWorkflow(1:2);
goalX = goalPoseWorkflow(1); %#ok<NASGU>
goalY = goalPoseWorkflow(2); %#ok<NASGU>
allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1,4)]; %#ok<NASGU>
obstacleYs = [allCenters(:,2).' nan(1,4)]; %#ok<NASGU>
obstacleSpawnTime = [[obs.spawnTime] nan(1,4)]; %#ok<NASGU>
obstacle3X = double(obs(3).centerXY(1)); %#ok<NASGU>
obstacle3Y = double(obs(3).centerXY(2)); %#ok<NASGU>
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);
staticObs = obs(~[obs.isDynamic]);
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded,initialWaypointCount,~,initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2),goalXY, ...
    staticCenters(:,1).',staticCenters(:,2).', ...
    obstacleClearance,maxWaypoints);
assert(initialWaypointCount >= 2,'Initial route calculation failed.');
waypoints = originalPathPadded(1:initialWaypointCount,:); %#ok<NASGU>
newPath = waypoints; %#ok<NASGU>
fprintf('Scenario: %s | obstacles: %s | initial route: %.3f m\n', ...
    scenarioConfig.name,mat2str([obs.id]),initialRouteLength);

%% 2. Verify ful sensing and the unconnected RConn2 port
fprintf('\n[2/5] Verifying Obstacle 3 upper-limit force output...\n');
jointPath = [diagModelName '/Obstacle3SpawnJoint'];
assert(strcmpi(get_param(jointPath,'SenseUpperLimitTorqueForce'),'on'), ...
    'Upper-limit force sensing is not enabled on the diagnostic copy.');

jointPH = get_param(jointPath,'PortHandles');
assert(numel(jointPH.RConn) >= 2, ...
    'Expected a second right-side physical port after enabling ful sensing.');
fulPort = jointPH.RConn(2);
fulLine = get_param(fulPort,'Line');
assert(fulLine == -1, ...
    'Obstacle 3 RConn2 is not free. Refusing to alter an existing connection.');
fprintf('SenseUpperLimitTorqueForce = on\n');
fprintf('Verified unconnected physical output: %s / RConn2 (ful candidate)\n',jointPath);

%% 3. Add converter and To Workspace blocks in memory only
fprintf('\n[3/5] Connecting ful to a Simulink timeseries logger...\n');
converterLib = 'nesl_utility/PS-Simulink Converter';
toWorkspaceLib = 'simulink/Sinks/To Workspace';
load_system('nesl_utility');
load_system('simulink');

converterPath = [diagModelName '/Probe_PS_Simulink_Obstacle3_ful'];
toWorkspacePath = [diagModelName '/Probe_ToWorkspace_Obstacle3_ful'];
assert(getSimulinkBlockHandle(converterPath) == -1, ...
    'Converter block already exists; refusing to create a duplicate.');
assert(getSimulinkBlockHandle(toWorkspacePath) == -1, ...
    'To Workspace block already exists; refusing to create a duplicate.');

add_block(converterLib,converterPath,'Position',[900 560 1035 600]);
add_block(toWorkspaceLib,toWorkspacePath, ...
    'Position',[1100 560 1225 600], ...
    'VariableName','obstacle3FulProbe', ...
    'SaveFormat','Timeseries');

converterPH = get_param(converterPath,'PortHandles');
if ~isempty(converterPH.LConn)
    psInputPort = converterPH.LConn(1);
elseif ~isempty(converterPH.RConn)
    psInputPort = converterPH.RConn(1);
else
    error('PS-Simulink Converter has no physical-signal input port.');
end
assert(~isempty(converterPH.Outport),'PS-Simulink Converter has no Simulink output.');
toWorkspacePH = get_param(toWorkspacePath,'PortHandles');
assert(~isempty(toWorkspacePH.Inport),'To Workspace block has no input port.');

add_line(diagModelName,fulPort,psInputPort,'autorouting','on');
add_line(diagModelName,converterPH.Outport(1),toWorkspacePH.Inport(1), ...
    'autorouting','on');

fprintf('Converter input connected to Obstacle3SpawnJoint RConn2.\n');
fprintf('Converter output connected to To Workspace: obstacle3FulProbe.\n');

%% 4. Compile and run the instrumented diagnostic copy only
fprintf('\n[4/5] Updating and running the 40 s diagnostic copy...\n');
set_param(diagModelName,'SimulationCommand','update');

stamp = datestr(now,'yyyymmdd_HHMMSS');
logName = ['simlog_UpperLimitFulCapture_' stamp];
matFile = fullfile(projectRoot, ...
    ['capturedObstacle3UpperLimitForceConnected40s_' stamp '.mat']);
csvFile = fullfile(projectRoot, ...
    ['obstacle3UpperLimitForceValues_' stamp '.csv']);

testIn = Simulink.SimulationInput(diagModelName);
testIn = testIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

probeOutput = sim(testIn);
assignin('base','probeOutputObstacle3Ful',probeOutput);

%% 5. Save returned partial output and extract the measured force
fprintf('\n[5/5] Saving output and extracting upper-limit force...\n');
returnedNames = probeOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));
try
    execInfo = probeOutput.SimulationMetadata.ExecutionInfo;
    fprintf('Stop event: %s | stop time: %.12f s\n', ...
        execInfo.StopEvent,execInfo.StopEventTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end
try
    err = char(probeOutput.ErrorMessage);
    if ~isempty(strtrim(err))
        fprintf('\nCaptured simulation error:\n%s\n',err);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end

save(matFile,'probeOutput','-v7.3');
fprintf('Saved diagnostic SimulationOutput: %s\n',matFile);

assert(any(strcmp(returnedNames,'obstacle3FulProbe')), ...
    ['The To Workspace signal obstacle3FulProbe was not returned. ' ...
     'Inspect probeOutput.who and the Simulink diagnostics.']);
forceTS = probeOutput.get('obstacle3FulProbe');
assert(isa(forceTS,'timeseries'), ...
    'Expected obstacle3FulProbe to be a timeseries; received %s.',class(forceTS));

forceTime = double(forceTS.Time(:));
forceData = double(forceTS.Data);
if size(forceData,1) ~= numel(forceTime) && size(forceData,2) == numel(forceTime)
    forceData = forceData.';
end
forceData = reshape(forceData,numel(forceTime),[]);
assert(size(forceData,2) == 1, ...
    'Expected scalar Z upper-limit force; received %d columns.',size(forceData,2));

forceTable = table(forceTime,forceData(:,1), ...
    'VariableNames',{'Time_s','UpperLimitForce_N'});
writetable(forceTable,csvFile);
fprintf('Force samples: %d | range: %.9g to %.9g N\n', ...
    numel(forceTime),min(forceData(:,1)),max(forceData(:,1)));
fprintf('Saved force CSV: %s\n',csvFile);

windows = [6.9 7.1; 14.7 15.1; 29.2 29.624814507405514];
labels = {'Obstacle 3 command window','Near obstacle 5 activation','Final failure window'};
for w = 1:size(windows,1)
    idx = find(forceTime >= windows(w,1) & forceTime <= windows(w,2));
    fprintf('\n--- %s [%.6f, %.6f] s ---\n',labels{w},windows(w,1),windows(w,2));
    if isempty(idx)
        fprintf('No samples.\n');
    else
        n = min(12,numel(idx));
        pick = unique([idx(1:min(6,numel(idx)));idx(max(1,numel(idx)-5):end)]);
        disp(forceTable(pick,:));
    end
end

fprintf('\nOriginal model dirty flag before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));
assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original model dirty flag changed unexpectedly.');

fprintf('\n============================================================\n');
fprintf(' UPPER-LIMIT FORCE CAPTURE FINISHED\n');
fprintf(' Original custom model and scenario were not saved or modified.\n');
fprintf(' Diagnostic copy changes were made in memory only.\n');
fprintf(' MAT: %s\nCSV: %s\n',matFile,csvFile);
fprintf('============================================================\n');

%% Local cleanup helper
function closeDiagnosticWithoutSaving(modelName,wasLoaded)
    if ~wasLoaded && bdIsLoaded(modelName)
        close_system(modelName,0);
    end
end
