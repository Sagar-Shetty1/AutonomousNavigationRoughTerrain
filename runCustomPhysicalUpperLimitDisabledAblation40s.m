%% A/B DIAGNOSTIC: DISABLE ONLY OBSTACLE 3 UPPER STOP IN MEMORY
% Operates only on the separate UpperStopProbe diagnostic model.
% The original Custom model and six-obstacle scenario are not modified.
% The diagnostic model is closed WITHOUT saving after the run, so its
% on-disk UpperStopProbe settings remain unchanged.
%
% Run from the project root or place this script in the project root:
%   runCustomPhysicalUpperLimitDisabledAblation40s

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

assert(isfile(diagModelFile),'UpperStopProbe model not found: %s',diagModelFile);
assert(isfile(scenarioFile),'Six-obstacle scenario not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    'Original Custom model should remain loaded; this script will not load or edit it.');

sourceDirtyBefore = get_param(sourceModelName,'Dirty');
sourceFileBefore = get_param(sourceModelName,'FileName');

wasDiagLoaded = bdIsLoaded(diagModelName);
if ~wasDiagLoaded
    load_system(diagModelFile);
end
cleanupDiag = onCleanup(@() closeDiagnosticWithoutSaving(diagModelName)); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 3 UPPER-LIMIT DISABLED A/B TEST — 40 s\n');
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original model dirty flag: %s\n',sourceDirtyBefore);

%% 1. Restore the same project workspace and six-obstacle inputs
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

%% 2. Remove the previous ful logger from this disposable diagnostic copy
fprintf('\n[2/5] Removing only the diagnostic ful logger, if present...\n');
converterPath = [diagModelName '/Probe_PS_Simulink_Obstacle3_ful'];
toWorkspacePath = [diagModelName '/Probe_ToWorkspace_Obstacle3_ful'];

if getSimulinkBlockHandle(converterPath) ~= -1
    delete_block(converterPath);
    fprintf('Removed temporary PS-Simulink Converter.\n');
end
if getSimulinkBlockHandle(toWorkspacePath) ~= -1
    delete_block(toWorkspacePath);
    fprintf('Removed temporary To Workspace logger.\n');
end

%% 3. Disable upper-limit sensing and the upper limit itself
fprintf('\n[3/5] Disabling Obstacle 3 upper limit on the diagnostic copy...\n');
jointPath = [diagModelName '/Obstacle3SpawnJoint'];

senseBefore = get_param(jointPath,'SenseUpperLimitTorqueForce');
limitBefore = get_param(jointPath,'UpperLimitSpecify');
fprintf('SenseUpperLimitTorqueForce before: %s\n',senseBefore);
fprintf('UpperLimitSpecify before:          %s\n',limitBefore);

assert(strcmpi(limitBefore,'on'), ...
    'Expected Obstacle 3 upper limit to be ON; refusing to run a non-comparable test.');

% Turn off force sensing first, because the old ful output may be removed
% when the upper limit is disabled. These are diagnostic-only settings.
if strcmpi(senseBefore,'on')
    set_param(jointPath,'SenseUpperLimitTorqueForce','off');
end
set_param(jointPath,'UpperLimitSpecify','off');

assert(strcmpi(get_param(jointPath,'UpperLimitSpecify'),'off'), ...
    'Could not disable Obstacle 3 upper limit.');
fprintf('SenseUpperLimitTorqueForce after: %s\n', ...
    get_param(jointPath,'SenseUpperLimitTorqueForce'));
fprintf('UpperLimitSpecify after:          %s\n', ...
    get_param(jointPath,'UpperLimitSpecify'));
fprintf('No other joint, tire, obstacle, terrain or solver parameter was changed.\n');

%% 4. Update and simulate with the same 40 s horizon
fprintf('\n[4/5] Updating and running the upper-limit-disabled diagnostic...\n');
set_param(diagModelName,'SimulationCommand','update');

stamp = datestr(now,'yyyymmdd_HHMMSS');
logName = ['simlog_UpperLimitDisabledAblation_' stamp];
matFile = fullfile(projectRoot, ...
    ['capturedObstacle3UpperLimitDisabledAblation40s_' stamp '.mat']);

testIn = Simulink.SimulationInput(diagModelName);
testIn = testIn.setModelParameter( ...
    'StopTime','40', ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

testOutput = sim(testIn);
assignin('base','obstacle3UpperLimitDisabledAblationOutput',testOutput);

%% 5. Save output and report the stop event
fprintf('\n[5/5] Saving output and reporting the simulation result...\n');
returnedNames = testOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));

try
    execInfo = testOutput.SimulationMetadata.ExecutionInfo;
    fprintf('Stop event: %s | stop time: %.12f s\n', ...
        execInfo.StopEvent,execInfo.StopEventTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end

try
    errText = char(testOutput.ErrorMessage);
    if ~isempty(strtrim(errText))
        fprintf('\nCaptured simulation error:\n%s\n',errText);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end

save(matFile,'testOutput','-v7.3');
fprintf('Saved diagnostic SimulationOutput: %s\n',matFile);

fprintf('\nOriginal model dirty flag before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));
assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original model dirty flag changed unexpectedly.');

fprintf('\n============================================================\n');
fprintf(' UPPER-LIMIT DISABLED A/B TEST FINISHED\n');
fprintf(' Original Custom model and scenario were not saved or modified.\n');
fprintf(' UpperStopProbe diagnostic copy was not saved; it is closed without saving.\n');
fprintf(' MAT: %s\n',matFile);
fprintf('============================================================\n');

function closeDiagnosticWithoutSaving(modelName)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
end
