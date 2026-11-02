%% CUSTOM MODEL — OBSTACLE 3 UPPER-LIMIT FORCE PROBE (40 s)
% Creates a separate diagnostic copy of the currently loaded custom model,
% enables only Obstacle 3's upper-limit force sensing on that copy, and runs
% a 40 s CaptureErrors simulation with full Simscape logging.
%
% The original custom model is NOT edited, saved, renamed, or closed.
% The six-obstacle scenario MAT file is read-only.
% A diagnostic SLX copy and partial SimulationOutput MAT file are created.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

sourceModelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
sourceModelFile = fullfile(projectRoot,[sourceModelName '.slx']);
scenarioFile = fullfile(projectRoot, ...
    'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot,'setupOffRoadProject.m');
testDuration = 40.0;

assert(isfile(sourceModelFile),'Custom model file not found: %s',sourceModelFile);
assert(isfile(scenarioFile),'Custom scenario file not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup file not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    ['Please leave the original custom model loaded in MATLAB. ' ...
     'This script clones its current in-memory state without editing it.']);

% Use unique diagnostic filenames so no earlier probe is overwritten.
stamp = datestr(now,'yyyymmdd_HHMMSS');
diagnosticModelName = [sourceModelName '_UpperStopProbe_' stamp];
diagnosticModelFile = fullfile(projectRoot, ...
    [diagnosticModelName '.slx']);
logName = ['simlog_UpperStopProbe_' stamp];
outputVariableName = ['simOutUpperStopProbe_' stamp];
captureFile = fullfile(projectRoot, ...
    ['capturedObstacle3UpperLimitForceProbe40s_' stamp '.mat']);
treeFile = fullfile(projectRoot, ...
    ['obstacle3UpperLimitForceProbeSimscapeTree_' stamp '.txt']);

assert(~bdIsLoaded(diagnosticModelName), ...
    'Diagnostic model is already loaded: %s',diagnosticModelName);
assert(~isfile(diagnosticModelFile), ...
    'Diagnostic model file already exists: %s',diagnosticModelFile);

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 3 UPPER-LIMIT FORCE PROBE — 40 s\n');
fprintf('============================================================\n');
fprintf('Original model: %s\n',sourceModelName);
fprintf('Original model dirty flag before probe: %s\n', ...
    get_param(sourceModelName,'Dirty'));

%% 1. Restore the project workspace and exact saved six-obstacle scenario
fprintf('\n[1/5] Restoring project workspace and scenario inputs...\n');
run(setupFile);
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig is missing from MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Unexpected obstacle roles in the six-obstacle scenario.');
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

fprintf('Scenario: %s\n',scenarioConfig.name);
fprintf('Obstacle IDs: %s\n',mat2str([obs.id]));
fprintf('Physical obstacle IDs: 1, 2, 3; virtual obstacle IDs: 4, 5, 6.\n');
fprintf('Obstacle 3 activation: %.3f s; obstacle 5 activation: %.3f s.\n', ...
    obs(3).spawnTime,obs(5).spawnTime);
fprintf('Initial static-obstacle route: %.3f m (%d waypoints).\n', ...
    initialRouteLength,initialWaypointCount);

%% 2. Clone the CURRENT IN-MEMORY source model without changing its name
fprintf('\n[2/5] Creating an isolated diagnostic model copy...\n');
new_system(diagnosticModelName,'FromFile',sourceModelName);
assert(bdIsLoaded(diagnosticModelName), ...
    'Simulink did not create the diagnostic model copy.');

diagnosticJointPath = [diagnosticModelName '/Obstacle3SpawnJoint'];
sourceJointPath = [sourceModelName '/Obstacle3SpawnJoint'];

% Verify the clone inherited the original joint configuration.
compareParams = {'PositionTargetValue','UpperLimitBound', ...
    'UpperLimitStiffness','UpperLimitDamping', ...
    'UpperLimitTransitionRegionWidth','SenseUpperLimitForce'};
for k = 1:numel(compareParams)
    sourceValue = get_param(sourceJointPath,compareParams{k});
    cloneValue = get_param(diagnosticJointPath,compareParams{k});
    fprintf('Clone check %-34s source=%s | clone=%s\n', ...
        compareParams{k},sourceValue,cloneValue);
    assert(strcmp(sourceValue,cloneValue), ...
        'Diagnostic clone differs from source for parameter %s.', ...
        compareParams{k});
end

%% 3. Enable only the upper-limit force sensor on the diagnostic copy
fprintf('\n[3/5] Enabling Obstacle 3 upper-limit force sensing on the copy...\n');
assert(isfield(get_param(diagnosticJointPath,'DialogParameters'), ...
    'SenseUpperLimitForce'), ...
    'SenseUpperLimitForce is not exposed by this joint block.');
set_param(diagnosticJointPath,'SenseUpperLimitForce','on');
assert(strcmpi(get_param(diagnosticJointPath,'SenseUpperLimitForce'),'on'), ...
    'Could not enable upper-limit force sensing.');

% Compile/update only the diagnostic copy, then persist this instrumented copy.
set_param(diagnosticModelName,'SimulationCommand','update');
save_system(diagnosticModelName,diagnosticModelFile);
fprintf('Diagnostic copy saved: %s\n',diagnosticModelFile);
fprintf('Original model remains loaded and untouched.\n');

%% 4. Simulate diagnostic copy with captured Simscape log
fprintf('\n[4/5] Running 40 s upper-limit force diagnostic...\n');
simIn = Simulink.SimulationInput(diagnosticModelName);
simIn = simIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

upperStopProbeOutput = sim(simIn);
assignin('base',outputVariableName,upperStopProbeOutput);

%% 5. Save partial output, log tree, and try to extract ful force signal
fprintf('\n[5/5] Saving diagnostic output and extracting upper-limit force...\n');
returnedNames = upperStopProbeOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));

try
    execInfo = upperStopProbeOutput.SimulationMetadata.ExecutionInfo;
    fprintf('Stop event: %s\n',execInfo.StopEvent);
    fprintf('Stop time: %.12f s\n',execInfo.StopEventTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end

try
    err = char(upperStopProbeOutput.ErrorMessage);
    if ~isempty(strtrim(err))
        fprintf('\nCaptured simulation error:\n%s\n',err);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end

save(captureFile,'upperStopProbeOutput','-v7.3');
fprintf('Saved SimulationOutput: %s\n',captureFile);

if any(strcmp(returnedNames,logName))
    simlog = upperStopProbeOutput.get(logName);
    treeText = evalc('print(simlog)');
    fid = fopen(treeFile,'w');
    assert(fid ~= -1,'Could not create Simscape tree report.');
    fwrite(fid,treeText,'char');
    fclose(fid);
    fprintf('Saved Simscape logging tree: %s\n',treeFile);

    try
        fulNode = simlog.Obstacle3SpawnJoint.Pz.ful;
        fulSeries = fulNode.series;
        forceTime = double(time(fulSeries));
        forceValue = double(values(fulSeries));
        forceTime = forceTime(:);
        forceValue = reshape(forceValue,numel(forceTime),[]);

        fprintf('\n===== OBSTACLE 3 UPPER-LIMIT FORCE (ful) =====\n');
        fprintf('Samples: %d\n',numel(forceTime));
        fprintf('Logged time range: %.12f to %.12f s\n', ...
            forceTime(1),forceTime(end));
        fprintf('Force min/max: %.12g / %.12g N\n', ...
            min(forceValue(:,1)),max(forceValue(:,1)));

        windows = [6.9 7.1; 14.7 15.1; 29.2 29.624814507405514];
        windowLabels = {'Obstacle 3 command window', ...
            'Near obstacle 5 activation', 'Final failure window'};
        for w = 1:size(windows,1)
            idx = find(forceTime >= windows(w,1) & forceTime <= windows(w,2));
            fprintf('\n%s [%.6f, %.6f] s\n', ...
                windowLabels{w},windows(w,1),windows(w,2));
            if isempty(idx)
                fprintf('No samples.\n');
            else
                n = min(12,numel(idx));
                pick = unique([idx(1:min(6,numel(idx)));idx(max(1,end-5):end)]);
                disp(table(forceTime(pick),forceValue(pick,1), ...
                    'VariableNames',{'Time_s','UpperLimitForce_N'}));
            end
        end

        forceReportFile = fullfile(projectRoot, ...
            ['obstacle3UpperLimitForceValues_' stamp '.csv']);
        writetable(table(forceTime,forceValue(:,1), ...
            'VariableNames',{'Time_s','UpperLimitForce_N'}),forceReportFile);
        fprintf('Saved ful force CSV: %s\n',forceReportFile);
    catch ME
        fprintf(2,['Could not directly extract simlog.Obstacle3SpawnJoint.Pz.ful: ' ...
            '%s\n'],ME.message);
        fprintf('Use the saved logging tree to verify the exact ful node path.\n');
    end
else
    warning('The requested Simscape log %s was not returned.',logName);
end

fprintf('\n============================================================\n');
fprintf(' UPPER-LIMIT FORCE PROBE COMPLETE\n');
fprintf('Original model file/scenario were not saved or modified.\n');
fprintf('Original model dirty flag after probe: %s\n', ...
    get_param(sourceModelName,'Dirty'));
fprintf('Diagnostic model copy: %s\n',diagnosticModelFile);
fprintf('Captured output: %s\n',captureFile);
fprintf('============================================================\n');

% Leave the original model loaded exactly as it was. Close only the probe copy;
% the separately saved diagnostic .slx and MAT output remain on disk.
if bdIsLoaded(diagnosticModelName)
    close_system(diagnosticModelName,0);
end
