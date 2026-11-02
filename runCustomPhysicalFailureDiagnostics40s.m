%% CAPTURE PARTIAL RESULTS FROM THE 40 s CUSTOM PHYSICAL PREFLIGHT
% Runs the same 40 s custom-scenario test with CaptureErrors enabled.
% If Simscape fails, MATLAB returns the SimulationOutput up to the failure
% time so that rover signals and physical-network logs can be inspected.
% This script does not save or modify any model/scenario file.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
modelFile = fullfile(projectRoot,[modelName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
testDuration = 40.0;
simscapeLogName = 'simlog_CustomFailureDiag40s';

assert(isfile(modelFile),'Custom model not found: %s',modelFile);
assert(isfile(scenarioFile),'Custom scenario not found: %s',scenarioFile);

fprintf('\n============================================================\n');
fprintf(' CUSTOM PHYSICAL MODEL — CAPTURE FAILURE DIAGNOSTICS\n');
fprintf('============================================================\n');

%% 1. Restore workspace and exact custom scenario
fprintf('\n[1/6] Restoring workspace and scenario...\n');
run(fullfile(projectRoot,'setupOffRoadProject.m'));
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,ord] = sort([obs.id]);
obs = obs(ord);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Obstacle roles do not match the saved six-obstacle scenario.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalX = goalPoseWorkflow(1);
goalY = goalPoseWorkflow(2);
goalXY = goalPoseWorkflow(1:2);

%% 2. Reconstruct chart and constant-block workspace variables
fprintf('\n[2/6] Reconstructing obstacle and route inputs...\n');
allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1,4)];
obstacleYs = [allCenters(:,2).' nan(1,4)];
obstacleSpawnTime = [[obs.spawnTime] nan(1,4)];
obstacle3X = double(obs(3).centerXY(1));
obstacle3Y = double(obs(3).centerXY(2));
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
fprintf('Initial static-obstacle route: %.3f m (%d waypoints)\n', ...
    initialRouteLength,initialWaypointCount);
fprintf('Obstacle 3 activation: %.3f s; obstacle 5 activation: %.3f s\n', ...
    obs(3).spawnTime,obs(5).spawnTime);

%% 3. Load model and inspect existing solver settings (read-only)
fprintf('\n[3/6] Loading custom model and reading solver settings...\n');
wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded, load_system(modelFile); end
savedStopTime = get_param(modelName,'StopTime');
fprintf('Saved model StopTime: %s s\n',savedStopTime);

modelParameters = {'Solver','RelTol','AbsTol','MaxStep','MinStep','SolverType'};
for k = 1:numel(modelParameters)
    try
        fprintf('Model %-12s = %s\n',modelParameters{k}, ...
            get_param(modelName,modelParameters{k}));
    catch
    end
end

solverBlocks = find_system(modelName, ...
    'LookUnderMasks','all','FollowLinks','off','Type','Block');
for k = 1:numel(solverBlocks)
    if strcmpi(get_param(solverBlocks{k},'Name'),'Solver Configuration')
        fprintf('\nSolver Configuration block: %s\n',solverBlocks{k});
        try
            dp = get_param(solverBlocks{k},'DialogParameters');
            pnames = fieldnames(dp);
            for j = 1:numel(pnames)
                pname = pnames{j};
                if contains(lower(pname),'solver') || contains(lower(pname),'nonlinear') || ...
                   contains(lower(pname),'tolerance') || contains(lower(pname),'iteration') || ...
                   contains(lower(pname),'local') || contains(lower(pname),'impulse')
                    try
                        fprintf('  %s = %s\n',pname, ...
                            strtrim(evalc('disp(get_param(solverBlocks{k},pname))')));
                    catch
                    end
                end
            end
        catch ME
            fprintf('  Could not read Solver Configuration parameters: %s\n',ME.message);
        end
    end
end

%% 4. Run with CaptureErrors enabled to retain partial output
fprintf('\n[4/6] Running 40-second simulation with CaptureErrors=on...\n');
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',simscapeLogName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

simOutCustomFailureDiag40s = sim(simIn);
assignin('base','simOutCustomFailureDiag40s',simOutCustomFailureDiag40s);

%% 5. Report failure metadata and all partial Simulink outputs
fprintf('\n[5/6] Reporting simulation status and partial results...\n');
logNames = simOutCustomFailureDiag40s.who;
fprintf('Returned outputs: %s\n',strjoin(logNames,', '));

errorMessage = '';
try
    errorMessage = char(simOutCustomFailureDiag40s.ErrorMessage);
catch
end

execInfo = simOutCustomFailureDiag40s.SimulationMetadata.ExecutionInfo;
try
    fprintf('Stop event: %s\n',execInfo.StopEvent);
    fprintf('Stop time: %.9f s\n',execInfo.StopEventTime);
catch
end

if ~isempty(strtrim(errorMessage))
    fprintf('\nCAPTURED SIMULATION ERROR:\n%s\n',errorMessage);
    try
        diagnostic = execInfo.ErrorDiagnostic;
        fprintf('\nERROR DIAGNOSTIC OBJECT:\n');
        disp(diagnostic);
    catch
    end
    try
        sldiagviewer.reportSimulationMetadataDiagnostics(simOutCustomFailureDiag40s);
    catch
    end
else
    fprintf('No simulation error was captured.\n');
end

if all(ismember({'mb_px','mb_py'},logNames))
    px = simOutCustomFailureDiag40s.get('mb_px');
    py = simOutCustomFailureDiag40s.get('mb_py');
    pxD = double(px.Data(:)); pyD = double(py.Data(:));
    pxT = double(px.Time(:)); pyT = double(py.Time(:));
    n = min([numel(pxD),numel(pyD),numel(pxT),numel(pyT)]);
    fprintf('\nROVER PARTIAL TRAJECTORY\n');
    fprintf('Last common logged time: %.9f s\n',min(pxT(n),pyT(n)));
    fprintf('Initial XY: [%.5f %.5f] m\n',pxD(1),pyD(1));
    fprintf('Last XY:    [%.5f %.5f] m\n',pxD(n),pyD(n));
    fprintf('Logged path length before stop: %.5f m\n', ...
        sum(hypot(diff(pxD(1:n)),diff(pyD(1:n)))));
    fprintf('Last 12 rover samples [time X Y]\n');
    first = max(1,n-11);
    for k = first:n
        fprintf('%.6f  %.6f  %.6f\n',pxT(k),pxD(k),pyD(k));
    end
else
    fprintf('\nRover position logs mb_px/mb_py are unavailable.\n');
end

if any(strcmp(logNames,'onlineReplanEventLog'))
    e = simOutCustomFailureDiag40s.get('onlineReplanEventLog');
    ed = double(e.Data(:)); et = double(e.Time(:));
    rises = find(diff([0;ed>0.5])==1);
    fprintf('\nREPLANNING EVENTS\n');
    for k = 1:numel(rises)
        fprintf('Event %d at %.6f s\n',k,et(rises(k)));
    end
end

if any(strcmp(logNames,'sensorDetectedLog'))
    sdet = simOutCustomFailureDiag40s.get('sensorDetectedLog');
    fprintf('\nSensor detection unique values: ');
    disp(unique(sdet.Data(:)).');
end

%% 6. Inspect relevant Simscape branches up to the failure time
fprintf('\n[6/6] Inspecting physical-network logs...\n');
if any(strcmp(logNames,simscapeLogName))
    simlog = simOutCustomFailureDiag40s.get(simscapeLogName);
    nodesToInspect = { ...
        'Obstacle2SpawnJoint', ...
        'Obstacle3SpawnJoint', ...
        'Spatial Contact Force', ...
        'Spatial Contact Force Obstacle2', ...
        'Spatial Contact Force Obstacle3'};
    for k = 1:numel(nodesToInspect)
        fullNodePath = [modelName '/' nodesToInspect{k}];
        fprintf('\n--- %s ---\n',nodesToInspect{k});
        try
            node = simscape.logging.findNode(simlog,fullNodePath);
            if isempty(node)
                fprintf('Node not present in returned Simscape log.\n');
            else
                print(node);
            end
        catch ME
            fprintf('Could not inspect node: %s\n',ME.message);
        end
    end
else
    fprintf('Simscape log was not returned.\n');
end

fprintf('\nSaved model StopTime after diagnostic run: %s s\n', ...
    get_param(modelName,'StopTime'));
fprintf('\n============================================================\n');
fprintf(' FAILURE DIAGNOSTICS COMPLETE\n');
fprintf(' Partial output retained as simOutCustomFailureDiag40s\n');
fprintf(' No model or scenario file was saved or modified.\n');
fprintf('============================================================\n');

if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName,0);
end
