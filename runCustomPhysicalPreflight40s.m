%% RUN CUSTOM PHYSICAL MODEL: 40-SECOND INTEGRATION PREFLIGHT
% Reconstructs the model's required base-workspace variables from the saved
% six-obstacle scenario, then runs a 40 s test using SimulationInput.
% The saved custom model StopTime remains 80 s. This script does not save or
% modify the model file, scenario MAT file, V2 model, or original checkpoint.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
cd(projectRoot);
addpath(projectRoot);

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
modelFile = fullfile(projectRoot, [modelName '.slx']);
scenarioFile = fullfile(projectRoot, ...
    'scenarioConfig_customization_physical_80s.mat');
testDuration = 40.0;

assert(isfile(modelFile), 'Custom physical model not found: %s', modelFile);
assert(isfile(scenarioFile), 'Custom physical scenario not found: %s', scenarioFile);

fprintf('\n============================================================\n');
fprintf(' CUSTOM PHYSICAL MODEL — 40 s INTEGRATION PREFLIGHT TEST\n');
fprintf('============================================================\n');

%% 1. Rebuild terrain/planner workspace and load the exact custom scenario
fprintf('\n[1/5] Restoring workspace and scenario...\n');
run(fullfile(projectRoot, 'setupOffRoadProject.m'));
S = load(scenarioFile, 'scenarioConfig');
assert(isfield(S, 'scenarioConfig'), ...
    'Scenario MAT file does not contain scenarioConfig.');
scenarioConfig = S.scenarioConfig;

assert(numel(scenarioConfig.obstacles) == 6, ...
    'Expected exactly six configured obstacles.');
assert(abs(double(scenarioConfig.simulation.stopTime) - 80) < 1e-9, ...
    'The source physical scenario should retain its 80 s full-run horizon.');

obs = scenarioConfig.obstacles(:);
[~, order] = sort([obs.id]);
obs = obs(order);
assert(isequal([obs.id], 1:6), ...
    'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]), [false false true false true false]), ...
    'Obstacle static/dynamic roles do not match the six-obstacle scenario.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalX = goalPoseWorkflow(1);
goalY = goalPoseWorkflow(2);
goalXY = goalPoseWorkflow(1:2);

%% 2. Reconstruct all variables referenced by Constant blocks / charts
fprintf('\n[2/5] Reconstructing obstacle and route inputs...\n');
allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1, 4)];
obstacleYs = [allCenters(:,2).' nan(1, 4)];
obstacleSpawnTime = [[obs.spawnTime] nan(1, 4)];
obstacle3X = double(obs(3).centerXY(1));
obstacle3Y = double(obs(3).centerXY(2));
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);

assert(exist('onlineReplanFromPoseGeometryAware', 'file') == 2, ...
    'onlineReplanFromPoseGeometryAware.m is not on the MATLAB path.');
staticObs = obs(~[obs.isDynamic]);
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded, initialWaypointCount, ~, initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware( ...
        startPoseWorkflow(1:2), goalXY, ...
        staticCenters(:,1).', staticCenters(:,2).', ...
        obstacleClearance, maxWaypoints);
assert(initialWaypointCount >= 2, 'Initial route calculation failed.');
waypoints = originalPathPadded(1:initialWaypointCount, :); %#ok<NASGU>
newPath = waypoints; %#ok<NASGU>

fprintf('Scenario: %s\n', scenarioConfig.name);
fprintf('Start XY: [%.3f %.3f] m\n', startPoseWorkflow(1:2));
fprintf('Goal XY:  [%.3f %.3f] m\n', goalXY);
fprintf('Six obstacles: validated\n');
fprintf('Initial static-obstacle route: %.3f m (%d waypoints)\n', ...
    initialRouteLength, initialWaypointCount);
fprintf('Obstacle 3 activation: %.3f s\n', obs(3).spawnTime);
fprintf('Obstacle 5 virtual activation: %.3f s\n', obs(5).spawnTime);

%% 3. Load the custom model without changing its saved StopTime
fprintf('\n[3/5] Loading custom model...\n');
wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelFile);
end
savedStopTime = get_param(modelName, 'StopTime');
fprintf('Saved model StopTime: %s s\n', savedStopTime);
fprintf('Preflight StopTime: %.1f s (temporary SimulationInput override)\n', ...
    testDuration);
fprintf('Simscape log history: up to 200,000 solver steps\n');

%% 4. Run the short preflight without saving model changes
fprintf('\n[4/5] Running 40-second simulation...\n');
simIn = Simulink.SimulationInput(modelName);
simscapeLogName = 'simlog_CustomPreflight40s';
simIn = simIn.setModelParameter( ...
    'StopTime', sprintf('%.15g', testDuration), ...
    'SimscapeLogType', 'all', ...
    'SimscapeLogName', simscapeLogName, ...
    'SimscapeLogLimitData', 'on', ...
    'SimscapeLogDataHistory', 200000); % Numeric positive integer, not text


try
    simOutCustomPreflight40s = sim(simIn);
catch ME
    fprintf(2, '\nPREFLIGHT SIMULATION FAILED\n%s\n', ...
        getReport(ME, 'extended', 'hyperlinks', 'off'));
    rethrow(ME);
end

assignin('base', 'simOutCustomPreflight40s', simOutCustomPreflight40s);
execInfo = simOutCustomPreflight40s.SimulationMetadata.ExecutionInfo;
fprintf('Stop event: %s at %.6f s\n', ...
    execInfo.StopEvent, execInfo.StopEventTime);

%% 5. Report logged rover motion, replanning and sensor detections
fprintf('\n[5/5] Checking simulation outputs...\n');
logNames = simOutCustomPreflight40s.who;
fprintf('Logged outputs: %s\n', strjoin(logNames, ', '));

assert(strcmp(execInfo.StopEvent, 'ReachedStopTime'), ...
    'Simulation did not reach the requested 40-second stop time.');

if all(ismember({'mb_px', 'mb_py'}, logNames))
    px = simOutCustomPreflight40s.get('mb_px');
    py = simOutCustomPreflight40s.get('mb_py');
    pxData = double(px.Data(:));
    pyData = double(py.Data(:));
    pxTime = double(px.Time(:));
    pyTime = double(py.Time(:));
    n = min([numel(pxData), numel(pyData), numel(pxTime), numel(pyTime)]);
    assert(n >= 2, 'Rover position logs contain fewer than two samples.');
    roverTravel = sum(hypot(diff(pxData(1:n)), diff(pyData(1:n))));
    fprintf('Rover initial XY: [%.3f %.3f] m\n', pxData(1), pyData(1));
    fprintf('Rover final XY:   [%.3f %.3f] m\n', pxData(n), pyData(n));
    fprintf('Rover logged path travelled during preflight: %.3f m\n', roverTravel);
else
    warning('Expected mb_px/mb_py rover position logs were not found.');
end

if any(strcmp(logNames, 'onlineReplanEventLog'))
    eventTS = simOutCustomPreflight40s.get('onlineReplanEventLog');
    eventData = double(eventTS.Data(:));
    eventTime = double(eventTS.Time(:));
    eventRise = find(diff([0; eventData > 0.5]) == 1);
    fprintf('Online replanning events during first %.1f s: %d\n', ...
        testDuration, numel(eventRise));
    for k = 1:numel(eventRise)
        fprintf('  Event %d at %.4f s\n', k, eventTime(eventRise(k)));
    end
else
    fprintf('onlineReplanEventLog: not present in simulation outputs.\n');
end

if any(strcmp(logNames, 'sensorDetectedLog'))
    sensorTS = simOutCustomPreflight40s.get('sensorDetectedLog');
    fprintf('Sensor detection log unique values: ');
    disp(unique(sensorTS.Data(:)).');
else
    fprintf('sensorDetectedLog: not present in simulation outputs.\n');
end

%% Inspect the Simscape log for the actual obstacle-3 joint motion
if any(strcmp(logNames, simscapeLogName))
    fprintf('\nObstacle 3 prismatic-joint Simscape log:');
    simlogCustomPreflight40s = simOutCustomPreflight40s.get(simscapeLogName);
    try
        obstacle3JointNode = simscape.logging.findNode( ...
            simlogCustomPreflight40s, ...
            [modelName '/Obstacle3SpawnJoint']);
        if isempty(obstacle3JointNode)
            fprintf('\n  Joint node was not found in the Simscape log.');
        else
            fprintf('\n');
            print(obstacle3JointNode);
            scanSimscapeNode(obstacle3JointNode, 'Obstacle3SpawnJoint', 0, 4);
        end
    catch logME
        fprintf('\n  Could not inspect joint log: %s\n', logME.message);
        disp(simlogCustomPreflight40s);
    end
else
    fprintf('\nSimscape log variable %s was not returned.\n', simscapeLogName);
end

fprintf('\nSaved model StopTime after test: %s s\n', ...
    get_param(modelName, 'StopTime'));
fprintf('\n============================================================\n');
fprintf(' 40 s PREFLIGHT COMPLETED\n');
fprintf(' Model file was not saved or modified by this test.\n');
fprintf(' Simulation output: simOutCustomPreflight40s\n');
fprintf('============================================================\n');

% Close only if this script opened the model. Keep the SimulationOutput in base.
if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end


%% Local helper: recursively report logged scalar/vector series for a block node
function scanSimscapeNode(node, nodePath, depth, maxDepth)
    if depth > maxDepth
        return;
    end

    ignored = {'id','savable','exportable','series'};
    propNames = properties(node);

    for i = 1:numel(propNames)
        propName = propNames{i};
        if any(strcmp(propName, ignored))
            continue;
        end

        try
            child = node.(propName);
        catch
            continue;
        end

        if isa(child, 'simscape.logging.Node')
            childPath = [nodePath '.' propName];
            try
                seriesObj = child.series;
                if isa(seriesObj, 'simscape.logging.Series') && seriesObj.points > 0
                    valuesVector = values(seriesObj);
                    timesVector = time(seriesObj);
                    finiteValues = double(valuesVector(isfinite(valuesVector)));
                    if ~isempty(finiteValues)
                        fprintf('  %s | points=%d | unit=%s | range=[%.6g, %.6g] | t=[%.4f, %.4f] s\n', ...
                            childPath, seriesObj.points, char(seriesObj.unit), ...
                            min(finiteValues), max(finiteValues), ...
                            timesVector(1), timesVector(end));
                    else
                        fprintf('  %s | points=%d | unit=%s | non-finite values\n', ...
                            childPath, seriesObj.points, char(seriesObj.unit));
                    end
                    continue;
                end
            catch
                % Non-leaf nodes do not expose a populated series.
            end
            scanSimscapeNode(child, childPath, depth+1, maxDepth);
        end
    end
end
