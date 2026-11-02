%% runRoughTerrainWorkflow
% Single-entry-point workflow for the saved rough-terrain scenario.
% 1) Rebuild terrain/planner workspace, 2) restore scenarioConfig.mat,
% 3) synchronize an isolated ScenarioIntegrationTest model, 4) simulate,
% 5) report replanning events, and 6) launch the multi-dynamic viewer.
%
% IMPORTANT:
% - Save your edits from the Scenario Editor using "Save scenario MAT" first.
% - Only OffRoadMultibodyStage1_ScenarioIntegrationTest.slx is updated/saved.
% - OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx is never modified.
% - Only obstacle 1 has physical Simscape collision geometry in this test copy;
%   other obstacles are virtual for detection and planning and are viewer graphics.

%% 0. Resolve project root and required assets
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
if ~strcmpi(pwd, projectRoot)
    cd(projectRoot);
end
addpath(projectRoot);

scenarioFile = fullfile(projectRoot, 'scenarioConfig.mat');
setupFile = fullfile(projectRoot, 'setupOffRoadProject.m');
builderFile = fullfile(projectRoot, 'createScenarioIntegrationTestModel.m');
testModelName = 'OffRoadMultibodyStage1_ScenarioIntegrationTest';
testModelFile = fullfile(projectRoot, [testModelName '.slx']);
sourceModelName = 'OffRoadMultibodyStage1_OnlineReplanCheckpoint';
sourceModelFile = fullfile(projectRoot, [sourceModelName '.slx']);

assert(isfile(scenarioFile), ...
    'Saved scenario not found: %s. Save it from the Scenario Editor first.', scenarioFile);
assert(isfile(setupFile), 'Missing workspace setup script: %s', setupFile);
assert(isfile(sourceModelFile), 'Missing checkpoint model: %s', sourceModelFile);
assert(isfile(fullfile(projectRoot, 'onlineReplanFromPoseGeometryAware.m')), ...
    'Missing onlineReplanFromPoseGeometryAware.m in the project root.');
assert(isfile(fullfile(projectRoot, 'launchElevationTerrainRoverViewerMultiDynamic.m')), ...
    'Missing launchElevationTerrainRoverViewerMultiDynamic.m in the project root.');

savedScenarioData = load(scenarioFile, 'scenarioConfig');
assert(isfield(savedScenarioData, 'scenarioConfig') && ...
       isstruct(savedScenarioData.scenarioConfig), ...
       'scenarioConfig.mat does not contain a valid scenarioConfig struct.');
savedScenario = savedScenarioData.scenarioConfig;

fprintf('\n============================================================\n');
fprintf(' ROUGH-TERRAIN SCENARIO WORKFLOW\n');
fprintf('============================================================\n');
fprintf('Project root: %s\n', projectRoot);
fprintf('Saved scenario: %s\n', scenarioFile);

%% 1. Rebuild dependencies, then restore the user-edited scenario
fprintf('\n[1/6] Rebuilding terrain/planner workspace...\n');
run(setupFile);
scenarioConfig = savedScenario;  % setup initializes generic defaults; restore saved edits after it.

% Sort consistently by obstacle ID and validate the current test-model scope.
obs = scenarioConfig.obstacles(:);
assert(~isempty(obs), 'The saved scenario contains no obstacles.');
if isfield(obs, 'id')
    [~, order] = sort([obs.id]);
    obs = obs(order);
end
assert(numel(obs) == 3, ...
    ['The current integration-test model supports exactly three obstacles. ' ...
     'The scenario was not simulated.']);
assert(all(arrayfun(@(o) strcmpi(o.shape, 'box'), obs)), ...
    'The current integration-test planner supports box obstacles only.');
assert(all(arrayfun(@(o) numel(o.centerXY) == 2 && numel(o.dimensions) == 3, obs)), ...
    'Each obstacle must define centerXY=[x y] and dimensions=[length width height].');
assert(~obs(1).isDynamic, ...
    'Obstacle 1 must remain static because it maps to the one physical brick in the test model.');
assert(all(obs(1).dimensions > 0) && all(vertcat(obs.dimensions) > 0, 'all'), ...
    'Obstacle dimensions must be positive.');
for k = 1:numel(obs)
    if obs(k).isDynamic
        assert(isfinite(obs(k).spawnTime) && obs(k).spawnTime >= 0, ...
            'Dynamic obstacle %d requires a finite nonnegative spawnTime.', obs(k).id);
    else
        assert(isnan(obs(k).spawnTime), ...
            'Static obstacle %d should have spawnTime=NaN.', obs(k).id);
    end
end
assert(isfield(scenarioConfig, 'startPose') && numel(scenarioConfig.startPose) >= 3, ...
    'scenarioConfig.startPose must be [x y yaw].');
assert(isfield(scenarioConfig, 'goalPose') && numel(scenarioConfig.goalPose) >= 2, ...
    'scenarioConfig.goalPose must contain [x y].');
assert(isfield(scenarioConfig, 'simulation') && ...
       isfield(scenarioConfig.simulation, 'stopTime') && ...
       isfinite(scenarioConfig.simulation.stopTime) && scenarioConfig.simulation.stopTime > 0, ...
       'scenarioConfig.simulation.stopTime must be positive and finite.');

startPose = double(scenarioConfig.startPose(:).');
goalPose = double(scenarioConfig.goalPose(:).');
goalXY = goalPose(1:2);
goalX = goalXY(1);
goalY = goalXY(2);
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);

allCenters = vertcat(obs.centerXY);
obstacleXs = allCenters(:,1).';
obstacleYs = allCenters(:,2).';
obstacleSpawnTime = [obs.spawnTime];
obstacle3X = obstacleXs(3);  % Retained for compatibility with the copied model's ports.
obstacle3Y = obstacleYs(3);

staticMask = ~[obs.isDynamic];
staticObs = obs(staticMask);
staticCenters = vertcat(staticObs.centerXY);
staticXs = staticCenters(:,1).';
staticYs = staticCenters(:,2).';

assert(all(startPose(1:2) >= [min(X(:)) min(Y(:))]) && ...
       all(startPose(1:2) <= [max(X(:)) max(Y(:))]), ...
       'Start position is outside the terrain bounds.');
assert(all(goalXY >= [min(X(:)) min(Y(:))]) && ...
       all(goalXY <= [max(X(:)) max(Y(:))]), ...
       'Goal position is outside the terrain bounds.');

%% 2. Create the isolated test copy if this is the first run
if ~isfile(testModelFile)
    assert(~bdIsLoaded(testModelName), ...
        'The test model is loaded but its SLX file is missing. Save/close it manually before continuing.');
    fprintf('\n[2/6] Test model does not exist; creating isolated test copy...\n');
    assert(isfile(builderFile), 'Missing test-model builder: %s', builderFile);
    run(builderFile);  % Builder preserves the baseline and does not simulate.
    % Builder calls setup as well; re-restore the saved config and current values below.
    scenarioConfig = savedScenario;
    startPose = double(scenarioConfig.startPose(:).');
    goalPose = double(scenarioConfig.goalPose(:).');
    goalXY = goalPose(1:2);
    goalX = goalXY(1);
    goalY = goalXY(2);
    obs = scenarioConfig.obstacles(:);
    if isfield(obs, 'id')
        [~, order] = sort([obs.id]);
        obs = obs(order);
    end
    allCenters = vertcat(obs.centerXY);
    obstacleXs = allCenters(:,1).';
    obstacleYs = allCenters(:,2).';
    obstacleSpawnTime = [obs.spawnTime];
    obstacle3X = obstacleXs(3);
    obstacle3Y = obstacleYs(3);
    obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
    maxWaypoints = double(scenarioConfig.planning.maxWaypoints);
    staticMask = ~[obs.isDynamic];
    staticObs = obs(staticMask);
    staticCenters = vertcat(staticObs.centerXY);
    staticXs = staticCenters(:,1).';
    staticYs = staticCenters(:,2).';
else
    fprintf('\n[2/6] Using existing isolated test copy: %s\n', testModelFile);
end

if ~bdIsLoaded(testModelName)
    load_system(testModelFile);
end

% Verify this saved copy contains the multi-obstacle integration charts.
rt = sfroot;
envChart = find(rt, '-isa', 'Stateflow.EMChart', ...
    'Path', [testModelName '/DynamicObstacleEnvironment']);
managerChart = find(rt, '-isa', 'Stateflow.EMChart', ...
    'Path', [testModelName '/OnlineReplanManager']);
assert(~isempty(envChart) && contains(char(envChart(1).Script), 'spawnTime(k)'), ...
    ['The test model does not contain the expected multi-obstacle environment chart. ' ...
     'Back up and rebuild the TEST copy with createScenarioIntegrationTestModel.m.']);
assert(~isempty(managerChart) && ...
       contains(char(managerChart(1).Script), 'onlineReplanFromPoseGeometryAware'), ...
    ['The test model does not contain the geometry-aware replan manager. ' ...
     'Back up and rebuild the TEST copy with createScenarioIntegrationTestModel.m.']);

%% 3. Recalculate the starting route and synchronize model constants
fprintf('\n[3/6] Recalculating initial route and synchronizing model...\n');
[originalPathPadded, initialWaypointCount, initialClearances, initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware(startPose(1:2), goalXY, ...
    staticXs, staticYs, obstacleClearance, maxWaypoints); %#ok<ASGLU>
originalPath = originalPathPadded(1:initialWaypointCount, :);
replannedPathPadded = originalPathPadded; %#ok<NASGU>
waypoints = originalPath; %#ok<NASGU>
newPath = originalPath; %#ok<NASGU>

expectedValues = {
    'ObstacleX',         'obstacleXs'
    'ObstacleY',         'obstacleYs'
    'Obstacle3X',        'obstacle3X'
    'Obstacle3Y',        'obstacle3Y'
    'ObstacleSpawnTime', 'obstacleSpawnTime'
    'PathWaypoints',     'originalPathPadded'
    'goalX',             'goalX'
    'goalY',             'goalY'
};
for k = 1:size(expectedValues,1)
    blockPath = [testModelName '/' expectedValues{k,1}];
    assert(getSimulinkBlockHandle(blockPath) ~= -1, ...
        'Required block missing from test copy: %s', blockPath);
    set_param(blockPath, 'Value', expectedValues{k,2});
end

% Update the one physical brick and its terrain-relative placement variables.
obstacleLength = double(obs(1).dimensions(1));
obstacleWidth = double(obs(1).dimensions(2));
obstacleHeight = double(obs(1).dimensions(3));
underFirstObstacle = abs(X - obstacleXs(1)) <= obstacleLength/2 & ...
                     abs(Y - obstacleYs(1)) <= obstacleWidth/2;
assert(any(underFirstObstacle(:)), ...
    'Could not find terrain samples beneath obstacle 1.');
maxTerrainZUnderObstacle = max(Z_multibody(underFirstObstacle));
obstacleCenterZ = maxTerrainZUnderObstacle + 0.005 + obstacleHeight/2;
zGround = obstacleCenterZ + 0.15; %#ok<NASGU>
brickPath = [testModelName '/PhysicalObstacle1'];
assert(getSimulinkBlockHandle(brickPath) ~= -1, 'PhysicalObstacle1 is missing from test copy.');
set_param(brickPath, 'BrickDimensions', '[obstacleLength obstacleWidth obstacleHeight]');

% Apply configured rover initial XY and yaw to the copied 6-DOF Joint.
% Adjust Z by the terrain elevation change to preserve its clearance relative to the terrain.
jointPath = [testModelName '/6-DOF Joint'];
assert(getSimulinkBlockHandle(jointPath) ~= -1, ...
    'The expected 6-DOF Joint was not found in the test model.');
oldX = str2double(get_param(jointPath, 'PxPositionTargetValue'));
oldY = str2double(get_param(jointPath, 'PyPositionTargetValue'));
oldZText = get_param(jointPath, 'PzPositionTargetValue');
oldZ = str2double(oldZText);
oldQuaternion = get_param(jointPath, 'SphPositionTargetQuaternion');
zAtOld = interp2(X, Y, Z_multibody, oldX, oldY, 'linear');
zAtNew = interp2(X, Y, Z_multibody, startPose(1), startPose(2), 'linear');
assert(all(isfinite([oldX oldY oldZ zAtOld zAtNew])), ...
    'Could not calculate terrain-relative initial pose.');
newZ = oldZ + (zAtNew - zAtOld);
yaw = startPose(3);
newQuaternion = [cos(yaw/2) 0 0 sin(yaw/2)];

try
    set_param(jointPath, ...
        'PxPositionTargetValue', sprintf('%.15g', startPose(1)), ...
        'PyPositionTargetValue', sprintf('%.15g', startPose(2)), ...
        'PzPositionTargetValue', sprintf('%.15g', newZ), ...
        'SphPositionTargetQuaternion', mat2str(newQuaternion, 15));
    set_param(testModelName, 'SimulationCommand', 'update');
    save_system(testModelName, testModelFile);
catch ME
    % Roll back only the initial-state targets to avoid leaving a partial edit.
    try
        set_param(jointPath, ...
            'PxPositionTargetValue', sprintf('%.15g', oldX), ...
            'PyPositionTargetValue', sprintf('%.15g', oldY), ...
            'PzPositionTargetValue', oldZText, ...
            'SphPositionTargetQuaternion', oldQuaternion);
    catch
    end
    rethrow(ME);
end

fprintf('Scenario: %s\n', scenarioConfig.name);
fprintf('Start target: (%.3f, %.3f, %.4f) m; yaw %.4f rad\n', ...
    startPose(1), startPose(2), newZ, yaw);
fprintf('Goal: (%.3f, %.3f) m\n', goalX, goalY);
fprintf('Initial route: %.3f m; %d waypoints\n', initialRouteLength, initialWaypointCount);
fprintf('Static obstacles: %d; dynamic obstacles: %d\n', ...
    sum(~[obs.isDynamic]), sum([obs.isDynamic]));

%% 4. Run the separate integration-test model
stopTime = double(scenarioConfig.simulation.stopTime);
fprintf('\n[4/6] Simulating %s for %.3f s...\n', testModelName, stopTime);
simOutScenarioWorkflow = sim(testModelName, 'StopTime', sprintf('%.15g', stopTime));
simOut = simOutScenarioWorkflow;  % Viewer expects a base-workspace variable named simOut.

%% 5. Report initial/final pose and online replanning events
fprintf('\n[5/6] Checking simulation logs...\n');
logNames = simOut.who;
assert(all(ismember({'mb_px','mb_py'}, logNames)), ...
    'Simulation completed but mb_px/mb_py signals are missing. Available: %s', ...
    strjoin(logNames, ', '));
px = simOut.get('mb_px');
py = simOut.get('mb_py');
assert(isa(px, 'timeseries') && isa(py, 'timeseries'), ...
    'Expected mb_px and mb_py to be timeseries logs.');

fprintf('Recorded initial XY: (%.3f, %.3f) m\n', px.Data(1), py.Data(1));
fprintf('Requested initial XY: (%.3f, %.3f) m\n', startPose(1), startPose(2));
fprintf('Recorded final XY: (%.3f, %.3f) m\n', px.Data(end), py.Data(end));
fprintf('Distance to configured goal: %.3f m\n', ...
    hypot(px.Data(end)-goalX, py.Data(end)-goalY));
if hypot(px.Data(1)-startPose(1), py.Data(1)-startPose(2)) > 0.10
    warning('RoughTerrainWorkflow:InitialPoseMismatch', ...
        'Actual rover start differs from scenario start by more than 0.10 m.');
end

if any(strcmp(logNames, 'onlineReplanEventLog'))
    eventTS = simOut.get('onlineReplanEventLog');
    eventData = double(eventTS.Data(:));
    eventTime = double(eventTS.Time(:));
    eventRise = find(diff([0; eventData > 0.5]) == 1);
    fprintf('Replanning events detected: %d\n', numel(eventRise));
    for k = 1:numel(eventRise)
        fprintf('  Event %d at t = %.3f s\n', k, eventTime(eventRise(k)));
    end
else
    warning('RoughTerrainWorkflow:MissingEventLog', ...
        'onlineReplanEventLog was not returned by the simulation.');
end

if any(strcmp(logNames, 'sensorDetectedLog'))
    sensorTS = simOut.get('sensorDetectedLog');
    fprintf('Sensor-log values: ');
    disp(unique(sensorTS.Data(:))');
end

%% 6. Launch the multi-dynamic viewer using this run's output
fprintf('\n[6/6] Launching elevation-colored rover viewer...\n');
% Close only earlier figures created by this viewer to avoid confusing stale playback.
oldViewerFigs = findall(0, 'Type', 'figure', 'Name', 'Elevation-Colored Terrain + Rover Viewer');
if ~isempty(oldViewerFigs)
    delete(oldViewerFigs);
end
viewerFigure = launchElevationTerrainRoverViewerMultiDynamic(); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' WORKFLOW COMPLETE\n');
fprintf('============================================================\n');
fprintf('Test model saved: %s\n', testModelFile);
fprintf('Simulation output available as simOut and simOutScenarioWorkflow.\n');
fprintf('Original checkpoint left unchanged: %s.slx\n', sourceModelName);
fprintf(['Note: only obstacle 1 has physical Simscape collision geometry; ' ...
    'other obstacles are virtual for detection/planning and are rendered in the viewer.\n']);
