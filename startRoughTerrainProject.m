%% startRoughTerrainProject
% Reproducible entry point for the autonomous rough-terrain rover project.
% Run from a fresh MATLAB session after placing this file in the project root.
% This script prepares the workspace and loads/updates the active model; it
% does not start the simulation and does not save changes to the SLX file.

projectDir = fileparts(mfilename('fullpath'));
assert(isfolder(projectDir), 'Project folder could not be resolved.');
addpath(projectDir);
cd(projectDir);

setupFile = fullfile(projectDir, 'setupOffRoadProject.m');
plannerFunction = fullfile(projectDir, 'onlineReplanFromPose.m');
scenarioFunction = fullfile(projectDir, 'defaultRoverScenario.m');
modelFile = fullfile(projectDir, ...
    'OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx');
assert(isfile(setupFile), 'Missing setupOffRoadProject.m.');
assert(isfile(plannerFunction), 'Missing onlineReplanFromPose.m.');
assert(isfile(scenarioFunction), 'Missing defaultRoverScenario.m.');
assert(isfile(modelFile), 'Missing active online-replanning model.');

fprintf('\n=== ROUGH-TERRAIN ROVER PROJECT STARTUP ===\n');
fprintf('Project root: %s\n', projectDir);

% Rebuild terrain, slope costs, graph and planner from source definitions.
run(setupFile);

% Canonical scenario configuration. Legacy variables are derived from this
% struct for compatibility with existing Simulink Constant blocks/functions.
scenarioConfig = defaultRoverScenario();
scenarioName = scenarioConfig.name;
startPose = scenarioConfig.startPose;
goalPose = scenarioConfig.goalPose;
goalXY = goalPose(1:2);
goalX = goalXY(1);
goalY = goalXY(2);

allObstacles = scenarioConfig.obstacles(:);
staticMask = ~[allObstacles.isDynamic];
dynamicMask = [allObstacles.isDynamic];
staticObstacles = allObstacles(staticMask);
dynamicObstacles = allObstacles(dynamicMask);
assert(numel(dynamicObstacles) == 1, ...
    'Current model startup expects exactly one time-triggered dynamic obstacle.');

obstacleXs = arrayfun(@(o) o.centerXY(1), staticObstacles)';
obstacleYs = arrayfun(@(o) o.centerXY(2), staticObstacles)';
obstacle3X = dynamicObstacles(1).centerXY(1);
obstacle3Y = dynamicObstacles(1).centerXY(2);
obstacleSpawnTime = dynamicObstacles(1).spawnTime;

% Current Simscape model has one hard-coded 1 x 1 x 0.5 m physical brick
% at obstacle 1. Enforce agreement until model geometry is parameterized.
modelBrickDimensions = [1.0 1.0 0.5];
assert(all(abs(staticObstacles(1).dimensions - modelBrickDimensions) < 1e-9), ...
    ['Obstacle 1 dimensions in scenarioConfig differ from the current ', ...
     'Simscape Brick Solid (1 x 1 x 0.5 m). Update the model geometry ', ...
     'before changing obstacle 1 dimensions.']);
obstacleLength = staticObstacles(1).dimensions(1);
obstacleWidth = staticObstacles(1).dimensions(2);
obstacleHeight = staticObstacles(1).dimensions(3);
obstacleClearance = scenarioConfig.planning.obstacleClearance;
maxWaypoints = scenarioConfig.planning.maxWaypoints;

% The model's Rigid Transform uses Z translation = zGround - 0.15.
% Calculate the center so the 0.5 m brick bottom clears the highest terrain
% sample within its 1 x 1 m footprint by 5 mm.
underFirstObstacle = abs(X - obstacleXs(1)) <= obstacleLength/2 & ...
                     abs(Y - obstacleYs(1)) <= obstacleWidth/2;
assert(any(underFirstObstacle(:)), ...
    'Could not find terrain samples under physical obstacle 1.');
maxTerrainZUnderObstacle = max(Z_multibody(underFirstObstacle));
obstacleCenterZ = maxTerrainZUnderObstacle + 0.005 + obstacleHeight/2;
zGround = obstacleCenterZ + 0.15;

% Generate a fresh fixed-size initial route around the two static obstacles.
% Dynamic obstacle 3 is introduced to online replanning at its spawn time.
[initialRoutePadded, initialWaypointCount, initialClearances, initialRouteLength] = ...
    onlineReplanFromPose(startPose(1:2), goalXY, ...
    obstacleXs, obstacleYs, obstacleClearance, maxWaypoints);

originalPath = initialRoutePadded(1:initialWaypointCount, :);
originalPathPadded = initialRoutePadded;
% The switch input needs a valid fixed-size matrix before the first replan.
% The runtime OnlineReplanManager is responsible for replanning during sim.
replannedPathPadded = originalPathPadded;
waypoints = originalPath;
newPath = originalPath;

% Load only the selected active model; don't load legacy experiment variants.
modelName = 'OffRoadMultibodyStage1_OnlineReplanCheckpoint';
if ~bdIsLoaded(modelName)
    load_system(modelFile);
end

% Validate that key model blocks consume these shared variables.
expectedBlockValues = {
    'ObstacleX',         'obstacleXs'
    'ObstacleY',         'obstacleYs'
    'Obstacle3X',        'obstacle3X'
    'Obstacle3Y',        'obstacle3Y'
    'ObstacleSpawnTime', 'obstacleSpawnTime'
    'PathWaypoints',     'originalPathPadded'
    'goalX',             'goalX'
    'goalY',             'goalY'
};
for k = 1:size(expectedBlockValues,1)
    blockPath = [modelName '/' expectedBlockValues{k,1}];
    actualValue = strtrim(get_param(blockPath, 'Value'));
    expectedValue = expectedBlockValues{k,2};
    assert(strcmp(actualValue, expectedValue), ...
        'Unexpected Constant block configuration at %s (found "%s", expected "%s").', ...
        blockPath, actualValue, expectedValue);
end

% Update only in memory. No save_system call is made here.
set_param(modelName, 'SimulationCommand', 'update');

fprintf('\n=== PROJECT READY ===\n');
fprintf('Scenario: %s\n', scenarioName);
fprintf('Active model: %s.slx\n', modelName);
fprintf('Start XY: (%.2f, %.2f) m\n', startPose(1), startPose(2));
fprintf('Goal XY: (%.2f, %.2f) m\n', goalX, goalY);
fprintf('Static obstacles: %d\n', numel(obstacleXs));
fprintf('Dynamic obstacle: (%.2f, %.2f) m at t = %.2f s\n', ...
    obstacle3X, obstacle3Y, obstacleSpawnTime);
fprintf('Initial route: %d waypoints, %.3f m\n', ...
    initialWaypointCount, initialRouteLength);
fprintf('Physical obstacle center Z: %.4f m\n', obstacleCenterZ);
fprintf('Model updated successfully. Simulation has NOT been started.\n');
fprintf('\nNext commands:\n');
fprintf('  simOut = sim(''%s'', ''StopTime'', ''45'');\n', modelName);
fprintf('  launchElevationTerrainRoverViewer\n');
