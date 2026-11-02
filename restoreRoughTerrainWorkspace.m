%% restoreRoughTerrainWorkspace.m
% Rebuild required terrain/planner variables after MATLAB or the project was closed.
% Restores the user-edited scenario from scenarioConfig.mat, then derives the
% legacy workspace variables required by the scenario-integration/physical-test
% models. Does NOT run a simulation, modify/save a Simulink model, or overwrite
% scenarioConfig.mat.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
if ~strcmpi(pwd, projectRoot)
    cd(projectRoot);
end
addpath(projectRoot);

setupFile = fullfile(projectRoot, 'setupOffRoadProject.m');
scenarioFile = fullfile(projectRoot, 'scenarioConfig.mat');
assert(isfile(setupFile), 'Missing setup script: %s', setupFile);
assert(isfile(scenarioFile), 'Missing saved scenario: %s', scenarioFile);
assert(isfile(fullfile(projectRoot, 'onlineReplanFromPoseGeometryAware.m')) || ...
       exist('onlineReplanFromPoseGeometryAware', 'file') == 2, ...
       'Missing onlineReplanFromPoseGeometryAware.m.');

fprintf('\n============================================================\n');
fprintf(' RESTORE ROUGH-TERRAIN WORKSPACE\n');
fprintf('============================================================\n');
fprintf('Project root: %s\n', projectRoot);

% Rebuild terrain, planner graph, vehicle footprint and terrain variables.
% setupOffRoadProject does not load/update Simulink models or run a simulation.
run(setupFile);

% Restore the user's saved scenario AFTER setup, which initializes default poses.
savedScenarioData = load(scenarioFile, 'scenarioConfig');
assert(isfield(savedScenarioData, 'scenarioConfig') && ...
       isstruct(savedScenarioData.scenarioConfig), ...
       'scenarioConfig.mat does not contain a valid scenarioConfig struct.');
scenarioConfig = savedScenarioData.scenarioConfig;

% Normalize obstacle ordering by ID and validate the expected three-box scope.
obs = scenarioConfig.obstacles(:);
assert(numel(obs) == 3, ...
    'This current model workflow expects exactly three obstacles. Found %d.', numel(obs));
[~, obstacleOrder] = sort([obs.id]);
obs = obs(obstacleOrder);
assert(all(arrayfun(@(o) strcmpi(o.shape, 'box'), obs)), ...
    'Current scenario integration supports box obstacles only.');

% Saved scenario-derived variables used by model Constant blocks and MATLAB Function charts.
scenarioName = scenarioConfig.name;
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
obstacleSpawnTime = [obs.spawnTime]; % NaN for static; configured seconds for dynamic.
obstacle3X = obstacleXs(3); % Legacy block/port compatibility.
obstacle3Y = obstacleYs(3);

staticMask = ~[obs.isDynamic];
staticObs = obs(staticMask);
staticCenters = vertcat(staticObs.centerXY);
staticXs = staticCenters(:,1).';
staticYs = staticCenters(:,2).';
dynamicObs = obs(~staticMask);

% Recalculate the fixed-size initial path around the static obstacles only.
[originalPathPadded, initialWaypointCount, initialClearances, initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware( ...
        startPose(1:2), goalXY, staticXs, staticYs, ...
        obstacleClearance, maxWaypoints);
originalPath = originalPathPadded(1:initialWaypointCount, :);
replannedPathPadded = originalPathPadded;
waypoints = originalPath;
newPath = originalPath;

% Legacy geometry variables used by the first physical obstacle in existing test copies.
obstacleLength = double(obs(1).dimensions(1));
obstacleWidth = double(obs(1).dimensions(2));
obstacleHeight = double(obs(1).dimensions(3));
assert(isequal(size(X), size(Y), size(Z_multibody)), ...
    'Terrain arrays X, Y and Z_multibody have inconsistent sizes.');
underFirstObstacle = abs(X - obstacleXs(1)) <= obstacleLength/2 & ...
                     abs(Y - obstacleYs(1)) <= obstacleWidth/2;
assert(any(underFirstObstacle(:)), ...
    'Could not find terrain samples under obstacle 1.');
maxTerrainZUnderObstacle = max(Z_multibody(underFirstObstacle));
obstacleCenterZ = maxTerrainZUnderObstacle + 0.005 + obstacleHeight/2;
zGround = obstacleCenterZ + 0.15;

% Summary and sanity checks.
assert(all(isfinite(startPose(1:2))) && all(isfinite(goalXY)), ...
    'Saved scenario start/goal coordinates must be finite.');
assert(exist('plannerFootprint', 'var') == 1 && ...
       exist('vehicleClearanceRadius', 'var') == 1, ...
       'Planner workspace was not initialized correctly.');

fprintf('\nWorkspace restored successfully.\n');
fprintf('Scenario: %s\n', scenarioName);
fprintf('Start XY: (%.3f, %.3f) m; yaw %.4f rad\n', ...
    startPose(1), startPose(2), startPose(3));
fprintf('Goal XY:  (%.3f, %.3f) m\n', goalX, goalY);
fprintf('Static obstacles: %d; dynamic obstacles: %d\n', ...
    sum(~[obs.isDynamic]), sum([obs.isDynamic]));
fprintf('Obstacle X array: '); disp(obstacleXs);
fprintf('Obstacle Y array: '); disp(obstacleYs);
fprintf('Spawn-time array (NaN = static): '); disp(obstacleSpawnTime);
fprintf('Initial route: %.3f m; %d waypoints (padded %d x %d)\n', ...
    initialRouteLength, initialWaypointCount, ...
    size(originalPathPadded,1), size(originalPathPadded,2));
fprintf('Physical obstacle 1: size [%.3f %.3f %.3f] m; center Z %.4f m\n', ...
    obstacleLength, obstacleWidth, obstacleHeight, obstacleCenterZ);
fprintf('\nNo simulation was started. No Simulink model was modified or saved.\n');
fprintf('NOTE: previous SimulationOutput variables must be regenerated by rerunning their simulation.\n');
