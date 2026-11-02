%% BUILD A SEPARATE PHYSICAL MODEL FOR THE SIX-OBSTACLE CUSTOM SCENARIO
% Build-only script. It does NOT simulate the model.
% It never modifies the V2 model or the original checkpoint.
%
% Place this file in the project root and run:
%   buildCustomPhysicalScenarioModel
%
% Physical bodies: obstacle IDs 1, 2, 3.
% Virtual sensing/planning obstacles: IDs 4, 5, 6.
%
% Vertical placement uses the 95th percentile of terrain elevation under each
% box footprint plus 5 mm. This is a provisional flat-box placement rule,
% NOT terrain contact. Small high-elevation regions may still intersect boxes.
%
% Obstacle 3 spawn: starts below its target bottom plane and is released using
% the existing guided Prismatic Joint mode-transition approximation. This
% requires short-run validation before relying on the nominal 7 s activation.

%% Project paths and safety checks
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
cd(projectRoot);
addpath(projectRoot);

sourceModelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2';
customModelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
sourceModelFile = fullfile(projectRoot, [sourceModelName '.slx']);
customModelFile = fullfile(projectRoot, [customModelName '.slx']);
scenarioFile = fullfile(projectRoot, 'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot, 'setupOffRoadProject.m');

assert(isfile(sourceModelFile), 'Validated V2 model not found: %s', sourceModelFile);
assert(isfile(scenarioFile), 'Custom 80 s scenario not found: %s', scenarioFile);
assert(isfile(setupFile), 'Workspace setup script not found: %s', setupFile);
assert(~isfile(customModelFile), ...
    'Custom model file already exists. Do not overwrite it automatically: %s', customModelFile);
assert(~bdIsLoaded(customModelName), ...
    'Custom model is already loaded. Close it before building: %s', customModelName);

fprintf('\n============================================================\n');
fprintf(' BUILD CUSTOM SIX-OBSTACLE PHYSICAL MODEL (BUILD ONLY)\n');
fprintf('============================================================\n');

%% Restore the project's terrain and navigation dependencies
fprintf('\n[1/7] Restoring project workspace...\n');
run(setupFile);

assert(exist('X','var') == 1 && exist('Y','var') == 1 && ...
       exist('Z_multibody','var') == 1, ...
       'Terrain variables X, Y, and Z_multibody were not restored.');
assert(isequal(size(X), size(Y), size(Z_multibody)), ...
       'Terrain grids X, Y, and Z_multibody do not have matching dimensions.');

%% Load and validate the exact custom scenario
scenarioData = load(scenarioFile, 'scenarioConfig');
assert(isfield(scenarioData, 'scenarioConfig'), ...
    'Scenario MAT file does not contain scenarioConfig.');
scenarioConfig = scenarioData.scenarioConfig;

obs = scenarioConfig.obstacles(:);
[~, order] = sort([obs.id]);
obs = obs(order);
ids = [obs.id];

assert(numel(obs) == 6, 'Expected exactly six configured obstacles.');
assert(isequal(ids, 1:6), 'Expected unique obstacle IDs 1 through 6.');
assert(all(arrayfun(@(o) strcmpi(o.shape, 'box'), obs)), ...
    'This model builder supports box obstacles only.');
assert(isequal(logical([obs.isDynamic]), logical([false false true false true false])), ...
    'Expected roles: IDs 1,2 static; 3 dynamic; 4 static; 5 dynamic; 6 static.');
assert(abs(obs(3).spawnTime - 7) < 1e-9 && abs(obs(5).spawnTime - 15) < 1e-9, ...
    'Expected obstacle 3 spawn at 7 s and obstacle 5 activation at 15 s.');
assert(isfield(scenarioConfig, 'simulation') && ...
       isfield(scenarioConfig.simulation, 'stopTime') && ...
       scenarioConfig.simulation.stopTime >= 80, ...
       'The custom physical scenario must have a simulation horizon of at least 80 s.');
assert(all(isfinite([scenarioConfig.startPose(1:2), scenarioConfig.goalPose(1:2)])), ...
    'Scenario start/goal coordinates must be finite.');

% Check all obstacle footprints lie within the terrain coordinate domain.
xMin = min(X(:)); xMax = max(X(:));
yMin = min(Y(:)); yMax = max(Y(:));
for k = 1:numel(obs)
    cx = obs(k).centerXY(1); cy = obs(k).centerXY(2);
    dx = obs(k).dimensions(1); dy = obs(k).dimensions(2);
    assert(cx-dx/2 >= xMin && cx+dx/2 <= xMax && ...
           cy-dy/2 >= yMin && cy+dy/2 <= yMax, ...
           'Obstacle %d footprint extends outside the terrain domain.', obs(k).id);
end

fprintf('Scenario: %s\n', scenarioConfig.name);
fprintf('Obstacles: %d | Start: [%.3f %.3f] | Goal: [%.3f %.3f] | StopTime: %.1f s\n', ...
    numel(obs), scenarioConfig.startPose(1), scenarioConfig.startPose(2), ...
    scenarioConfig.goalPose(1), scenarioConfig.goalPose(2), ...
    scenarioConfig.simulation.stopTime);

%% Derive conservative/provisional Z placement from terrain footprint percentiles
fprintf('\n[2/7] Calculating terrain-relative obstacle placement...\n');
terrainBottom = nan(1,3);
terrainP95 = nan(1,3);
for id = 1:3
    o = obs(id);
    mask = abs(X-o.centerXY(1)) <= o.dimensions(1)/2 & ...
           abs(Y-o.centerXY(2)) <= o.dimensions(2)/2 & ...
           isfinite(Z_multibody);
    zValues = sort(Z_multibody(mask));
    assert(~isempty(zValues), 'No valid terrain samples under obstacle %d.', id);
    p = 1 + 0.95*(numel(zValues)-1);
    lo = floor(p); hi = ceil(p);
    z95 = zValues(lo) + (p-lo)*(zValues(hi)-zValues(lo));
    terrainP95(id) = z95;
    terrainBottom(id) = z95 + 0.005;
    fprintf('Obstacle %d: terrain P95=%.4f m; provisional bottom=%.4f m; box centre Z=%.4f m\n', ...
        id, z95, terrainBottom(id), terrainBottom(id)+o.dimensions(3)/2);
end

%% Create an isolated copy of the validated V2 model
fprintf('\n[3/7] Copying V2 model to a separate custom model...\n');
[copyOK, copyMessage] = copyfile(sourceModelFile, customModelFile);
assert(copyOK, 'Could not copy V2 model: %s', copyMessage);
load_system(customModelFile);

try
    %% Synchronize base-workspace variables and initial route
    fprintf('\n[4/7] Preparing six-obstacle planner/sensor inputs...\n');
    assignin('base', 'scenarioConfig', scenarioConfig);

    startPoseWorkflow = double(scenarioConfig.startPose(:).');
    goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
    goalX = goalPoseWorkflow(1);
    goalY = goalPoseWorkflow(2);
    goalXY = goalPoseWorkflow(1:2);

    allCenters = vertcat(obs.centerXY);
    obstacleXs = allCenters(:,1).';
    obstacleYs = allCenters(:,2).';
    obstacleSpawnTime = [obs.spawnTime];
    obstacleXs = [obstacleXs nan(1,4)];
    obstacleYs = [obstacleYs nan(1,4)];
    obstacleSpawnTime = [obstacleSpawnTime nan(1,4)];
    obstacle3X = obs(3).centerXY(1);
    obstacle3Y = obs(3).centerXY(2);
    obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
    maxWaypoints = double(scenarioConfig.planning.maxWaypoints);

    assert(exist('onlineReplanFromPoseGeometryAware','file') == 2, ...
        'onlineReplanFromPoseGeometryAware.m is not on the MATLAB path.');
    staticObs = obs(~[obs.isDynamic]);
    staticCenters = vertcat(staticObs.centerXY);
    staticXs = staticCenters(:,1).';
    staticYs = staticCenters(:,2).';
    [originalPathPadded, initialWaypointCount, ~, initialRouteLength] = ...
        onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2), goalXY, ...
        staticXs, staticYs, obstacleClearance, maxWaypoints);
    assert(initialWaypointCount >= 2, 'Initial route contains fewer than two waypoints.');
    originalPath = originalPathPadded(1:initialWaypointCount,:); %#ok<NASGU>
    waypoints = originalPath; %#ok<NASGU>
    newPath = originalPath; %#ok<NASGU>

    constants = {
        'ObstacleX', 'obstacleXs';
        'ObstacleY', 'obstacleYs';
        'Obstacle3X', 'obstacle3X';
        'Obstacle3Y', 'obstacle3Y';
        'ObstacleSpawnTime', 'obstacleSpawnTime';
        'PathWaypoints', 'originalPathPadded';
        'goalX', 'goalX';
        'goalY', 'goalY'};
    for k = 1:size(constants,1)
        blockPath = [customModelName '/' constants{k,1}];
        assert(getSimulinkBlockHandle(blockPath) ~= -1, ...
            'Required model constant block missing: %s', blockPath);
        set_param(blockPath, 'Value', constants{k,2});
    end

    %% Configure the three physical obstacle solids and their placement frames
    fprintf('\n[5/7] Configuring physical obstacle bodies...\n');
    placementBlocks = { ...
        [customModelName '/Obstacle1Placement'], ...
        [customModelName '/Obstacle2Placement'], ...
        [customModelName '/Obstacle3Placement']};
    for id = 1:3
        brick = [customModelName '/PhysicalObstacle' num2str(id)];
        placement = placementBlocks{id};
        dims = double(obs(id).dimensions(:).');
        center = double(obs(id).centerXY(:).');
        centerZ = terrainBottom(id) + dims(3)/2;

        assert(getSimulinkBlockHandle(brick) ~= -1, ...
            'Physical obstacle solid missing: %s', brick);
        assert(getSimulinkBlockHandle(placement) ~= -1, ...
            'Obstacle placement frame missing: %s', placement);

        set_param(brick, 'BrickDimensions', mat2str(dims,12));
        set_param(placement, ...
            'TranslationMethod', 'Cartesian', ...
            'TranslationCartesianOffset', mat2str([center centerZ],12), ...
            'TranslationCartesianOffsetUnits', 'm');

        fprintf('Physical obstacle %d: centre=[%.3f %.3f %.3f] m; dimensions=[%.3f %.3f %.3f] m\n', ...
            id, center(1), center(2), centerZ, dims(1), dims(2), dims(3));
    end

    %% Obstacle 2 is static: lock its prismatic joint at q=0 for the whole run
    joint2 = [customModelName '/Obstacle2SpawnJoint'];
    step2 = [customModelName '/Obstacle2SpawnModeStep'];
    set_param(joint2, ...
        'JointMode', 'ProvidedByInput', ...
        'MotionActuationMode', 'ComputedMotion', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', '0', ...
        'PositionTargetValueUnits', 'm');
    % In this model, mode 1 is Locked and mode 0 is Normal.
    set_param(step2, 'Time', '0', 'Before', '1', 'After', '1');
    fprintf('Obstacle 2: static; spawn joint held in Locked mode at q=0.\n');

    %% Obstacle 3 remains physical and dynamic, with a size-dependent hidden start
    joint3 = [customModelName '/Obstacle3SpawnJoint'];
    step3 = [customModelName '/Obstacle3SpawnModeStep'];
    embedDepth = 0.05;
    g = 9.81;
    h3 = double(obs(3).dimensions(3));
    qInitial3 = -(h3 + embedDepth);
    travelTime3 = sqrt(2*abs(qInitial3)/g);
    releaseTime3 = double(obs(3).spawnTime) - travelTime3;
    assert(releaseTime3 > 0, ...
        'Obstacle 3 spawn time is too early for the calculated release lead-in.');

    set_param(joint3, ...
        'JointMode', 'ProvidedByInput', ...
        'MotionActuationMode', 'ComputedMotion', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.12g',qInitial3), ...
        'PositionTargetValueUnits', 'm', ...
        'LowerLimitSpecify', 'off', ...
        'UpperLimitSpecify', 'on', ...
        'UpperLimitBound', '0', ...
        'UpperLimitBoundUnits', 'm', ...
        'UpperLimitStiffness', '1e5', ...
        'UpperLimitDamping', '1500', ...
        'UpperLimitTransitionRegionWidth', '0.005');
    set_param(step3, 'Time', sprintf('%.12g',releaseTime3), ...
        'Before', '1', 'After', '0');

    fprintf(['Obstacle 3: configured spawn=%.3f s; initial q=%.4f m; ', ...
        'release command=%.4f s; nominal travel time=%.4f s.\n'], ...
        obs(3).spawnTime, qInitial3, releaseTime3, travelTime3);

    %% Expand sensing and replanning charts to the 10-obstacle fixed capacity
    fprintf('\n[6/7] Configuring sensor and online-replanning charts...\n');
    sensorPath = [customModelName '/ObstacleSensorLogic'];
    sensorChart = findChartByPath(sensorPath);
    sensorSource = char(sensorChart.Script);
    sensorPattern = '(?m)^[ \t]*sensorRange\s*=\s*[0-9.]+[ \t]*;';
    assert(numel(regexp(sensorSource,sensorPattern,'match')) == 1, ...
        'Expected exactly one sensorRange assignment in %s.', sensorPath);
    sensorChart.Script = regexprep(sensorSource,sensorPattern, ...
        'sensorRange = 18.0;','once');

    environmentPath = [customModelName '/DynamicObstacleEnvironment'];
    environmentChart = findChartByPath(environmentPath);
    environmentChart.Script = sprintf([ ...
        'function [environmentXs, environmentYs] = ...\n' ...
        '    fcn(t, knownXs, knownYs, obstacle3X, obstacle3Y, spawnTime)\n' ...
        '%% Fixed capacity: up to 10 scenario obstacles.\n' ...
        '%% NaN spawnTime means static from t=0; finite spawnTime activates by time t.\n' ...
        'maxScenarioObstacles = 10;\n' ...
        'environmentXs = nan(1,maxScenarioObstacles);\n' ...
        'environmentYs = nan(1,maxScenarioObstacles);\n' ...
        'for k = 1:maxScenarioObstacles\n' ...
        '    if numel(knownXs) >= k && numel(knownYs) >= k && numel(spawnTime) >= k\n' ...
        '        tk = spawnTime(k);\n' ...
        '        if isnan(tk) || t >= tk\n' ...
        '            environmentXs(k) = knownXs(k);\n' ...
        '            environmentYs(k) = knownYs(k);\n' ...
        '        end\n' ...
        '    end\n' ...
        'end\n' ...
        'end\n']);

    managerPath = [customModelName '/OnlineReplanManager'];
    managerChart = findChartByPath(managerPath);
    managerSource = char(managerChart.Script);
    managerPattern = '(?m)^[ \t]*maxObstacles\s*=\s*\d+\s*;';
    assert(numel(regexp(managerSource,managerPattern,'match')) == 1, ...
        'Expected exactly one maxObstacles assignment in %s.', managerPath);
    managerChart.Script = regexprep(managerSource,managerPattern, ...
        'maxObstacles = 10;','once');

    %% Synchronize the rover start pose, goal and model stop time
    fprintf('\n[7/7] Synchronizing rover pose and updating model...\n');
    chassisJoint = [customModelName '/6-DOF Joint'];
    oldX = str2double(get_param(chassisJoint,'PxPositionTargetValue'));
    oldY = str2double(get_param(chassisJoint,'PyPositionTargetValue'));
    oldZ = str2double(get_param(chassisJoint,'PzPositionTargetValue'));
    oldQuaternion = get_param(chassisJoint,'SphPositionTargetQuaternion');
    zOldTerrain = interp2(X(1,:),Y(:,1),Z_multibody,oldX,oldY,'linear');
    zNewTerrain = interp2(X(1,:),Y(:,1),Z_multibody, ...
        startPoseWorkflow(1),startPoseWorkflow(2),'linear');
    assert(all(isfinite([oldX oldY oldZ zOldTerrain zNewTerrain])), ...
        'Could not calculate terrain-adjusted rover initial pose.');
    newZ = oldZ + (zNewTerrain-zOldTerrain);
    yaw = startPoseWorkflow(3);
    newQuaternion = [cos(yaw/2) 0 0 sin(yaw/2)];

    set_param(chassisJoint, ...
        'PxPositionTargetValue',sprintf('%.15g',startPoseWorkflow(1)), ...
        'PyPositionTargetValue',sprintf('%.15g',startPoseWorkflow(2)), ...
        'PzPositionTargetValue',sprintf('%.15g',newZ), ...
        'SphPositionTargetQuaternion',mat2str(newQuaternion,15));

    set_param(customModelName,'StopTime', ...
        sprintf('%.15g',double(scenarioConfig.simulation.stopTime)));

    set_param(customModelName,'SimulationCommand','update');
    save_system(customModelName,customModelFile);

    fprintf('\n============================================================\n');
    fprintf(' CUSTOM PHYSICAL MODEL BUILT SUCCESSFULLY\n');
    fprintf('============================================================\n');
    fprintf('Model: %s\n',customModelFile);
    fprintf('StopTime: %s s\n',get_param(customModelName,'StopTime'));
    fprintf('Initial route length: %.3f m (%d waypoints)\n', ...
        initialRouteLength,initialWaypointCount);
    fprintf('Physical bodies: IDs 1, 2, 3.\n');
    fprintf('Virtual-only obstacles: IDs 4, 5, 6.\n');
    fprintf('Obstacle 2: static locked joint.\n');
    fprintf('Obstacle 3: dynamic joint-mode release configured for 7 s.\n');
    fprintf('Sensor range: 18 m; sensor/replanner capacity: 10 slots.\n');
    fprintf('Simulation has NOT been started.\n');
    fprintf('\nPlacement caveat: P95 placement is provisional; terrain contact is not modelled.\n');
    fprintf('Spawn caveat: verify obstacle 3 motion in a short simulation before full run.\n');
    fprintf('Original V2 file and checkpoint were not edited.\n');

catch ME
    fprintf(2,'\nCustom model build failed: %s\n',ME.message);
    if bdIsLoaded(customModelName)
        close_system(customModelName,0);
    end
    % Keep the copied file as an untouched V2-derived recovery point.
    fprintf(2,'The custom file on disk was not saved with partial edits: %s\n',customModelFile);
    rethrow(ME);
end

%% Local helper: locate a Stateflow MATLAB Function chart by full path
function chart = findChartByPath(targetPath)
    root = sfroot;
    charts = find(root,'-isa','Stateflow.EMChart');
    chart = [];
    for n = 1:numel(charts)
        if strcmp(charts(n).Path,targetPath)
            chart = charts(n);
            break;
        end
    end
    assert(~isempty(chart),'Stateflow MATLAB Function chart not found: %s',targetPath);
end
