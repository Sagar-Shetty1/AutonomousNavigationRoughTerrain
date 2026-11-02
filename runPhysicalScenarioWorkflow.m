%% runPhysicalScenarioWorkflow.m
% One-entry-point runner for the separately calibrated physical-spawn model.
% Restores terrain/planner and saved scenario; creates the v2 physical test copy
% if missing; applies validated 18 m sensor + calibrated world-Z=0 spawn settings;
% synchronizes start/goal/path/obstacle inputs; simulates; reports logs; opens viewer.
%
% Safety/scope:
% - Never changes the original OnlineReplanCheckpoint model.
% - Never changes the ScenarioIntegrationTest model.
% - Uses only OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2.slx.
% - World-Z spawn calibration is valid only for current obstacle-2/3 centers and
%   dimensions, guarded below. If those change, this runner stops before simulation.
% - Obstacles 1-3 are the calibrated physical core. Obstacles 4-10 are virtual
%   sensor/planner/viewer obstacles only; they have no separate Simscape bodies.
% - Obstacle 2/3 spawning uses a guided joint-stop approximation, not terrain contact.

workflowRoot = fileparts(mfilename('fullpath'));
if isempty(workflowRoot), workflowRoot = pwd; end
if ~strcmpi(pwd, workflowRoot), cd(workflowRoot); end
addpath(workflowRoot);

savedScenarioFile = fullfile(workflowRoot, 'scenarioConfig.mat');
setupScript = fullfile(workflowRoot, 'setupOffRoadProject.m');
builderScript = fullfile(workflowRoot, 'createPhysicalScenarioSpawnTestModel_v2.m');
sourceModelName = 'OffRoadMultibodyStage1_ScenarioIntegrationTest';
sourceModelFile = fullfile(workflowRoot, [sourceModelName '.slx']);
physicalModelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2';
physicalModelFile = fullfile(workflowRoot, [physicalModelName '.slx']);
viewerScript = fullfile(workflowRoot, 'launchElevationTerrainRoverViewerMultiDynamic.m');

assert(isfile(savedScenarioFile), 'Saved scenario file missing: %s', savedScenarioFile);
assert(isfile(setupScript), 'Workspace setup missing: %s', setupScript);
assert(isfile(viewerScript), 'Multi-dynamic viewer missing: %s', viewerScript);

savedDataWorkflow = load(savedScenarioFile, 'scenarioConfig');
assert(isfield(savedDataWorkflow, 'scenarioConfig'), ...
    'scenarioConfig.mat does not contain scenarioConfig.');
savedScenarioWorkflow = savedDataWorkflow.scenarioConfig;

fprintf('\n============================================================\n');
fprintf(' PHYSICAL ROUGH-TERRAIN SCENARIO WORKFLOW\n');
fprintf('============================================================\n');
fprintf('Project root: %s\n', workflowRoot);
fprintf('Scenario file: %s\n', savedScenarioFile);

%% 1. Rebuild workspace and restore saved user scenario
fprintf('\n[1/6] Rebuilding terrain/planner workspace...\n');
run(setupScript);
scenarioConfig = savedScenarioWorkflow;

obsWorkflow = scenarioConfig.obstacles(:);
assert(numel(obsWorkflow) >= 3, ...
    'Physical workflow requires the calibrated core obstacles with IDs 1, 2, and 3.');
assert(numel(obsWorkflow) <= 10, ...
    'The current sensor/replanning model supports at most 10 configured obstacles.');
[~, ordWorkflow] = sort([obsWorkflow.id]);
obsWorkflow = obsWorkflow(ordWorkflow);
idsWorkflow = [obsWorkflow.id];
assert(numel(unique(idsWorkflow)) == numel(idsWorkflow) && ...
    all(isfinite(idsWorkflow)) && all(idsWorkflow >= 1) && ...
    all(idsWorkflow == fix(idsWorkflow)) && all(ismember([1 2 3], idsWorkflow)), ...
    'Obstacle IDs must be unique positive integers and include core IDs 1, 2, and 3.');
assert(isequal([obsWorkflow(1:3).id], [1 2 3]), ...
    'Obstacle IDs 1, 2, and 3 must be present as the calibrated physical core.');
assert(all(arrayfun(@(o) strcmpi(o.shape, 'box'), obsWorkflow)), ...
    'Physical workflow currently supports box obstacles only.');
assert(~obsWorkflow(1).isDynamic && obsWorkflow(2).isDynamic && obsWorkflow(3).isDynamic, ...
    'Expected obstacle 1 static and obstacles 2/3 dynamic.');
for oiWorkflow = 1:numel(obsWorkflow)
    if obsWorkflow(oiWorkflow).isDynamic
        assert(isfinite(obsWorkflow(oiWorkflow).spawnTime) && obsWorkflow(oiWorkflow).spawnTime >= 0, ...
            'Dynamic obstacle %d requires a finite nonnegative spawn time.', obsWorkflow(oiWorkflow).id);
    else
        assert(isnan(obsWorkflow(oiWorkflow).spawnTime), ...
            'Static obstacle %d should have spawnTime=NaN.', obsWorkflow(oiWorkflow).id);
    end
end
if numel(obsWorkflow) > 3
    extraIdsWorkflow = idsWorkflow(idsWorkflow >= 4);
    fprintf('Additional virtual-only obstacles: IDs %s (no separate Simscape collision bodies).\n', ...
        mat2str(extraIdsWorkflow));
end

% Fixed world-Z calibration guarded to the geometry whose motion was validated.
calibratedCentersWorkflow = [23.7161198288 22.8601997147; ...
                             16.7974322397 36.0556348074];
calibratedDimsWorkflow = [2 23 1; 4 4 0.5];
centersDynamicWorkflow = [obsWorkflow(2).centerXY; obsWorkflow(3).centerXY];
dimsDynamicWorkflow = [obsWorkflow(2).dimensions; obsWorkflow(3).dimensions];
assert(all(abs(centersDynamicWorkflow-calibratedCentersWorkflow) < 1e-3, 'all'), ...
    ['Physical Z calibration is only validated for current obstacle-2/3 centers. ', ...
     'Use runRoughTerrainWorkflow for virtual planning/viewer, or recalibrate physical spawning.']);
assert(all(abs(dimsDynamicWorkflow-calibratedDimsWorkflow) < 1e-9, 'all'), ...
    ['Physical Z calibration is only validated for obstacle 2=[2 23 1] and ', ...
     'obstacle 3=[4 4 0.5]. Recalibrate after resizing.']);
assert(all(scenarioConfig.startPose(1:2) >= [min(X(:)) min(Y(:))]) && ...
       all(scenarioConfig.startPose(1:2) <= [max(X(:)) max(Y(:))]), ...
    'Scenario start is outside terrain bounds.');
assert(all(scenarioConfig.goalPose(1:2) >= [min(X(:)) min(Y(:))]) && ...
       all(scenarioConfig.goalPose(1:2) <= [max(X(:)) max(Y(:))]), ...
    'Scenario goal is outside terrain bounds.');

%% 2. Create v2 physical test model if not yet built
if ~isfile(physicalModelFile)
    assert(isfile(sourceModelFile), ...
        'Source ScenarioIntegrationTest model missing. Run runRoughTerrainWorkflow first.');
    assert(isfile(builderScript), 'Physical model builder missing: %s', builderScript);
    fprintf('\n[2/6] Creating calibrated physical test model...\n');
    run(builderScript);
    % Builder rebuilds base workspace; restore config and all dependencies below.
    run(setupScript);
    scenarioConfig = savedScenarioWorkflow;
    obsWorkflow = scenarioConfig.obstacles(:);
    [~, ordWorkflow] = sort([obsWorkflow.id]);
    obsWorkflow = obsWorkflow(ordWorkflow);
else
    fprintf('\n[2/6] Reusing calibrated physical test model: %s\n', physicalModelFile);
end

if ~bdIsLoaded(physicalModelName)
    load_system(physicalModelFile);
end

% Backup the last on-disk model BEFORE applying scenario synchronization.
stampWorkflow = datestr(now, 'yyyymmdd_HHMMSS');
backupWorkflow = fullfile(workflowRoot, ...
    [physicalModelName '_preWorkflowSync_' stampWorkflow '.slx']);
assert(copyfile(physicalModelFile, backupWorkflow), ...
    'Could not back up physical test model: %s', physicalModelFile);
fprintf('Physical model backup: %s\n', backupWorkflow);

try
%% 3. Derive current scenario variables and synchronize the physical test model
fprintf('\n[3/6] Synchronizing model constants, physical geometry, and initial pose...\n');
startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalX = goalPoseWorkflow(1);
goalY = goalPoseWorkflow(2);
goalXY = goalPoseWorkflow(1:2);
allCentersWorkflow = vertcat(obsWorkflow.centerXY);
configuredObstacleCountWorkflow = numel(obsWorkflow);
maxVirtualObstacleSlotsWorkflow = 10;
obstacleXs = allCentersWorkflow(:,1).';
obstacleYs = allCentersWorkflow(:,2).';
obstacleSpawnTime = [obsWorkflow.spawnTime];
% Pad all environment input vectors to a fixed width for MATLAB Function code generation.
unusedSlotsWorkflow = maxVirtualObstacleSlotsWorkflow - configuredObstacleCountWorkflow;
obstacleXs = [obstacleXs nan(1, unusedSlotsWorkflow)];
obstacleYs = [obstacleYs nan(1, unusedSlotsWorkflow)];
obstacleSpawnTime = [obstacleSpawnTime nan(1, unusedSlotsWorkflow)];
obstacle3X = obsWorkflow(3).centerXY(1);
obstacle3Y = obsWorkflow(3).centerXY(2);
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);
obstacleLength = double(obsWorkflow(1).dimensions(1));
obstacleWidth = double(obsWorkflow(1).dimensions(2));
obstacleHeight = double(obsWorkflow(1).dimensions(3));

% Calculate static-obstacle initial route; dynamic obstacles are added by the runtime sensor.
staticObsWorkflow = obsWorkflow(~[obsWorkflow.isDynamic]);
staticCentersWorkflow = vertcat(staticObsWorkflow.centerXY);
staticXs = staticCentersWorkflow(:,1).';
staticYs = staticCentersWorkflow(:,2).';
[originalPathPadded, initialWaypointCountWorkflow, ~, initialRouteLengthWorkflow] = ...
    onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2), goalXY, ...
    staticXs, staticYs, obstacleClearance, maxWaypoints);
originalPath = originalPathPadded(1:initialWaypointCountWorkflow,:);
replannedPathPadded = originalPathPadded; %#ok<NASGU>
waypoints = originalPath; %#ok<NASGU>
newPath = originalPath; %#ok<NASGU>

% Ensure the model's constant blocks point at the shared workspace values.
expectedConstantsWorkflow = {
    'ObstacleX', 'obstacleXs';
    'ObstacleY', 'obstacleYs';
    'Obstacle3X', 'obstacle3X';
    'Obstacle3Y', 'obstacle3Y';
    'ObstacleSpawnTime', 'obstacleSpawnTime';
    'PathWaypoints', 'originalPathPadded';
    'goalX', 'goalX';
    'goalY', 'goalY'};
for k = 1:size(expectedConstantsWorkflow,1)
    blockPath = [physicalModelName '/' expectedConstantsWorkflow{k,1}];
    try
        get_param(blockPath, 'Handle');
    catch
        error('Required block missing: %s', blockPath);
    end
    set_param(blockPath, 'Value', expectedConstantsWorkflow{k,2});
end

% Synchronize the single existing physical static obstacle.
mask1 = abs(X - obstacleXs(1)) <= obstacleLength/2 & ...
        abs(Y - obstacleYs(1)) <= obstacleWidth/2;
assert(any(mask1(:)), 'No terrain samples under static obstacle 1.');
obstacleCenterZ = max(Z_multibody(mask1)) + 0.005 + obstacleHeight/2;
zGround = obstacleCenterZ + 0.15; %#ok<NASGU>
set_param([physicalModelName '/PhysicalObstacle1'], ...
    'BrickDimensions', '[obstacleLength obstacleWidth obstacleHeight]');

% Apply validated world-Z spawn settings. Calibrations are specific to the guarded geometry.
placementZOffsetsWorkflow = [0.061312 0.024525];
embedDepthWorkflow = 0.05;
contactMassesWorkflow = [25 10];
g = 9.81;
for n = 1:2
    id = n + 1;
    o = obsWorkflow(id);
    suffix = sprintf('%d', id);
    brick = [physicalModelName '/PhysicalObstacle' suffix];
    placement = [physicalModelName '/Obstacle' suffix 'Placement'];
    joint = [physicalModelName '/Obstacle' suffix 'SpawnJoint'];
    step = [physicalModelName '/Obstacle' suffix 'SpawnModeStep'];

    qInitial = -(o.dimensions(3)/2 + embedDepthWorkflow);
    releaseTime = o.spawnTime - sqrt(2*abs(qInitial)/g);
    assert(releaseTime > 0, 'Obstacle %d spawn time too early for calibrated release.', id);

    set_param(brick, 'BrickDimensions', mat2str(o.dimensions, 12));
    brickDP = get_param(brick, 'DialogParameters');
    if isfield(brickDP, 'BasedOnType'), set_param(brick, 'BasedOnType', 'Mass'); end
    if isfield(brickDP, 'Mass'), set_param(brick, 'Mass', sprintf('%.12g', contactMassesWorkflow(n))); end
    if isfield(brickDP, 'InertiaType'), set_param(brick, 'InertiaType', 'CalculateFromGeometry'); end

    set_param(placement, ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', mat2str([o.centerXY placementZOffsetsWorkflow(n)], 12), ...
        'TranslationCartesianOffsetUnits', 'm');

    set_param(joint, ...
        'JointMode', 'ProvidedByInput', ...
        'MotionActuationMode', 'ComputedMotion', ...
        'PositionTargetSpecify', 'on', ...
        'PositionTargetPriority', 'High', ...
        'PositionTargetValue', sprintf('%.12g', qInitial), ...
        'PositionTargetValueUnits', 'm', ...
        'LowerLimitSpecify', 'off', ...
        'UpperLimitSpecify', 'on', ...
        'UpperLimitBound', '0', ...
        'UpperLimitBoundUnits', 'm', ...
        'UpperLimitStiffness', '1e5', ...
        'UpperLimitDamping', '1500', ...
        'UpperLimitTransitionRegionWidth', '0.005', ...
        'SensePosition', 'off', 'SenseVelocity', 'off');
    set_param(step, 'Time', sprintf('%.12g', releaseTime), ...
        'Before', '1', 'After', '0');
end

% Persist the validated sensor range in the physical test model.
sensorPathWorkflow = [physicalModelName '/ObstacleSensorLogic'];
rtWorkflow = sfroot;
chartsWorkflow = find(rtWorkflow, '-isa', 'Stateflow.EMChart');
sensorChartWorkflow = [];
for k = 1:numel(chartsWorkflow)
    if strcmp(chartsWorkflow(k).Path, sensorPathWorkflow)
        sensorChartWorkflow = chartsWorkflow(k);
        break;
    end
end
assert(~isempty(sensorChartWorkflow), 'Sensor chart not found: %s', sensorPathWorkflow);
sensorSourceWorkflow = char(sensorChartWorkflow.Script);
sensorPatternWorkflow = '(?m)^[ \t]*sensorRange\s*=\s*[0-9.]+[ \t]*;';
assert(numel(regexp(sensorSourceWorkflow, sensorPatternWorkflow, 'match')) == 1, ...
    'Expected exactly one sensorRange assignment.');
sensorChartWorkflow.Script = regexprep(sensorSourceWorkflow, sensorPatternWorkflow, ...
    'sensorRange = 18.0;', 'once');

% Generalize configured obstacle environment to a fixed capacity of 10.
% Unused slots remain NaN and are ignored by the sensor.
environmentPathWorkflow = [physicalModelName '/DynamicObstacleEnvironment'];
environmentChartWorkflow = [];
for k = 1:numel(chartsWorkflow)
    if strcmp(chartsWorkflow(k).Path, environmentPathWorkflow)
        environmentChartWorkflow = chartsWorkflow(k);
        break;
    end
end
assert(~isempty(environmentChartWorkflow), ...
    'Dynamic obstacle environment chart not found: %s', environmentPathWorkflow);
environmentSourceWorkflow = sprintf([ ...
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
environmentChartWorkflow.Script = environmentSourceWorkflow;

% Match online replan manager storage capacity to the 10-slot environment.
managerPathWorkflow = [physicalModelName '/OnlineReplanManager'];
managerChartWorkflow = [];
for k = 1:numel(chartsWorkflow)
    if strcmp(chartsWorkflow(k).Path, managerPathWorkflow)
        managerChartWorkflow = chartsWorkflow(k);
        break;
    end
end
assert(~isempty(managerChartWorkflow), ...
    'Online replan manager chart not found: %s', managerPathWorkflow);
managerSourceWorkflow = char(managerChartWorkflow.Script);
managerPatternWorkflow = '(?m)^[ \t]*maxObstacles\s*=\s*\d+\s*;';
assert(numel(regexp(managerSourceWorkflow, managerPatternWorkflow, 'match')) == 1, ...
    'Expected exactly one maxObstacles assignment in OnlineReplanManager.');
managerChartWorkflow.Script = regexprep(managerSourceWorkflow, managerPatternWorkflow, ...
    'maxObstacles = 10;', 'once');

% Synchronize the rover's actual initial state to the saved scenario pose.
chassisJointWorkflow = [physicalModelName '/6-DOF Joint'];
oldXWorkflow = str2double(get_param(chassisJointWorkflow, 'PxPositionTargetValue'));
oldYWorkflow = str2double(get_param(chassisJointWorkflow, 'PyPositionTargetValue'));
oldZTextWorkflow = get_param(chassisJointWorkflow, 'PzPositionTargetValue');
oldZWorkflow = str2double(oldZTextWorkflow);
oldQuatWorkflow = get_param(chassisJointWorkflow, 'SphPositionTargetQuaternion');
zOldWorkflow = interp2(X, Y, Z_multibody, oldXWorkflow, oldYWorkflow, 'linear');
zNewWorkflow = interp2(X, Y, Z_multibody, startPoseWorkflow(1), startPoseWorkflow(2), 'linear');
assert(all(isfinite([oldXWorkflow oldYWorkflow oldZWorkflow zOldWorkflow zNewWorkflow])), ...
    'Could not compute terrain-adjusted rover initial pose.');
newZWorkflow = oldZWorkflow + (zNewWorkflow - zOldWorkflow);
yawWorkflow = startPoseWorkflow(3);
newQuatWorkflow = [cos(yawWorkflow/2) 0 0 sin(yawWorkflow/2)];
set_param(chassisJointWorkflow, ...
    'PxPositionTargetValue', sprintf('%.15g', startPoseWorkflow(1)), ...
    'PyPositionTargetValue', sprintf('%.15g', startPoseWorkflow(2)), ...
    'PzPositionTargetValue', sprintf('%.15g', newZWorkflow), ...
    'SphPositionTargetQuaternion', mat2str(newQuatWorkflow, 15));

set_param(physicalModelName, 'SimulationCommand', 'update');
save_system(physicalModelName, physicalModelFile);
catch ME
    % Roll back all in-memory model edits if synchronization or model update fails.
    try
        close_system(physicalModelName, 0);
        copyfile(backupWorkflow, physicalModelFile, 'f');
        load_system(physicalModelFile);
    catch rollbackME
        fprintf(2, 'Rollback warning: %s\n', rollbackME.message);
    end
    rethrow(ME);
end

fprintf('Scenario: %s\n', scenarioConfig.name);
fprintf('Start: (%.3f, %.3f) m; goal: (%.3f, %.3f) m\n', ...
    startPoseWorkflow(1), startPoseWorkflow(2), goalX, goalY);
fprintf('Initial route length: %.3f m (%d waypoints)\n', ...
    initialRouteLengthWorkflow, initialWaypointCountWorkflow);
fprintf('Physical sensor range: 18 m\n');
fprintf('Obstacle 2 placement-Z calibration: %.6f m; release %.3f s; spawn %.3f s\n', ...
    placementZOffsetsWorkflow(1), obsWorkflow(2).spawnTime - sqrt(2*abs(-(obsWorkflow(2).dimensions(3)/2+embedDepthWorkflow))/g), obsWorkflow(2).spawnTime);
fprintf('Obstacle 3 placement-Z calibration: %.6f m; release %.3f s; spawn %.3f s\n', ...
    placementZOffsetsWorkflow(2), obsWorkflow(3).spawnTime - sqrt(2*abs(-(obsWorkflow(3).dimensions(3)/2+embedDepthWorkflow))/g), obsWorkflow(3).spawnTime);

%% 4. Run the physical test model
stopTimeWorkflow = 60;
fprintf('\n[4/6] Simulating physical test model for %.3f s...\n', stopTimeWorkflow);
simOutPhysicalScenarioWorkflow = sim(physicalModelName, ...
    'StopTime', sprintf('%.15g', stopTimeWorkflow));
simOut = simOutPhysicalScenarioWorkflow; % Viewer reads this workspace variable.

execInfoWorkflow = simOutPhysicalScenarioWorkflow.SimulationMetadata.ExecutionInfo;
fprintf('Simulation stop event: %s at %.3f s\n', ...
    execInfoWorkflow.StopEvent, execInfoWorkflow.StopEventTime);
assert(strcmp(execInfoWorkflow.StopEvent, 'ReachedStopTime'), ...
    'Physical simulation did not reach configured stop time.');

%% 5. Verify logs, replanning events and final goal error
fprintf('\n[5/6] Checking simulation logs...\n');
logNamesWorkflow = simOutPhysicalScenarioWorkflow.who;
assert(all(ismember({'mb_px','mb_py'}, logNamesWorkflow)), ...
    'Physical simulation lacks mb_px/mb_py. Available outputs: %s', strjoin(logNamesWorkflow, ', '));
pxWorkflow = simOutPhysicalScenarioWorkflow.get('mb_px');
pyWorkflow = simOutPhysicalScenarioWorkflow.get('mb_py');
initialErrorWorkflow = hypot(pxWorkflow.Data(1)-startPoseWorkflow(1), ...
                             pyWorkflow.Data(1)-startPoseWorkflow(2));
finalErrorWorkflow = hypot(pxWorkflow.Data(end)-goalX, pyWorkflow.Data(end)-goalY);
fprintf('Logged rover interval: %.3f to %.3f s\n', pxWorkflow.Time(1), pxWorkflow.Time(end));
fprintf('Initial rover XY: (%.3f, %.3f) m; start error %.3f m\n', ...
    pxWorkflow.Data(1), pyWorkflow.Data(1), initialErrorWorkflow);
fprintf('Final rover XY: (%.3f, %.3f) m; distance to goal %.3f m\n', ...
    pxWorkflow.Data(end), pyWorkflow.Data(end), finalErrorWorkflow);

eventCountWorkflow = 0;
if any(strcmp(logNamesWorkflow, 'onlineReplanEventLog'))
    eventTSWorkflow = simOutPhysicalScenarioWorkflow.get('onlineReplanEventLog');
    eventDataWorkflow = double(eventTSWorkflow.Data(:));
    eventTimeWorkflow = double(eventTSWorkflow.Time(:));
    eventRiseWorkflow = find(diff([0; eventDataWorkflow > 0.5]) == 1);
    eventCountWorkflow = numel(eventRiseWorkflow);
    fprintf('Online replanning events: %d\n', eventCountWorkflow);
    for k = 1:eventCountWorkflow
        fprintf('  Event %d: %.3f s\n', k, eventTimeWorkflow(eventRiseWorkflow(k)));
    end
else
    warning('Physical workflow returned no onlineReplanEventLog.');
end
if any(strcmp(logNamesWorkflow, 'sensorDetectedLog'))
    sensorTSWorkflow = simOutPhysicalScenarioWorkflow.get('sensorDetectedLog');
    fprintf('Sensor-log values: ');
    disp(unique(sensorTSWorkflow.Data(:))');
end

%% 6. Launch viewer using current physical run
fprintf('\n[6/6] Launching multi-dynamic elevation viewer...\n');
oldViewerFiguresWorkflow = findall(0, 'Type', 'figure', ...
    'Name', 'Elevation-Colored Terrain + Rover Viewer');
if ~isempty(oldViewerFiguresWorkflow), delete(oldViewerFiguresWorkflow); end
viewerFigurePhysicalWorkflow = launchElevationTerrainRoverViewerMultiDynamic(); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' PHYSICAL WORKFLOW COMPLETE\n');
fprintf('============================================================\n');
fprintf('Physical model: %s\n', physicalModelFile);
fprintf('Simulation output: simOutPhysicalScenarioWorkflow (also aliased as simOut)\n');
fprintf('Original checkpoint untouched: OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx\n');
fprintf('ScenarioIntegrationTest untouched by this physical runner.\n');
fprintf(['Caution: obstacle 2/3 centers are calibrated toward world Z=0; ', ...
    'the guided joint stop is an approximation, not terrain contact, and can intersect the terrain.\n']);
if numel(obsWorkflow) > 3
    fprintf(['Additional obstacle IDs >= 4 were included in virtual sensing, planning, and viewer display, ' ...
        'but have no separate Simscape collision bodies.\n']);
end
