%% createPhysicalScenarioSpawnTestModel_v2.m
% Create a new isolated physical test model containing physical box bodies for
% dynamic obstacles 2 and 3. Never overwrites the original checkpoint, the
% ScenarioIntegrationTest model, or the earlier PhysicalScenarioSpawnTest.
%
% Tested configuration carried forward from the user's successful 45 s run:
% - obstacle sensor range = 18 m;
% - obstacle 2/3 box centers calibrated to settle at world Z=0;
% - initial joint coordinates place most of each box below Z=0 before release;
% - upper joint stop at q=0, stiffness 1e5 N/m, damping 1500 N/(m/s);
% - mass-based inertia with explicit test masses 25 kg and 10 kg.
%
% The release is a guided joint-mode approximation. Bodies exist physically
% throughout the simulation; they are mostly below Z=0 until release. This is
% NOT true terrain contact. World Z=0 can intersect the terrain.
%
% Calibration is valid only for the current obstacle centers/dimensions below.
% Recalibrate before using this physical model for changed obstacle geometry.
%
% Build only; this script does not launch a simulation.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
if ~strcmpi(pwd, projectRoot), cd(projectRoot); end
addpath(projectRoot);

sourceModel = 'OffRoadMultibodyStage1_ScenarioIntegrationTest';
testModel = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2';
sourceFile = fullfile(projectRoot, [sourceModel '.slx']);
testFile = fullfile(projectRoot, [testModel '.slx']);
scenarioFile = fullfile(projectRoot, 'scenarioConfig.mat');

assert(isfile(sourceFile), ...
    'Missing working source test model: %s. Run runRoughTerrainWorkflow first.', sourceFile);
assert(isfile(scenarioFile), ...
    'Missing scenarioConfig.mat. Save the scenario from the editor first.');
assert(~isfile(testFile), ...
    ['Physical test model already exists; it will not be overwritten automatically: %s. ', ...
     'Rename/archive that TEST file before creating a fresh copy.'], testFile);
assert(~bdIsLoaded(testModel), 'Target model is already loaded: %s', testModel);

fprintf('=== 1. Rebuilding workspace dependencies ===\n');
run(fullfile(projectRoot, 'setupOffRoadProject.m'));
savedData = load(scenarioFile, 'scenarioConfig');
assert(isfield(savedData, 'scenarioConfig'), ...
    'scenarioConfig.mat does not contain scenarioConfig.');
scenarioConfig = savedData.scenarioConfig;

obs = scenarioConfig.obstacles(:);
[~, ix] = sort([obs.id]);
obs = obs(ix);
obs2Index = find([obs.id] == 2, 1, 'first');
obs3Index = find([obs.id] == 3, 1, 'first');
assert(~isempty(obs2Index) && ~isempty(obs3Index), ...
    'Scenario must include obstacles with IDs 2 and 3.');
obs2 = obs(obs2Index);
obs3 = obs(obs3Index);
assert(strcmpi(obs2.shape, 'box') && strcmpi(obs3.shape, 'box'), ...
    'Physical spawn test currently supports box obstacles only.');
assert(obs2.isDynamic && obs3.isDynamic, ...
    'Obstacles 2 and 3 must both be dynamic for this spawn test.');
assert(all(isfinite([obs2.spawnTime obs3.spawnTime])) && ...
       all([obs2.spawnTime obs3.spawnTime] > 0), ...
    'Dynamic spawn times must be positive and finite.');
assert(all(obs2.dimensions > 0) && all(obs3.dimensions > 0), ...
    'Obstacle dimensions must be positive.');

% Check required source blocks exist in the saved source model.
load_system(sourceFile);
sourceBlocks = {'Chassis','World Frame','PhysicalObstacle1', ...
    'Obstacle1Placement','Spatial Contact Force','Prismatic Joint3'};
for k = 1:numel(sourceBlocks)
    assert(getSimulinkBlockHandle([sourceModel '/' sourceBlocks{k}]) ~= -1, ...
        'Expected block missing in source model: %s', sourceBlocks{k});
end

% World-Z calibration, based on the empirically verified 10/15 s runs.
% These exact centers and dimensions are guarded to prevent silent use of a
% stale calibration when the scenario editor changes obstacle geometry.
spawnTimes = [obs2.spawnTime obs3.spawnTime];
dimsList = [obs2.dimensions; obs3.dimensions];
centers = [obs2.centerXY; obs3.centerXY];
calibratedCenters = [23.7161198288 22.8601997147; ...
                    16.7974322397 36.0556348074];
calibratedDimensions = [2 23 1; 4 4 0.5];
assert(all(abs(centers-calibratedCenters) < 1e-3, 'all'), ...
    ['The world-Z calibration is specific to the current obstacle 2/3 centers. ', ...
     'Use the virtual integration workflow or recalibrate after moving obstacles.']);
assert(all(abs(dimsList-calibratedDimensions) < 1e-9, 'all'), ...
    ['The world-Z calibration is specific to obstacle 2=[2 23 1] and ', ...
     'obstacle 3=[4 4 0.5]. Recalibrate after resizing them.']);

placementZOffsets = [0.061312 0.024525];
embedDepth = 0.05; % hidden below the world-Z=0 plane before release
accelG = 9.81;
qInitialList = -(dimsList(:,3)'/2 + embedDepth);
travelDistances = abs(qInitialList);
releaseTimes = spawnTimes - sqrt(2*travelDistances/accelG);
assert(all(releaseTimes > 0), ...
    'Each configured spawn time must exceed its nominal release lead-in.');

% Configure output test copy using the current working integration test as template.
createdModel = false;
try
    fprintf('\n=== 2. Creating separate physical-spawn test copy ===\n');
    new_system(testModel, 'FromFile', sourceFile);
    createdModel = true;

    % Persist the experimentally validated 18 m sensor range in this new copy.
    sensorPath = [testModel '/ObstacleSensorLogic'];
    sfRootV2 = sfroot;
    chartsV2 = find(sfRootV2, '-isa', 'Stateflow.EMChart');
    sensorChartV2 = [];
    for chartIndexV2 = 1:numel(chartsV2)
        if strcmp(chartsV2(chartIndexV2).Path, sensorPath)
            sensorChartV2 = chartsV2(chartIndexV2);
            break;
        end
    end
    assert(~isempty(sensorChartV2), 'Could not find chart %s.', sensorPath);
    sensorSourceV2 = char(sensorChartV2.Script);
    sensorPatternV2 = '(?m)^[ \t]*sensorRange\s*=\s*[0-9.]+[ \t]*;';
    assert(numel(regexp(sensorSourceV2, sensorPatternV2, 'match')) == 1, ...
        'Expected exactly one sensorRange assignment in %s.', sensorPath);
    sensorChartV2.Script = regexprep(sensorSourceV2, sensorPatternV2, ...
        'sensorRange = 18.0;', 'once');

    oldBrick = [testModel '/PhysicalObstacle1'];
    oldPlacement = [testModel '/Obstacle1Placement'];
    oldContact = [testModel '/Spatial Contact Force'];
    jointTemplate = [testModel '/Prismatic Joint3'];

    chassisPorts = get_param([testModel '/Chassis'], 'PortHandles');
    worldPorts = get_param([testModel '/World Frame'], 'PortHandles');
    assert(~isempty(chassisPorts.RConn) && ~isempty(worldPorts.RConn), ...
        'Chassis/World physical connection ports were not found.');

    % Moderate test masses, set via Mass-based inertia. These are explicit
    % simulation placeholders, not material-density estimates.
    testMasses = [25 10];
    obstacleIds = [2 3];

    load_system('nesl_utility');
    converterLibraryPath = sprintf('nesl_utility/Simulink-PS\nConverter');

    for n = 1:2
        id = obstacleIds(n);
        suffix = sprintf('%d', id);
        shift = [420 + 300*n, 260 + 280*n, 420 + 300*n, 260 + 280*n];

        brick = [testModel '/PhysicalObstacle' suffix];
        placement = [testModel '/Obstacle' suffix 'Placement'];
        contact = [testModel '/Spatial Contact Force Obstacle' suffix];
        joint = [testModel '/Obstacle' suffix 'SpawnJoint'];
        step = [testModel '/Obstacle' suffix 'SpawnModeStep'];
        converter = [testModel '/Obstacle' suffix 'ModeConverter'];

        add_block(oldBrick, brick, ...
            'Position', get_param(oldBrick,'Position') + shift);
        add_block(oldPlacement, placement, ...
            'Position', get_param(oldPlacement,'Position') + shift);
        add_block(oldContact, contact, ...
            'Position', get_param(oldContact,'Position') + shift);
        add_block(jointTemplate, joint, ...
            'Position', get_param(jointTemplate,'Position') + shift);

        % Parameterize the obstacle geometry and a stable, explicit test mass.
        brickParams = get_param(brick, 'DialogParameters');
        set_param(brick, 'BrickDimensions', mat2str(dimsList(n,:), 12));
        if isfield(brickParams, 'BasedOnType')
            set_param(brick, 'BasedOnType', 'Mass');
        end
        if isfield(brickParams, 'Mass')
            set_param(brick, 'Mass', sprintf('%.12g', testMasses(n)));
        end
        if isfield(brickParams, 'InertiaType')
            set_param(brick, 'InertiaType', 'CalculateFromGeometry');
        end

        % Apply the verified world-Z=0 calibration for this exact geometry.
        % The placement z offset compensates the settled joint residual measured
        % in the successful physical test; initial q places the box mostly below Z=0.
        placementXYZ = [centers(n,:), placementZOffsets(n)];
        set_param(placement, ...
            'TranslationMethod', 'Cartesian', ...
            'TranslationCartesianOffset', mat2str(placementXYZ, 12), ...
            'TranslationCartesianOffsetUnits', 'm');

        contactParams = get_param(contact, 'DialogParameters');
        optionalSensors = {'SenseContactSignal','SensePenetrationDepth', ...
            'SenseNormalForceMagnitude','SenseFrictionalForce', ...
            'SenseFrictionalForceMagnitude','SenseFrictionalTorque'};
        for q = 1:numel(optionalSensors)
            if isfield(contactParams, optionalSensors{q})
                set_param(contact, optionalSensors{q}, 'off');
            end
        end

        jointParams = get_param(joint, 'DialogParameters');
        requiredJointParams = {'JointMode','MotionActuationMode', ...
            'PositionTargetSpecify','PositionTargetValue','UpperLimitSpecify', ...
            'UpperLimitBound','UpperLimitStiffness','UpperLimitDamping'};
        for q = 1:numel(requiredJointParams)
            assert(isfield(jointParams, requiredJointParams{q}), ...
                'Joint parameter not available: %s', requiredJointParams{q});
        end
        set_param(joint, ...
            'JointMode', 'ProvidedByInput', ...
            'MotionActuationMode', 'ComputedMotion', ...
            'PositionTargetSpecify', 'on', ...
            'PositionTargetPriority', 'High', ...
            'PositionTargetValue', sprintf('%.12g', qInitialList(n)), ...
            'PositionTargetValueUnits', 'm', ...
            'LowerLimitSpecify', 'off', ...
            'UpperLimitSpecify', 'on', ...
            'UpperLimitBound', '0', ...
            'UpperLimitBoundUnits', 'm', ...
            'UpperLimitStiffness', '1e5', ...
            'UpperLimitDamping', '1500', ...
            'UpperLimitTransitionRegionWidth', '0.005', ...
            'SensePosition', 'off', ...
            'SenseVelocity', 'off');

        % Step encodes Locked=1 until the nominal descent begins, then Normal=0.
        add_block('simulink/Sources/Step', step, ...
            'Position', [1200 120+100*n 1240 150+100*n]);
        set_param(step, 'Time', sprintf('%.12g', releaseTimes(n)), ...
            'Before', '1', 'After', '0');
        add_block(converterLibraryPath, converter, ...
            'Position', [1310 115+100*n 1385 155+100*n]);
        converterParams = get_param(converter, 'DialogParameters');
        assert(isfield(converterParams, 'Unit'), ...
            'Simulink-PS Converter Unit parameter is missing.');
        set_param(converter, 'Unit', '1');

        stepPorts = get_param(step, 'PortHandles');
        converterPorts = get_param(converter, 'PortHandles');
        assert(~isempty(stepPorts.Outport) && ~isempty(converterPorts.Inport), ...
            'Step or Simulink-PS Converter ports are unavailable.');
        add_line(testModel, stepPorts.Outport(1), converterPorts.Inport(1), ...
            'autorouting', 'on');

        % Add physical collision connection and the world-to-body joint chain.
        brickPorts = get_param(brick, 'PortHandles');
        placementPorts = get_param(placement, 'PortHandles');
        jointPorts = get_param(joint, 'PortHandles');
        contactPorts = get_param(contact, 'PortHandles');
        assert(~isempty(brickPorts.LConn) && ~isempty(brickPorts.RConn) && ...
               ~isempty(placementPorts.LConn) && ~isempty(placementPorts.RConn) && ...
               ~isempty(jointPorts.LConn) && ~isempty(jointPorts.RConn) && ...
               ~isempty(contactPorts.LConn) && ~isempty(contactPorts.RConn), ...
            'One or more physical ports are missing for obstacle %d.', id);

        add_line(testModel, chassisPorts.RConn(1), contactPorts.LConn(1), 'autorouting','on');
        add_line(testModel, contactPorts.RConn(1), brickPorts.LConn(1), 'autorouting','on');
        add_line(testModel, worldPorts.RConn(1), placementPorts.LConn(1), 'autorouting','on');
        add_line(testModel, placementPorts.RConn(1), jointPorts.LConn(1), 'autorouting','on');
        add_line(testModel, jointPorts.RConn(1), brickPorts.RConn(1), 'autorouting','on');
        simscape.addConnection(converter, 'output', joint, 'mode_in', 'autorouting', 'on');

        fprintf('Obstacle %d: center XY=(%.3f, %.3f) m, size=[%.3f %.3f %.3f] m, mass=%.1f kg\n', ...
            id, centers(n,1), centers(n,2), dimsList(n,1), dimsList(n,2), dimsList(n,3), testMasses(n));
        fprintf('  placement Z offset=%.6f m; initial q=%.3f m; final q target=0; release=%.3f s; configured spawn=%.3f s\n', ...
            placementZOffsets(n), qInitialList(n), releaseTimes(n), spawnTimes(n));
    end

    fprintf('\n=== 3. Updating the new model (no simulation) ===\n');
    set_param(testModel, 'SimulationCommand', 'update');
    set_param([testModel '/Transfer Fcn'], 'Denominator', '[0.15 1]');
    save_system(testModel, testFile);

    fprintf('\n=== PHYSICAL SCENARIO SPAWN TEST MODEL READY ===\n');
    fprintf('Saved test copy: %s\n', testFile);
    fprintf('Source integration model preserved: %s.slx\n', sourceModel);
    fprintf('Original checkpoint preserved: OffRoadMultibodyStage1_OnlineReplanCheckpoint.slx\n');
    fprintf('Simulation has NOT been started.\n');
    fprintf(['Note: bodies exist from t=0 and are mostly below world Z=0 before release. ', ...
        'Joint-mode release plus a calibrated upper stop is an approximation, not terrain contact.\n']);
    fprintf('Sensor range persisted at 18 m. Simulation has NOT been started.\n');

catch ME
    if createdModel && bdIsLoaded(testModel)
        close_system(testModel, 0);
    end
    rethrow(ME);
end
