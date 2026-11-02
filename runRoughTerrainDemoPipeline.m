%% runRoughTerrainDemoPipeline.m
% One-entry-point runner for the final six-obstacle rough-terrain rover demo.
%
% Pipeline:
%   1) Rebuild the terrain/planner workspace.
%   2) Restore the saved six-obstacle demonstration scenario.
%   3) Reconstruct the base-workspace variables used by the model.
%   4) Generate the initial geometry-aware A* route.
%   5) Run the selected, already-built Simscape model for 100 s.
%   6) Validate rover motion and online replanning logs.
%   7) Save the SimulationOutput and launch the six-obstacle viewer.
%
% IMPORTANT MODEL-SAFETY RULES:
%   - This script does NOT edit Simulink blocks or Stateflow charts.
%   - It does NOT call save_system and does NOT modify the .slx file on disk.
%   - The existing physical Simscape obstacles remain exactly as they are.
%   - All six configured obstacles are rendered by the separate MATLAB viewer.
%   - StopTime is overridden through SimulationInput, not written to the model.
%
% Run from the project root, or run this file by its full path.

%% 0. Resolve project files
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
if ~isfolder(projectRoot)
    error('DemoPipeline:ProjectRoot', 'Could not resolve project root.');
end
cd(projectRoot);
addpath(projectRoot);

% Final demonstration model selected from the current project inventory.
modelName = 'OffRoadMultibodyStage1_Demo_20261101';
modelFile = fullfile(projectRoot, [modelName '.slx']);
scenarioFile = fullfile(projectRoot, ...
    'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot, 'setupOffRoadProject.m');
plannerFile = fullfile(projectRoot, 'onlineReplanFromPoseGeometryAware.m');
viewerFile = fullfile(projectRoot, 'launchElevationTerrainRoverViewerMultiDynamic.m');

demoStopTime = 100.0;  % Matches the validated 100 s Cz20 demonstration run.
saveResults = true;
launchViewer = true;

requiredFiles = {modelFile, scenarioFile, setupFile, plannerFile, viewerFile};
for k = 1:numel(requiredFiles)
    assert(isfile(requiredFiles{k}), 'Required project file is missing: %s', ...
        requiredFiles{k});
end

fprintf('\n============================================================\n');
fprintf(' ROUGH-TERRAIN ROVER — SIX-OBSTACLE DEMO PIPELINE\n');
fprintf('============================================================\n');
fprintf('Project root : %s\n', projectRoot);
fprintf('Model        : %s\n', modelName);
fprintf('Scenario     : %s\n', scenarioFile);
fprintf('Run duration : %.1f s\n', demoStopTime);
fprintf('Model edits  : NONE (read-only simulation run)\n');

%% 1. Rebuild terrain and planner workspace
fprintf('\n[1/7] Rebuilding terrain and planner workspace...\n');
run(setupFile);

assert(exist('X_mb','var') == 1 && exist('Y_mb','var') == 1 && ...
       exist('Z_mb','var') == 1, ...
    'Terrain setup did not create X_mb, Y_mb and Z_mb.');
assert(exist('plannerFootprint','var') == 1, ...
    'Terrain setup did not create plannerFootprint.');

%% 2. Restore and validate the exact six-obstacle scenario
fprintf('\n[2/7] Restoring six-obstacle scenario...\n');
Sdemo = load(scenarioFile, 'scenarioConfig');
assert(isfield(Sdemo, 'scenarioConfig') && isstruct(Sdemo.scenarioConfig), ...
    'Scenario file does not contain a valid scenarioConfig structure.');
scenarioConfig = Sdemo.scenarioConfig;
assert(isfield(scenarioConfig, 'obstacles') && ...
       numel(scenarioConfig.obstacles) == 6, ...
    'Expected exactly six obstacle definitions in the saved scenario.');

obs = scenarioConfig.obstacles(:);
[~, obstacleOrder] = sort([obs.id]);
obs = obs(obstacleOrder);
assert(isequal([obs.id], 1:6), ...
    'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]), ...
    [false false true false true false]), ...
    'Obstacle static/dynamic roles differ from the validated demo scenario.');
assert(abs(double(obs(3).spawnTime) - 7.0) < 1e-9, ...
    'Obstacle 3 activation must remain at 7 s.');
assert(abs(double(obs(5).spawnTime) - 15.0) < 1e-9, ...
    'Obstacle 5 activation must remain at 15 s.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
assert(numel(startPoseWorkflow) >= 3 && numel(goalPoseWorkflow) >= 2, ...
    'Scenario startPose/goalPose have an unexpected format.');
goalXY = goalPoseWorkflow(1:2);
goalX = goalXY(1);
goalY = goalXY(2);

fprintf('Scenario      : %s\n', scenarioConfig.name);
fprintf('Start XY      : [%.4f %.4f] m\n', startPoseWorkflow(1:2));
fprintf('Goal XY       : [%.4f %.4f] m\n', goalXY);
fprintf('Obstacle IDs  : %s\n', mat2str([obs.id]));
fprintf('Physical core : IDs 1, 2 and 3 remain in the Simscape model.\n');
fprintf('Viewer layer  : all six obstacles come from scenarioConfig.\n');

%% 3. Reconstruct model workspace inputs (10 obstacle slots)
% The model's obstacle environment and Stateflow charts use fixed-width
% arrays. Unused slots are NaN and therefore ignored by the environment.
fprintf('\n[3/7] Reconstructing model inputs...\n');
allCenters = vertcat(obs.centerXY);
configuredObstacleCount = numel(obs);
modelObstacleCapacity = 10;
assert(configuredObstacleCount <= modelObstacleCapacity, ...
    'The model supports at most %d configured obstacle slots.', ...
    modelObstacleCapacity);
paddingCount = modelObstacleCapacity - configuredObstacleCount;

obstacleXs = [double(allCenters(:,1).'), nan(1,paddingCount)];
obstacleYs = [double(allCenters(:,2).'), nan(1,paddingCount)];
obstacleSpawnTime = [double([obs.spawnTime]), nan(1,paddingCount)];
obstacle3X = double(obs(3).centerXY(1));
obstacle3Y = double(obs(3).centerXY(2));
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);

% Generate the initial path around all static obstacles. Dynamic obstacles
% (IDs 3 and 5) enter through the model's existing sensor/replanning logic.
staticObs = obs(~[obs.isDynamic]);
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded, initialWaypointCount, initialClearances, ...
    initialRouteLength] = onlineReplanFromPoseGeometryAware( ...
        startPoseWorkflow(1:2), goalXY, ...
        double(staticCenters(:,1).'), double(staticCenters(:,2).'), ...
        obstacleClearance, maxWaypoints); %#ok<ASGLU>

assert(initialWaypointCount >= 2 && ...
       size(originalPathPadded,2) == 2, ...
    'Initial geometry-aware A* route generation failed.');
waypoints = originalPathPadded(1:initialWaypointCount,:);
newPath = waypoints; %#ok<NASGU>

fprintf('Initial route length : %.3f m\n', initialRouteLength);
fprintf('Initial waypoints    : %d\n', initialWaypointCount);
fprintf('Static obstacles     : %s\n', mat2str([staticObs.id]));
fprintf('Dynamic obstacles    : %s\n', mat2str([obs([obs.isDynamic]).id]));

%% 4. Load the selected model without editing or saving it
fprintf('\n[4/7] Loading demonstration model...\n');
wasLoadedBefore = bdIsLoaded(modelName);
if ~wasLoadedBefore
    load_system(modelFile);
end
modelDirtyBefore = get_param(modelName, 'Dirty');
savedModelStopTime = get_param(modelName, 'StopTime');
fprintf('Model loaded          : yes\n');
fprintf('Model Dirty flag      : %s\n', modelDirtyBefore);
fprintf('Saved model StopTime  : %s s\n', savedModelStopTime);
fprintf('Temporary run StopTime: %.1f s\n', demoStopTime);

%% 5. Run using SimulationInput (does not write model parameter changes)
fprintf('\n[5/7] Running the %g-second Simscape simulation...\n', demoStopTime);
simInDemo = Simulink.SimulationInput(modelName);
simInDemo = simInDemo.setModelParameter( ...
    'StopTime', sprintf('%.15g', demoStopTime));

try
    simOutDemoPipeline = sim(simInDemo);
catch ME
    fprintf(2, '\nDEMONSTRATION SIMULATION FAILED\n%s\n', ...
        getReport(ME, 'extended', 'hyperlinks', 'off'));
    rethrow(ME);
end

% The viewer expects a variable named simOut in the MATLAB base workspace.
simOut = simOutDemoPipeline;
assignin('base', 'simOutDemoPipeline', simOutDemoPipeline);
assignin('base', 'simOut', simOutDemoPipeline);
assignin('base', 'scenarioConfig', scenarioConfig);

execInfoDemo = simOutDemoPipeline.SimulationMetadata.ExecutionInfo;
fprintf('Simulation stop event: %s at %.3f s\n', ...
    execInfoDemo.StopEvent, execInfoDemo.StopEventTime);
assert(strcmp(execInfoDemo.StopEvent, 'ReachedStopTime'), ...
    'Simulation did not reach the requested %.1f s stop time.', demoStopTime);

%% 6. Validate motion and replanning logs
fprintf('\n[6/7] Validating simulation results...\n');
logNamesDemo = simOutDemoPipeline.who;
assert(all(ismember({'mb_px','mb_py'}, logNamesDemo)), ...
    'Simulation output is missing mb_px and/or mb_py. Available: %s', ...
    strjoin(logNamesDemo, ', '));

pxTSdemo = simOutDemoPipeline.get('mb_px');
pyTSdemo = simOutDemoPipeline.get('mb_py');
assert(isa(pxTSdemo,'timeseries') && isa(pyTSdemo,'timeseries'), ...
    'Expected mb_px and mb_py to be timeseries objects.');

pxDataDemo = double(pxTSdemo.Data(:));
pyDataDemo = double(pyTSdemo.Data(:));
pxTimeDemo = double(pxTSdemo.Time(:));
pyTimeDemo = double(pyTSdemo.Time(:));
nDemo = min([numel(pxDataDemo), numel(pyDataDemo), ...
             numel(pxTimeDemo), numel(pyTimeDemo)]);
assert(nDemo >= 2, 'Rover XY logs contain fewer than two samples.');
pxDataDemo = pxDataDemo(1:nDemo);
pyDataDemo = pyDataDemo(1:nDemo);
pxTimeDemo = pxTimeDemo(1:nDemo);
pyTimeDemo = pyTimeDemo(1:nDemo);

initialPositionErrorDemo = hypot(pxDataDemo(1)-startPoseWorkflow(1), ...
    pyDataDemo(1)-startPoseWorkflow(2));
finalGoalErrorDemo = hypot(pxDataDemo(end)-goalX, pyDataDemo(end)-goalY);
travelDistanceDemo = sum(hypot(diff(pxDataDemo), diff(pyDataDemo)));

fprintf('Recorded rover time : %.3f to %.3f s\n', ...
    pxTimeDemo(1), pxTimeDemo(end));
fprintf('Initial rover XY    : [%.4f %.4f] m (start error %.4f m)\n', ...
    pxDataDemo(1), pyDataDemo(1), initialPositionErrorDemo);
fprintf('Final rover XY      : [%.4f %.4f] m\n', ...
    pxDataDemo(end), pyDataDemo(end));
fprintf('Goal error          : %.4f m\n', finalGoalErrorDemo);
fprintf('Logged path length  : %.3f m\n', travelDistanceDemo);

replanEventTimesDemo = [];
if any(strcmp(logNamesDemo, 'onlineReplanEventLog'))
    eventTSdemo = simOutDemoPipeline.get('onlineReplanEventLog');
    eventDataDemo = double(eventTSdemo.Data(:));
    eventTimeDemo = double(eventTSdemo.Time(:));
    eventRiseDemo = find(diff([0; eventDataDemo > 0.5]) == 1);
    replanEventTimesDemo = eventTimeDemo(eventRiseDemo);
    fprintf('Online replanning events: %d\n', numel(replanEventTimesDemo));
    for k = 1:numel(replanEventTimesDemo)
        fprintf('  Replan %d at %.3f s\n', k, replanEventTimesDemo(k));
    end
else
    warning('DemoPipeline:MissingReplanLog', ...
        'onlineReplanEventLog was not present in the simulation output.');
end

if any(strcmp(logNamesDemo, 'sensorDetectedLog'))
    sensorTSdemo = simOutDemoPipeline.get('sensorDetectedLog');
    fprintf('Sensor log unique values: %s\n', ...
        mat2str(unique(double(sensorTSdemo.Data(:))).'));
end

% Save only the output and scenario evidence; never save the Simulink model.
if saveResults
    stampDemo = datestr(now, 'yyyymmdd_HHMMSS');
    resultFileDemo = fullfile(projectRoot, ...
        ['Demo_Run_' stampDemo '_100s.mat']);
    save(resultFileDemo, 'simOutDemoPipeline', 'scenarioConfig', ...
        'initialRouteLength', 'initialWaypointCount', ...
        'initialPositionErrorDemo', 'finalGoalErrorDemo', ...
        'travelDistanceDemo', 'replanEventTimesDemo', '-v7.3');
    fprintf('Saved run output     : %s\n', resultFileDemo);
end

%% 7. Launch the separate viewer (all six obstacles are graphics here)
if launchViewer
    fprintf('\n[7/7] Launching six-obstacle elevation viewer...\n');
    oldViewerFigures = findall(0, 'Type', 'figure', ...
        'Name', 'Elevation-Colored Terrain + Rover Viewer');
    if ~isempty(oldViewerFigures)
        delete(oldViewerFigures);
    end
    viewerFigureDemo = launchElevationTerrainRoverViewerMultiDynamic(); %#ok<NASGU>
else
    fprintf('\n[7/7] Viewer launch disabled by user setting.\n');
end

modelDirtyAfter = get_param(modelName, 'Dirty');
modelStopTimeAfter = get_param(modelName, 'StopTime');
assert(strcmp(modelDirtyBefore, modelDirtyAfter), ...
    'Model Dirty flag changed during the pipeline; inspect before saving.');
assert(strcmp(savedModelStopTime, modelStopTimeAfter), ...
    'Model StopTime changed on the loaded model unexpectedly.');

fprintf('\n============================================================\n');
fprintf(' SIX-OBSTACLE DEMONSTRATION PIPELINE COMPLETE\n');
fprintf('============================================================\n');
fprintf('Simscape model : %s.slx (not edited or saved)\n', modelName);
fprintf('Viewer         : all six obstacles from scenarioConfig\n');
fprintf('Simulation     : %.1f s completed\n', demoStopTime);
fprintf('Goal error     : %.4f m\n', finalGoalErrorDemo);
fprintf('Workspace vars : simOut, simOutDemoPipeline, scenarioConfig\n');
fprintf('============================================================\n');
