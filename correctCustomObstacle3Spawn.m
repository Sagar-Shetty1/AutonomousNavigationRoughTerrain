%% CORRECT OBSTACLE 3 INITIAL HIDDEN POSITION IN CUSTOM MODEL
% This is a build/configuration correction only. It does NOT simulate.
% It modifies only the CUSTOM model and keeps a timestamped backup.

projectRoot = 'C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain';
cd(projectRoot);
addpath(projectRoot);

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
modelFile = fullfile(projectRoot,[modelName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');

assert(isfile(modelFile),'Custom model not found: %s',modelFile);
assert(isfile(scenarioFile),'Custom scenario not found: %s',scenarioFile);

%% Restore terrain and scenario data needed for a safe recalculation
run(fullfile(projectRoot,'setupOffRoadProject.m'));
S = load(scenarioFile,'scenarioConfig');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,ord] = sort([obs.id]);
obs = obs(ord);

idx3 = find([obs.id] == 3,1);
assert(~isempty(idx3),'Obstacle ID 3 is missing.');
o3 = obs(idx3);
assert(o3.isDynamic && abs(o3.spawnTime-7) < 1e-9, ...
    'Obstacle 3 is not configured as dynamic with a 7 s activation.');

%% Recreate base-workspace values referenced by the model
startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalX = goalPoseWorkflow(1);
goalY = goalPoseWorkflow(2);
goalXY = goalPoseWorkflow(1:2);
allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1,4)];
obstacleYs = [allCenters(:,2).' nan(1,4)];
obstacleSpawnTime = [[obs.spawnTime] nan(1,4)];
obstacle3X = o3.centerXY(1);
obstacle3Y = o3.centerXY(2);
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

%% Calculate a fully hidden initial position relative to terrain minimum
mask = abs(X-o3.centerXY(1)) <= o3.dimensions(1)/2 & ...
       abs(Y-o3.centerXY(2)) <= o3.dimensions(2)/2 & ...
       isfinite(Z_multibody);
assert(any(mask(:)),'No terrain samples under obstacle 3.');
terrainMinZ = min(Z_multibody(mask));

wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelFile);
end

placementPath = [modelName '/Obstacle3Placement'];
jointPath = [modelName '/Obstacle3SpawnJoint'];
stepPath = [modelName '/Obstacle3SpawnModeStep'];

placementXYZ = str2num(get_param(placementPath,'TranslationCartesianOffset')); %#ok<ST2NM>
assert(numel(placementXYZ) == 3,'Unexpected obstacle 3 placement transform.');

height3 = double(o3.dimensions(3));
embedDepth = 0.05;
gravityMagnitude = 9.81;

% With q=0, the top of the box is placementZ + height/2.
% Place its initial top 5 cm below the lowest sampled terrain elevation.
finalBoxTopZ = placementXYZ(3) + height3/2;
desiredInitialTopZ = terrainMinZ - embedDepth;
qInitial3 = desiredInitialTopZ - finalBoxTopZ;

% Preserve the existing model's ballistic lead-in approximation.
travelTime3 = sqrt(2*abs(qInitial3)/gravityMagnitude);
releaseTime3 = double(o3.spawnTime) - travelTime3;
assert(qInitial3 < 0 && releaseTime3 > 0, ...
    'Calculated initial position or release time is invalid.');

%% Back up custom model before applying the correction
stamp = datestr(now,'yyyymmdd_HHMMSS');
backupFile = fullfile(projectRoot, ...
    [modelName '_preObstacle3SpawnCorrection_' stamp '.slx']);
[backupOK,backupMessage] = copyfile(modelFile,backupFile);
assert(backupOK,'Could not back up custom model: %s',backupMessage);

try
    set_param(jointPath, ...
        'JointMode','ProvidedByInput', ...
        'MotionActuationMode','ComputedMotion', ...
        'PositionTargetSpecify','on', ...
        'PositionTargetPriority','High', ...
        'PositionTargetValue',sprintf('%.12g',qInitial3), ...
        'PositionTargetValueUnits','m', ...
        'LowerLimitSpecify','off', ...
        'UpperLimitSpecify','on', ...
        'UpperLimitBound','0', ...
        'UpperLimitBoundUnits','m', ...
        'UpperLimitStiffness','1e5', ...
        'UpperLimitDamping','1500', ...
        'UpperLimitTransitionRegionWidth','0.005');

    set_param(stepPath, ...
        'Time',sprintf('%.12g',releaseTime3), ...
        'Before','1', ...
        'After','0');

    set_param(modelName,'SimulationCommand','update');
    save_system(modelName,modelFile);

catch ME
    % Discard any failed in-memory edit and restore the on-disk backup.
    try
        close_system(modelName,0);
        copyfile(backupFile,modelFile,'f');
        if wasLoaded
            load_system(modelFile);
        end
    catch rollbackME
        fprintf(2,'Rollback warning: %s\n',rollbackME.message);
    end
    rethrow(ME);
end

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 3 SPAWN POSITION CORRECTED (NO SIMULATION)\n');
fprintf('============================================================\n');
fprintf('Terrain minimum under footprint: %.6f m\n',terrainMinZ);
fprintf('Desired initial box top: %.6f m\n',desiredInitialTopZ);
fprintf('Final box top at q=0: %.6f m\n',finalBoxTopZ);
fprintf('Corrected initial joint target: %.6f m\n',qInitial3);
fprintf('Nominal travel time: %.6f s\n',travelTime3);
fprintf('Corrected release command: %.6f s\n',releaseTime3);
fprintf('Target activation time: %.3f s\n',o3.spawnTime);
fprintf('Backup: %s\n',backupFile);
fprintf('Custom model saved; no simulation was started.\n');
fprintf('============================================================\n');

if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName,0);
end
