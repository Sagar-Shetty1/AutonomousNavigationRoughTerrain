%% SIX-OBSTACLE MODEL — OBSTACLE 5 EARLY-ACTIVATION ABLATION (32 s)
% Purpose:
%   Test whether activating virtual obstacle 5 earlier (12.5 s instead of 15 s)
%   gives the rover more distance/time to accommodate the replacement route.
%
% Controlled change:
%   obstacleSpawnTime(5): 15 s -> 12.5 s, for this simulation only.
%
% Preserved:
%   - All six obstacle IDs, XY coordinates, and dimensions
%   - Obstacle 3 physical spawn at 7 s
%   - Normal steering, tires, terrain, controller, and solver settings
%   - Original Custom model and saved scenario MAT file
%
% The script creates a fresh diagnostic model copy from the saved
% UpperStopProbe model, adds temporary signal loggers in memory, runs 32 s,
% saves diagnostic outputs, and closes the copy without saving logger edits.
% It does NOT call solverprofiler.profileModel or any optional profiler API.

%% 0. Resolve files and create a uniquely named disposable model copy
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
cd(projectRoot);
addpath(projectRoot);

sourceProbeName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
sourceProbeFile = fullfile(projectRoot,[sourceProbeName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot,'setupOffRoadProject.m');

assert(isfile(sourceProbeFile), ...
    'Saved diagnostic source model not found: %s',sourceProbeFile);
assert(isfile(scenarioFile), ...
    'Six-obstacle scenario file not found: %s',scenarioFile);
assert(isfile(setupFile), ...
    'Workspace setup script not found: %s',setupFile);

stamp = datestr(now,'yyyymmdd_HHMMSS');
testModelName = ['OffRoadMultibodyStage1_Obstacle5EarlyActivationAblation_' stamp];
testModelFile = fullfile(projectRoot,[testModelName '.slx']);
assert(~bdIsLoaded(testModelName) && ~isfile(testModelFile), ...
    'The unique diagnostic model name/file already exists. Rerun the script to generate a new name.');

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 5 EARLY-ACTIVATION ABLATION — 32 s\n');
fprintf('============================================================\n');
fprintf('Source diagnostic model: %s\n',sourceProbeName);
fprintf('New disposable copy:     %s\n',testModelName);

%% 1. Restore project workspace and exact six-obstacle scenario
fprintf('\n[1/6] Restoring terrain/planner workspace and scenario...\n');
run(setupFile);
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'Scenario MAT file does not contain scenarioConfig.');
scenarioConfig = S.scenarioConfig;

assert(numel(scenarioConfig.obstacles)==6, ...
    'Expected exactly six obstacle definitions.');
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Static/dynamic obstacle roles do not match the six-obstacle scenario.');
assert(abs(double(obs(3).spawnTime)-7)<1e-9, ...
    'Obstacle 3 spawn time is not the expected 7 s.');
assert(abs(double(obs(5).spawnTime)-15)<1e-9, ...
    'Obstacle 5 original activation is not the expected 15 s.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalXY = goalPoseWorkflow(1:2);
goalX = goalPoseWorkflow(1); %#ok<NASGU>
goalY = goalPoseWorkflow(2); %#ok<NASGU>
allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1,4)]; %#ok<NASGU>
obstacleYs = [allCenters(:,2).' nan(1,4)]; %#ok<NASGU>
obstacleSpawnTime = [[obs.spawnTime] nan(1,4)]; %#ok<NASGU>
obstacle3X = double(obs(3).centerXY(1)); %#ok<NASGU>
obstacle3Y = double(obs(3).centerXY(2)); %#ok<NASGU>
obstacleClearance = double(scenarioConfig.planning.obstacleClearance); %#ok<NASGU>
maxWaypoints = double(scenarioConfig.planning.maxWaypoints); %#ok<NASGU>

% Rebuild the same initial path used by the prior six-obstacle diagnostics.
staticObs = obs(~[obs.isDynamic]);
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded,initialWaypointCount,~,initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2),goalXY, ...
    staticCenters(:,1).',staticCenters(:,2).', ...
    obstacleClearance,maxWaypoints);
assert(initialWaypointCount>=2,'Initial route calculation failed.');
waypoints = originalPathPadded(1:initialWaypointCount,:); %#ok<NASGU>
newPath = waypoints; %#ok<NASGU>

fprintf('Scenario: %s\n',scenarioConfig.name);
fprintf('Obstacle IDs: %s\n',mat2str([obs.id]));
fprintf('Initial route length: %.3f m (%d waypoints)\n', ...
    initialRouteLength,initialWaypointCount);
fprintf('Obstacle 3 activation retained: %.3f s\n',obstacleSpawnTime(3));
fprintf('Obstacle 5 original activation: %.3f s\n',obstacleSpawnTime(5));

%% 2. Apply exactly one temporary scenario change
obstacleSpawnTimeTest = obstacleSpawnTime;
obstacleSpawnTimeTest(5) = 12.5;
assert(obstacleSpawnTimeTest(3)==7.0 && obstacleSpawnTimeTest(5)==12.5, ...
    'Runtime activation-time override failed validation.');
fprintf('TEST OVERRIDE: obstacleSpawnTime(5) = %.1f s (original %.1f s).\n', ...
    obstacleSpawnTimeTest(5),obstacleSpawnTime(5));
fprintf('All obstacle positions/dimensions and other spawn times remain unchanged.\n');

%% 3. Create a new model from the saved diagnostic source (not the open source)
fprintf('\n[2/6] Creating an isolated diagnostic model copy...\n');
new_system(testModelName,'FromFile',sourceProbeFile);
copyCleanup = onCleanup(@() closeDiscardModel(testModelName)); %#ok<NASGU>
save_system(testModelName,testModelFile);

% Verify the relevant model blocks and expected signal interfaces before edits.
requiredBlocks = {'PurePursuitController','Transfer Fcn','AckermannSteering', ...
    'RouteSwitch','Obstacle3SpawnJoint','DynamicObstacleEnvironment'};
for k=1:numel(requiredBlocks)
    blockPath = [testModelName '/' requiredBlocks{k}];
    assert(getSimulinkBlockHandle(blockPath)~=-1, ...
        'Expected block missing from diagnostic copy: %s',blockPath);
end

%% 4. Add temporary steering and route loggers to this copy only
fprintf('\n[3/6] Adding temporary signal loggers...\n');
load_system('simulink');
ppPH = get_param([testModelName '/PurePursuitController'],'PortHandles');
tfPH = get_param([testModelName '/Transfer Fcn'],'PortHandles');
ackPH = get_param([testModelName '/AckermannSteering'],'PortHandles');
routePH = get_param([testModelName '/RouteSwitch'],'PortHandles');
assert(numel(ppPH.Outport)>=1 && numel(tfPH.Outport)>=1 && ...
       numel(ackPH.Outport)>=2 && numel(routePH.Outport)>=1, ...
    'One or more expected steering/route output ports are unavailable.');

signalSources = {ppPH.Outport(1),tfPH.Outport(1), ...
    ackPH.Outport(1),ackPH.Outport(2),routePH.Outport(1)};
variableNames = {'steerCmdProbe','deltaFilteredProbe', ...
    'deltaFLProbe','deltaFRProbe','routeInputProbe'};
prettyNames = {'Pure Pursuit steerCmd (rad)', ...
    'filtered steering command (rad)','Ackermann deltaFL (rad)', ...
    'Ackermann deltaFR (rad)','active RouteSwitch waypoint matrix'};
loggerBaseX = 1450;
loggerBaseY = 100;
loggerRowGap = 75;

for k=1:numel(variableNames)
    loggerPath = [testModelName '/Probe_ToWorkspace_' variableNames{k}];
    add_block('simulink/Sinks/To Workspace',loggerPath, ...
        'Position',[loggerBaseX loggerBaseY+(k-1)*loggerRowGap ...
                    loggerBaseX+170 loggerBaseY+35+(k-1)*loggerRowGap], ...
        'VariableName',variableNames{k},'SaveFormat','Timeseries');
    loggerPH = get_param(loggerPath,'PortHandles');
    assert(~isempty(loggerPH.Inport),'Logger input port missing: %s',loggerPath);
    add_line(testModelName,signalSources{k},loggerPH.Inport(1),'autorouting','on');
    fprintf('  %-22s <- %s\n',variableNames{k},prettyNames{k});
end

%% 5. Update and simulate with a SimulationInput-only variable override
fprintf('\n[4/6] Updating model and running 32-second test...\n');
set_param(testModelName,'SimulationCommand','update');

simscapeLogName = ['simlog_Obstacle5EarlyActivationAblation_' stamp];
simIn = Simulink.SimulationInput(testModelName);
simIn = simIn.setVariable('obstacleSpawnTime',obstacleSpawnTimeTest);
simIn = simIn.setModelParameter( ...
    'StopTime','32', ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',simscapeLogName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

ablationOutput = sim(simIn);
assignin('base','ablationOutput',ablationOutput);

%% 6. Save outputs and report the key comparison signals
fprintf('\n[5/6] Saving and inspecting diagnostic outputs...\n');
matFile = fullfile(projectRoot,['capturedObstacle5EarlyActivationAblation32s_' stamp '.mat']);
csvFile = fullfile(projectRoot,['steeringObstacle5EarlyActivationAblation32s_' stamp '.csv']);
save(matFile,'ablationOutput','scenarioConfig','obstacleSpawnTime', ...
    'obstacleSpawnTimeTest','variableNames','prettyNames','-v7.3');
fprintf('Saved SimulationOutput: %s\n',matFile);

returnedNames = ablationOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));
execInfo = ablationOutput.SimulationMetadata.ExecutionInfo;
fprintf('Stop event: %s | stop time: %.12f s\n', ...
    execInfo.StopEvent,execInfo.StopEventTime);

try
    capturedError = char(ablationOutput.ErrorMessage);
    if ~isempty(strtrim(capturedError))
        fprintf('\nCaptured simulation error:\n%s\n',capturedError);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
    fprintf('ErrorMessage field was not available in this SimulationOutput.\n');
end

% Report online replanning pulse times. Obstacle 5 should not cause a new
% detection/replan at approximately 15.2 s in this ablation.
if any(strcmp(returnedNames,'onlineReplanEventLog'))
    eventTS = ablationOutput.get('onlineReplanEventLog');
    eventData = double(eventTS.Data(:));
    eventTime = double(eventTS.Time(:));
    eventRise = find(diff([0;eventData>0.5])==1);
    fprintf('\nOnline replanning pulses: %d\n',numel(eventRise));
    for k=1:numel(eventRise)
        fprintf('  Replan pulse %d at %.6f s\n',k,eventTime(eventRise(k)));
    end
else
    fprintf('onlineReplanEventLog was not returned.\n');
end

% Report whether rover XY and all scalar steering signals were captured.
if all(ismember({'mb_px','mb_py'},returnedNames))
    px = ablationOutput.get('mb_px');
    py = ablationOutput.get('mb_py');
    fprintf('\nRover position samples: X=%d, Y=%d\n',numel(px.Time),numel(py.Time));
    fprintf('Rover initial XY: [%.4f %.4f] m\n', ...
        double(px.Data(1)),double(py.Data(1)));
    fprintf('Rover final XY:   [%.4f %.4f] m\n', ...
        double(px.Data(end)),double(py.Data(end)));
end

scalarNames = variableNames(1:4);
if all(ismember(scalarNames,returnedNames))
    ts = cell(1,4);
    for k=1:4
        ts{k}=ablationOutput.get(scalarNames{k});
        assert(isa(ts{k},'timeseries'),'Expected timeseries output: %s',scalarNames{k});
    end
    commonTime=double(ts{1}.Time(:));
    scalarData=nan(numel(commonTime),4);
    for k=1:4
        t=double(ts{k}.Time(:));
        d=squeeze(double(ts{k}.Data));
        d=d(:);
        if numel(t)==numel(commonTime) && ...
                all(abs(t-commonTime)<=1e-12*max(1,max(abs(commonTime))))
            scalarData(:,k)=d;
        else
            scalarData(:,k)=interp1(t,d,commonTime,'linear',nan);
        end
    end
    T=table(commonTime,scalarData(:,1),scalarData(:,2), ...
        scalarData(:,3),scalarData(:,4), ...
        'VariableNames',{'Time_s','steerCmd_rad','deltaFiltered_rad', ...
        'deltaFL_rad','deltaFR_rad'});
    writetable(T,csvFile);
    fprintf('Steering CSV: %s (%d samples)\n',csvFile,height(T));

    windows=[14.5 16.5;28.5 min(32,execInfo.StopEventTime)];
    labels={'Early Obstacle 5 activation window','Late-run comparison window'};
    for w=1:size(windows,1)
        idx=find(commonTime>=windows(w,1) & commonTime<=windows(w,2));
        fprintf('\n%s [%.3f, %.3f] s\n',labels{w},windows(w,1),windows(w,2));
        if isempty(idx)
            fprintf('  No samples in this interval.\n');
            continue;
        end
        fprintf('  steerCmd: %.5f to %.5f rad (%.2f to %.2f deg)\n', ...
            min(scalarData(idx,1),[],'omitnan'),max(scalarData(idx,1),[],'omitnan'), ...
            rad2deg(min(scalarData(idx,1),[],'omitnan')), ...
            rad2deg(max(scalarData(idx,1),[],'omitnan')));
        fprintf('  deltaFL : %.5f to %.5f rad\n', ...
            min(scalarData(idx,3),[],'omitnan'),max(scalarData(idx,3),[],'omitnan'));
        fprintf('  deltaFR : %.5f to %.5f rad\n', ...
            min(scalarData(idx,4),[],'omitnan'),max(scalarData(idx,4),[],'omitnan'));
    end
else
    fprintf('One or more scalar steering outputs were not returned; inspect MAT output.\n');
end

if strcmp(execInfo.StopEvent,'ReachedStopTime') && abs(execInfo.StopEventTime-32)<1e-6
    fprintf('\nRESULT: TEST PASSED — simulation reached 32 s with Obstacle 5 activated early.\n');
else
    fprintf('\nRESULT: TEST DID NOT REACH 32 s — compare its stop time with the normal run.\n');
end

fprintf('\n[6/6] Safety check and cleanup...\n');
fprintf('Saved scenario file was read only: %s\n',scenarioFile);
fprintf('Original Custom model was not loaded, edited, or saved by this script.\n');
fprintf('The disposable diagnostic model file remains available: %s\n',testModelFile);
fprintf('Its temporary logger branches will be discarded when the model closes.\n');
fprintf('\n============================================================\n');
fprintf(' OBSTACLE 5 EARLY-ACTIVATION ABLATION FINISHED\n');
fprintf(' MAT: %s\n',matFile);
if isfile(csvFile), fprintf(' CSV: %s\n',csvFile); end
fprintf('============================================================\n');

%% Local cleanup helper: close only the disposable model, without saving edits
function closeDiscardModel(modelName)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
end
