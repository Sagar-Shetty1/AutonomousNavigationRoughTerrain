%% SIX-OBSTACLE MODEL — OBSTACLE 5 PRESENT FROM START ABLATION (32 s)
% Purpose:
%   Separate "Obstacle 5 becoming active at 15 s" from "the planner having
%   to route around Obstacle 5".
%
% Diagnostic scenario:
%   - Keep all six obstacle IDs, positions and dimensions.
%   - Keep physical Obstacle 3 activation at 7 s.
%   - Treat virtual-only Obstacle 5 as STATIC from t=0 in this test only.
%   - Include Obstacle 5 in the initial A* route calculation.
%   - Preserve normal steering, terrain, tires, chassis and solver settings.
%
% This changes no saved scenario MAT file and does not modify the source model.
% A new disposable model copy is created from the saved UpperStopProbe model.
%
% Run from the project root:
%   runCustomObstacle5StaticFromStartAblation32s

%% 0. Paths and unique diagnostic copy
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

sourceProbeName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
sourceProbeFile = fullfile(projectRoot,[sourceProbeName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot,'setupOffRoadProject.m');

assert(isfile(sourceProbeFile),'Saved diagnostic source model not found: %s',sourceProbeFile);
assert(isfile(scenarioFile),'Six-obstacle scenario not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup script not found: %s',setupFile);

stamp = datestr(now,'yyyymmdd_HHMMSS');
testModelName = ['OffRoadMultibodyStage1_Obstacle5StaticFromStart_' stamp];
testModelFile = fullfile(projectRoot,[testModelName '.slx']);
assert(~bdIsLoaded(testModelName) && ~isfile(testModelFile), ...
    'Unique diagnostic model name/file already exists. Rerun for a new name.');

fprintf('\n============================================================\n');
fprintf(' OBSTACLE 5 STATIC-FROM-START ABLATION — 32 s\n');
fprintf('============================================================\n');
fprintf('Source diagnostic model: %s\n',sourceProbeName);
fprintf('New disposable copy:     %s\n',testModelName);

%% 1. Restore project workspace and load scenario
fprintf('\n[1/6] Restoring project workspace and six-obstacle scenario...\n');
run(setupFile);
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from scenario MAT file.');
scenarioConfig = S.scenarioConfig;

assert(numel(scenarioConfig.obstacles)==6,'Expected exactly six obstacle definitions.');
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Original scenario static/dynamic roles differ from expected.');
assert(abs(double(obs(3).spawnTime)-7)<1e-9, ...
    'Obstacle 3 activation must remain at 7 s.');
assert(abs(double(obs(5).spawnTime)-15)<1e-9, ...
    'Original Obstacle 5 activation must be 15 s.');

% Change only the in-memory test definition for virtual obstacle 5.
% Its geometry and coordinates remain unchanged.
idx5 = find([obs.id]==5,1);
obs(idx5).isDynamic = false;
obs(idx5).spawnTime = 0.0;
scenarioConfigTest = scenarioConfig;
scenarioConfigTest.obstacles = obs;

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalXY = goalPoseWorkflow(1:2);
goalX = goalXY(1); %#ok<NASGU>
goalY = goalXY(2); %#ok<NASGU>

allCenters = vertcat(obs.centerXY);
obstacleXs = [allCenters(:,1).' nan(1,4)]; %#ok<NASGU>
obstacleYs = [allCenters(:,2).' nan(1,4)]; %#ok<NASGU>
obstacleSpawnTime = [[obs.spawnTime] nan(1,4)]; %#ok<NASGU>
obstacle3X = double(obs(3).centerXY(1)); %#ok<NASGU>
obstacle3Y = double(obs(3).centerXY(2)); %#ok<NASGU>
obstacleClearance = double(scenarioConfig.planning.obstacleClearance);
maxWaypoints = double(scenarioConfig.planning.maxWaypoints);

% Include Obstacle 5 in the initial static-obstacle planning problem.
staticObs = obs(~[obs.isDynamic]);
assert(isequal([staticObs.id],[1 2 4 5 6]), ...
    'Expected initial route to include static IDs 1,2,4,5,6.');
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded,initialWaypointCount,~,initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2),goalXY, ...
    staticCenters(:,1).',staticCenters(:,2).', ...
    obstacleClearance,maxWaypoints);
assert(initialWaypointCount>=2,'Initial route calculation failed.');
waypoints = originalPathPadded(1:initialWaypointCount,:); %#ok<NASGU>
newPath = waypoints; %#ok<NASGU>

fprintf('Scenario: %s\n',scenarioConfig.name);
fprintf('All obstacle IDs retained: %s\n',mat2str([obs.id]));
fprintf('Physical bodies: IDs 1,2,3 | virtual obstacles: IDs 4,5,6\n');
fprintf('Obstacle 3 physical activation: %.1f s (unchanged)\n',obstacleSpawnTime(3));
fprintf('Obstacle 5 test role: static from t=0 (originally dynamic at 15 s)\n');
fprintf('Obstacle 5 XY and dimensions: [%.4f %.4f] m; [%.4f %.4f %.4f] m\n', ...
    obs(idx5).centerXY(1),obs(idx5).centerXY(2), ...
    obs(idx5).dimensions(1),obs(idx5).dimensions(2),obs(idx5).dimensions(3));
fprintf('Initial route includes obstacle 5: %d waypoints, %.3f m\n', ...
    initialWaypointCount,initialRouteLength);

%% 2. Verify runtime obstacle inputs
assert(obstacleSpawnTime(3)==7.0 && obstacleSpawnTime(5)==0.0, ...
    'Runtime spawn-time test values are incorrect.');
fprintf('Runtime obstacleSpawnTime(5) = %.1f s; obstacleSpawnTime(3) = %.1f s.\n', ...
    obstacleSpawnTime(5),obstacleSpawnTime(3));

%% 3. Create a fresh diagnostic model copy
fprintf('\n[2/6] Creating isolated diagnostic model copy...\n');
new_system(testModelName,'FromFile',sourceProbeFile);
cleanupCopy = onCleanup(@() closeDiscardModel(testModelName)); %#ok<NASGU>
save_system(testModelName,testModelFile);

requiredBlocks = {'PurePursuitController','Transfer Fcn','AckermannSteering', ...
    'RouteSwitch','Obstacle3SpawnJoint','DynamicObstacleEnvironment'};
for k=1:numel(requiredBlocks)
    bp=[testModelName '/' requiredBlocks{k}];
    assert(getSimulinkBlockHandle(bp)~=-1,'Required block missing: %s',bp);
end

%% 4. Add temporary signal loggers
fprintf('\n[3/6] Adding temporary steering and route loggers...\n');
load_system('simulink');
ppPH=get_param([testModelName '/PurePursuitController'],'PortHandles');
tfPH=get_param([testModelName '/Transfer Fcn'],'PortHandles');
ackPH=get_param([testModelName '/AckermannSteering'],'PortHandles');
routePH=get_param([testModelName '/RouteSwitch'],'PortHandles');

assert(numel(ppPH.Outport)>=1 && numel(tfPH.Outport)>=1 && ...
    numel(ackPH.Outport)>=2 && numel(routePH.Outport)>=1, ...
    'Expected steering/route ports unavailable.');

signalSources={ppPH.Outport(1),tfPH.Outport(1), ...
    ackPH.Outport(1),ackPH.Outport(2),routePH.Outport(1)};
variableNames={'steerCmdProbe','deltaFilteredProbe', ...
    'deltaFLProbe','deltaFRProbe','routeInputProbe'};
prettyNames={'Pure Pursuit steerCmd','filtered steering command', ...
    'Ackermann deltaFL','Ackermann deltaFR','active RouteSwitch route'};

baseX=1450; baseY=100; rowGap=75;
for k=1:numel(variableNames)
    loggerPath=[testModelName '/Probe_ToWorkspace_' variableNames{k}];
    add_block('simulink/Sinks/To Workspace',loggerPath, ...
        'Position',[baseX baseY+(k-1)*rowGap baseX+170 baseY+35+(k-1)*rowGap], ...
        'VariableName',variableNames{k},'SaveFormat','Timeseries');
    loggerPH=get_param(loggerPath,'PortHandles');
    assert(~isempty(loggerPH.Inport),'Logger input missing: %s',loggerPath);
    add_line(testModelName,signalSources{k},loggerPH.Inport(1),'autorouting','on');
    fprintf('  %-22s <- %s\n',variableNames{k},prettyNames{k});
end

%% 5. Update and run 32 s with the test-only static-from-start inputs
fprintf('\n[4/6] Updating and running 32-second test...\n');
set_param(testModelName,'SimulationCommand','update');

simscapeLogName=['simlog_Obstacle5StaticFromStart_' stamp];
simIn=Simulink.SimulationInput(testModelName);
simIn=simIn.setVariable('obstacleSpawnTime',obstacleSpawnTime);
simIn=simIn.setVariable('scenarioConfig',scenarioConfigTest);
simIn=simIn.setModelParameter( ...
    'StopTime','32', ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',simscapeLogName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

staticFromStartOutput=sim(simIn);
assignin('base','staticFromStartOutput',staticFromStartOutput);

%% 6. Save outputs and report route/event history
fprintf('\n[5/6] Saving and inspecting outputs...\n');
matFile=fullfile(projectRoot,['capturedObstacle5StaticFromStart32s_' stamp '.mat']);
csvFile=fullfile(projectRoot,['steeringObstacle5StaticFromStart32s_' stamp '.csv']);
save(matFile,'staticFromStartOutput','scenarioConfig','scenarioConfigTest', ...
    'obstacleSpawnTime','variableNames','prettyNames','-v7.3');
fprintf('Saved MAT: %s\n',matFile);

returnedNames=staticFromStartOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));
execInfo=staticFromStartOutput.SimulationMetadata.ExecutionInfo;
fprintf('Stop event: %s | stop time: %.12f s\n', ...
    execInfo.StopEvent,execInfo.StopEventTime);
try
    err=char(staticFromStartOutput.ErrorMessage);
    if isempty(strtrim(err))
        fprintf('No simulation error was captured.\n');
    else
        fprintf('Captured simulation error:\n%s\n',err);
    end
catch
end

if any(strcmp(returnedNames,'onlineReplanEventLog'))
    eTS=staticFromStartOutput.get('onlineReplanEventLog');
    ed=double(eTS.Data(:)); et=double(eTS.Time(:));
    rises=find(diff([0;ed>0.5])==1);
    fprintf('\nOnline replanning pulses: %d\n',numel(rises));
    for k=1:numel(rises)
        fprintf('  Replan pulse %d at %.6f s\n',k,et(rises(k)));
    end
else
    fprintf('onlineReplanEventLog not returned.\n');
end

if all(ismember({'mb_px','mb_py'},returnedNames))
    px=staticFromStartOutput.get('mb_px');
    py=staticFromStartOutput.get('mb_py');
    fprintf('\nRover initial XY: [%.4f %.4f] m\n',double(px.Data(1)),double(py.Data(1)));
    fprintf('Rover final XY:   [%.4f %.4f] m\n',double(px.Data(end)),double(py.Data(end)));
    fprintf('Rover samples: X=%d, Y=%d\n',numel(px.Time),numel(py.Time));
end

scalarNames=variableNames(1:4);
if all(ismember(scalarNames,returnedNames))
    ts=cell(1,4);
    for k=1:4
        ts{k}=staticFromStartOutput.get(scalarNames{k});
        assert(isa(ts{k},'timeseries'),'Expected timeseries output %s.',scalarNames{k});
    end
    commonTime=double(ts{1}.Time(:));
    data=nan(numel(commonTime),4);
    for k=1:4
        tk=double(ts{k}.Time(:));
        dk=squeeze(double(ts{k}.Data)); dk=dk(:);
        if numel(tk)==numel(commonTime) && ...
           all(abs(tk-commonTime)<=1e-12*max(1,max(abs(commonTime))))
            data(:,k)=dk;
        else
            data(:,k)=interp1(tk,dk,commonTime,'linear',nan);
        end
    end
    T=table(commonTime,data(:,1),data(:,2),data(:,3),data(:,4), ...
        'VariableNames',{'Time_s','steerCmd_rad','deltaFiltered_rad','deltaFL_rad','deltaFR_rad'});
    writetable(T,csvFile);
    fprintf('Steering CSV: %s (%d samples)\n',csvFile,height(T));
end

if strcmp(execInfo.StopEvent,'ReachedStopTime') && execInfo.StopEventTime>=31.999
    fprintf('\nRESULT: PASSED 32 s with Obstacle 5 included in the initial route.\n');
else
    fprintf('\nRESULT: DID NOT reach 32 s. Stop time = %.9f s.\n',execInfo.StopEventTime);
end

fprintf('\n[6/6] Safety checks...\n');
fprintf('Original six-obstacle scenario MAT file was read only.\n');
fprintf('Source diagnostic model file was not saved or modified.\n');
fprintf('Disposable model file (without temporary logger edits): %s\n',testModelFile);
fprintf('Temporary logger edits will be discarded when the copy closes.\n');
fprintf('MAT: %s\nCSV: %s\n',matFile,csvFile);
fprintf('\n============================================================\n');
fprintf(' OBSTACLE 5 STATIC-FROM-START ABLATION FINISHED\n');
fprintf('============================================================\n');

function closeDiscardModel(modelName)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
end
