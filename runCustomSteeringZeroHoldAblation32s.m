%% STEERING-HOLD ABLATION — 32 s
% Diagnostic-only test:
% Hold the two front steering joints at zero by replacing the signal
% entering AckermannSteering with a Constant(0). All other scenario,
% terrain, tire, obstacle, chassis and solver settings remain unchanged.
%
% This is an isolation test, NOT a candidate demonstration configuration.
% The rover will not follow the planned route while steering is held.
% All edits are in memory on the separate UpperStopProbe copy. The original
% Custom model is never saved, closed, or modified by this script.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

sourceModelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
diagModelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
diagModelFile = fullfile(projectRoot,[diagModelName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
setupFile = fullfile(projectRoot,'setupOffRoadProject.m');

testDuration = 32.0;

assert(isfile(diagModelFile),'Diagnostic model copy not found: %s',diagModelFile);
assert(isfile(scenarioFile),'Six-obstacle scenario not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    'Original Custom model must remain loaded. This script will not load or alter it.');

sourceDirtyBefore = get_param(sourceModelName,'Dirty');
sourceFileBefore = get_param(sourceModelName,'FileName');

wasDiagLoaded = bdIsLoaded(diagModelName);
if wasDiagLoaded
    assert(strcmp(get_param(diagModelName,'Dirty'),'off'), ...
        ['Diagnostic copy is dirty. Refusing to discard in-memory edits. ' ...
         'Close/reload only the diagnostic copy without saving, then rerun.']);
else
    load_system(diagModelFile);
end
cleanupDiag = onCleanup(@() restoreDiagnosticCopy(diagModelName,wasDiagLoaded,diagModelFile)); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' FRONT STEERING ZERO-HOLD ABLATION — %.1f s\n',testDuration);
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original Custom dirty flag: %s\n',sourceDirtyBefore);
fprintf('Diagnostic copy was loaded at start: %d\n',wasDiagLoaded);

%% 1. Restore the exact six-obstacle workspace/scenario inputs
fprintf('\n[1/5] Restoring project workspace and six-obstacle inputs...\n');
run(setupFile);

if ~bdIsLoaded(diagModelName)
    fprintf('Workspace setup unloaded the diagnostic copy; reloading saved copy...\n');
    load_system(diagModelFile);
end
assert(strcmp(get_param(diagModelName,'Dirty'),'off'), ...
    'Diagnostic model became dirty during workspace setup; refusing to proceed.');

S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from scenario MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);

assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Unexpected static/dynamic obstacle roles.');
assert(abs(double(obs(3).spawnTime)-7) < 1e-9 && ...
       abs(double(obs(5).spawnTime)-15) < 1e-9, ...
    'Expected obstacle 3 at 7 s and obstacle 5 at 15 s.');

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

staticObs = obs(~[obs.isDynamic]);
staticCenters = vertcat(staticObs.centerXY);
[originalPathPadded,initialWaypointCount,~,initialRouteLength] = ...
    onlineReplanFromPoseGeometryAware(startPoseWorkflow(1:2),goalXY, ...
    staticCenters(:,1).',staticCenters(:,2).', ...
    obstacleClearance,maxWaypoints);
assert(initialWaypointCount >= 2,'Initial route calculation failed.');
waypoints = originalPathPadded(1:initialWaypointCount,:); %#ok<NASGU>
newPath = waypoints; %#ok<NASGU>

fprintf('Scenario: %s\n',scenarioConfig.name);
fprintf('Obstacle IDs: %s | initial path length: %.3f m\n', ...
    mat2str([obs.id]),initialRouteLength);
fprintf('Physical bodies: 1, 2, 3 | virtual planning obstacles: 4, 5, 6\n');

%% 2. Resolve the existing Transfer Fcn -> AckermannSteering line
fprintf('\n[2/5] Resolving steering input line...\n');
ackPath = [diagModelName '/AckermannSteering'];
transferPath = [diagModelName '/Transfer Fcn'];

assert(getSimulinkBlockHandle(ackPath) ~= -1, ...
    'AckermannSteering subsystem not found: %s',ackPath);
assert(getSimulinkBlockHandle(transferPath) ~= -1, ...
    'Transfer Fcn block not found: %s',transferPath);

ackPH = get_param(ackPath,'PortHandles');
tfPH = get_param(transferPath,'PortHandles');
assert(~isempty(ackPH.Inport) && ~isempty(tfPH.Outport), ...
    'Steering input/output ports could not be resolved.');

ackIn = ackPH.Inport(1);
lineHandle = get_param(ackIn,'Line');
assert(~isempty(lineHandle) && lineHandle ~= -1, ...
    'AckermannSteering input is not connected; refusing to guess a connection.');

srcPort = get_param(lineHandle,'SrcPortHandle');
dstPort = get_param(lineHandle,'DstPortHandle');
assert(isscalar(srcPort) && isscalar(dstPort), ...
    'Expected one source and one destination on the Ackermann input line.');

sourceBlock = get_param(srcPort,'Parent');
fprintf('Existing Ackermann input source: %s\n',sourceBlock);
assert(strcmp(sourceBlock,transferPath), ...
    ['Expected Transfer Fcn to feed AckermannSteering directly. ' ...
     'Actual source is %s; stopping without edits.'],sourceBlock);

%% 3. Replace only Ackermann input with a constant zero command
fprintf('\n[3/5] Applying zero-steering command on diagnostic copy...\n');
zeroBlock = [diagModelName '/SteeringZeroHoldProbe'];
assert(getSimulinkBlockHandle(zeroBlock) == -1, ...
    'Temporary zero-hold block already exists.');

tfPos = get_param(transferPath,'Position');
ackPos = get_param(ackPath,'Position');
x = min(tfPos(3)+70,ackPos(1)-180);
if x < tfPos(3)+25
    x = tfPos(3)+45;
end
y = round((tfPos(2)+tfPos(4))/2);

add_block('simulink/Sources/Constant',zeroBlock, ...
    'Value','0', ...
    'Position',[x y x+55 y+35]);

% Delete the existing Transfer Fcn -> Ackermann input branch and wire zero.
delete_line(diagModelName,srcPort,dstPort);

zeroPH = get_param(zeroBlock,'PortHandles');
assert(~isempty(zeroPH.Outport),'Zero-hold Constant output port missing.');
add_line(diagModelName,zeroPH.Outport(1),ackIn,'autorouting','on');

% Verify the new source is the zero Constant.
lineAfter = get_param(ackIn,'Line');
assert(~isempty(lineAfter) && lineAfter ~= -1, ...
    'Zero-hold input connection was not created.');
verifiedSrc = get_param(lineAfter,'SrcPortHandle');
verifiedBlock = get_param(verifiedSrc,'Parent');
assert(strcmp(verifiedBlock,zeroBlock), ...
    'AckermannSteering is not connected to the zero-hold Constant.');
fprintf('Verified: AckermannSteering input = constant 0 rad.\n');
fprintf('Only the steering command input was changed on the diagnostic copy.\n');

%% 4. Run only long enough to pass the known failure time
fprintf('\n[4/5] Updating and running %.1f s zero-steering ablation...\n',testDuration);
set_param(diagModelName,'SimulationCommand','update');

stamp = datestr(now,'yyyymmdd_HHMMSS');
logName = ['simlog_SteeringZeroHold_' stamp];
matFile = fullfile(projectRoot,['capturedSteeringZeroHold32s_' stamp '.mat']);

testIn = Simulink.SimulationInput(diagModelName);
testIn = testIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);

zeroHoldOutput = sim(testIn);
assignin('base','zeroHoldOutput',zeroHoldOutput);

%% 5. Save output and report whether it passed the previous failure time
fprintf('\n[5/5] Saving and reporting ablation result...\n');
returnedNames = zeroHoldOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));

stopEvent = 'Unknown';
stopTime = NaN;
try
    execInfo = zeroHoldOutput.SimulationMetadata.ExecutionInfo;
    stopEvent = char(execInfo.StopEvent);
    stopTime = double(execInfo.StopEventTime);
    fprintf('Stop event: %s | stop time: %.12f s\n',stopEvent,stopTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end

try
    err = char(zeroHoldOutput.ErrorMessage);
    if ~isempty(strtrim(err))
        fprintf('\nCaptured simulation error:\n%s\n',err);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end

save(matFile,'zeroHoldOutput','scenarioConfig','testDuration','-v7.3');
fprintf('Saved SimulationOutput: %s\n',matFile);

if isfinite(stopTime) && stopTime >= 31.999
    fprintf('\nRESULT: PASSED THE PREVIOUS 29.6248 s FAILURE POINT.\n');
    fprintf('Steering command is strongly implicated; next test should restore steering with a smoother command.\n');
else
    fprintf('\nRESULT: DID NOT REACH %.1f s.\n',testDuration);
    fprintf('Compare its stop time with 29.6248 s before drawing a conclusion.\n');
end

assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original Custom model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original Custom model dirty flag changed unexpectedly.');
fprintf('\nOriginal Custom model dirty flag before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));

fprintf('\n============================================================\n');
fprintf(' STEERING ZERO-HOLD ABLATION FINISHED\n');
fprintf(' This is a diagnostic run, not a path-following demonstration.\n');
fprintf(' Original Custom model and six-obstacle scenario were not saved or modified.\n');
fprintf(' Diagnostic copy will be restored from its saved file.\n');
fprintf(' MAT: %s\n',matFile);
fprintf('============================================================\n');

%% Local cleanup
function restoreDiagnosticCopy(modelName,wasLoaded,modelFile)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
    if wasLoaded
        load_system(modelFile);
    end
end
