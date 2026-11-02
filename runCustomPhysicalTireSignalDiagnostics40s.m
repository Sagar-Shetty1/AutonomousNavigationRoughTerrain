%% CUSTOM SIX-OBSTACLE MODEL — LOG TIRE INPUTS/OUTPUTS AROUND SOLVER FAILURE
% Save this script in the project root beside the custom .slx model,
% setupOffRoadProject.m, and scenarioConfig_customization_physical_80s.mat.
%
% The script repeats the 40 s diagnostic run with signal logging enabled
% on each tire model's five inputs, three outputs, and outer contact outputs.
% It does NOT change obstacle positions, tire equations, solver settings,
% model structure, or save the Simulink model/scenario.
%
% The partial result is saved as:
% capturedTireSignals40s_20261031.mat

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
cd(projectRoot);
addpath(projectRoot);

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';
modelFile = fullfile(projectRoot,[modelName '.slx']);
scenarioFile = fullfile(projectRoot,'scenarioConfig_customization_physical_80s.mat');
testDuration = 40.0;
simscapeLogName = 'simlog_CustomTireSignalDiag40s';
outputName = 'simOutCustomTireSignalDiag40s';

assert(isfile(modelFile),'Custom model not found: %s',modelFile);
assert(isfile(scenarioFile),'Custom scenario not found: %s',scenarioFile);
assert(isfile(fullfile(projectRoot,'setupOffRoadProject.m')), ...
    'setupOffRoadProject.m not found in project root.');

fprintf('\n============================================================\n');
fprintf(' CUSTOM MODEL — TIRE SIGNAL DIAGNOSTIC REPEAT (40 s)\n');
fprintf('============================================================\n');

%% 1. Restore the same project workspace and six-obstacle scenario
fprintf('\n[1/5] Restoring project workspace and saved scenario...\n');
run(fullfile(projectRoot,'setupOffRoadProject.m'));
S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,ord] = sort([obs.id]);
obs = obs(ord);
assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]),[false false true false true false]), ...
    'Obstacle roles do not match the saved six-obstacle scenario.');

startPoseWorkflow = double(scenarioConfig.startPose(:).');
goalPoseWorkflow = double(scenarioConfig.goalPose(:).');
goalX = goalPoseWorkflow(1); %#ok<NASGU>
goalY = goalPoseWorkflow(2); %#ok<NASGU>
goalXY = goalPoseWorkflow(1:2);

%% 2. Reconstruct model chart/constant workspace variables (same as prior runner)
fprintf('\n[2/5] Reconstructing obstacle and route inputs...\n');
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
fprintf('Six obstacles validated; initial route %.3f m (%d waypoints).\n', ...
    initialRouteLength,initialWaypointCount);
fprintf('Dynamic obstacle activation times: ID 3 = %.3f s; ID 5 = %.3f s.\n', ...
    obs(3).spawnTime,obs(5).spawnTime);

%% 3. Load model and temporarily mark tire signals for logging
fprintf('\n[3/5] Preparing tire signal logging...\n');
wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelFile);
end

logPorts = [];
oldDataLogging = {};
oldNameMode = {};
oldDataName = {};

corners = {'FL','FR','RL','RR'};
inputLabels = {'vx','vy','rho','rhoDot','omega'};
outputLabels = {'Fx','Fy','Fz'};

try
    for c = 1:numel(corners)
        corner = corners{c};
        tirePath = [modelName '/RoverTireContact_' corner];
        tireModelPath = [tirePath '/RoverTireModel_' corner];

        assert(getSimulinkBlockHandle(tirePath) ~= -1, ...
            'Tire subsystem not found: %s',tirePath);
        assert(getSimulinkBlockHandle(tireModelPath) ~= -1, ...
            'Tire force model not found: %s',tireModelPath);

        modelPorts = get_param(tireModelPath,'PortHandles');
        assert(numel(modelPorts.Inport) == 5, ...
            '%s expected 5 model inputs, found %d.',corner,numel(modelPorts.Inport));
        assert(numel(modelPorts.Outport) == 3, ...
            '%s expected 3 model outputs, found %d.',corner,numel(modelPorts.Outport));

        % Log the source signal feeding each tire model input.
        for p = 1:numel(modelPorts.Inport)
            lineHandle = get_param(modelPorts.Inport(p),'Line');
            if isempty(lineHandle) || lineHandle == -1
                warning('%s input %d (%s) has no connected signal line.', ...
                    corner,p,inputLabels{p});
                continue;
            end
            sourcePort = get_param(lineHandle,'SrcPortHandle');
            if isempty(sourcePort) || sourcePort(1) == -1
                warning('%s input %d (%s) has no valid source port.', ...
                    corner,p,inputLabels{p});
                continue;
            end
            logName = sprintf('Tire_%s_%s',corner,inputLabels{p});
            [logPorts,oldDataLogging,oldNameMode,oldDataName] = ...
                markTireSignalPort(sourcePort(1),logName, ...
                logPorts,oldDataLogging,oldNameMode,oldDataName);
            fprintf('  Marked %s\n',logName);
        end

        % Log Fx, Fy, Fz generated by each tire force model.
        for p = 1:numel(modelPorts.Outport)
            logName = sprintf('Tire_%s_Model_%s',corner,outputLabels{p});
            [logPorts,oldDataLogging,oldNameMode,oldDataName] = ...
                markTireSignalPort(modelPorts.Outport(p),logName, ...
                logPorts,oldDataLogging,oldNameMode,oldDataName);
            fprintf('  Marked %s\n',logName);
        end

        % Also log the external outputs of each tire subsystem (Contact, Fz, Fx).
        outerPorts = get_param(tirePath,'PortHandles');
        outportBlocks = find_system(tirePath,'SearchDepth',1,'BlockType','Outport');
        for b = 1:numel(outportBlocks)
            portNumber = str2double(get_param(outportBlocks{b},'Port'));
            portLabel = get_param(outportBlocks{b},'Name');
            if isfinite(portNumber) && portNumber >= 1 && ...
                    portNumber <= numel(outerPorts.Outport)
                logName = sprintf('Tire_%s_Port_%s',corner,portLabel);
                [logPorts,oldDataLogging,oldNameMode,oldDataName] = ...
                    markTireSignalPort(outerPorts.Outport(portNumber),logName, ...
                    logPorts,oldDataLogging,oldNameMode,oldDataName);
                fprintf('  Marked %s\n',logName);
            end
        end
    end

    fprintf('\nTotal unique logged Simulink output ports: %d\n',numel(logPorts));

    %% 4. Run same 40 s diagnostic with CaptureErrors, now including logsout
    fprintf('\n[4/5] Running the 40 s diagnostic with tire signal capture...\n');
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter( ...
        'StopTime',sprintf('%.15g',testDuration), ...
        'CaptureErrors','on', ...
        'SignalLogging','on', ...
        'SignalLoggingName','logsout', ...
        'SimscapeLogType','all', ...
        'SimscapeLogName',simscapeLogName, ...
        'SimscapeLogLimitData','on', ...
        'SimscapeLogDataHistory',200000);

    simOutCustomTireSignalDiag40s = sim(simIn);
    assignin('base',outputName,simOutCustomTireSignalDiag40s);

catch ME
    restoreTireSignalPorts(logPorts,oldDataLogging,oldNameMode,oldDataName);
    if ~wasLoaded && bdIsLoaded(modelName)
        close_system(modelName,0);
    end
    rethrow(ME);
end

% Restore all previous per-port logging settings. The captured data remains.
restoreTireSignalPorts(logPorts,oldDataLogging,oldNameMode,oldDataName);

%% 5. Report and save captured partial output
fprintf('\n[5/5] Reporting and saving captured output...\n');
returnedNames = simOutCustomTireSignalDiag40s.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));

execInfo = simOutCustomTireSignalDiag40s.SimulationMetadata.ExecutionInfo;
try
    fprintf('Stop event: %s\n',execInfo.StopEvent);
    fprintf('Stop time: %.9f s\n',execInfo.StopEventTime);
catch
end

if any(strcmp(returnedNames,'logsout'))
    tireDataset = simOutCustomTireSignalDiag40s.get('logsout');
    fprintf('\nCaptured tire signal dataset:\n');
    disp(tireDataset);
else
    warning('No logsout dataset was returned. Check the signal logging setup.');
end

captureFile = fullfile(projectRoot,'capturedTireSignals40s_20261031.mat');
save(captureFile,'simOutCustomTireSignalDiag40s','-v7.3');
fprintf('\nSaved diagnostic output to:\n%s\n',captureFile);

if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName,0);
end

fprintf('\n============================================================\n');
fprintf(' TIRE SIGNAL DIAGNOSTIC COMPLETE\n');
fprintf(' Existing model/scenario files were not saved or modified.\n');
fprintf(' Original per-port signal logging settings were restored.\n');
fprintf('============================================================\n');

%% Local helper functions
function [ports,oldDL,oldMode,oldName] = markTireSignalPort( ...
    portHandle,logName,ports,oldDL,oldMode,oldName)

if isempty(portHandle) || portHandle == -1
    return;
end
if any(ports == portHandle)
    return;
end

% Save the prior state before changing this port.
ports(end+1,1) = portHandle;
oldDL{end+1,1} = get_param(portHandle,'DataLogging');
oldMode{end+1,1} = get_param(portHandle,'DataLoggingNameMode');
oldName{end+1,1} = get_param(portHandle,'DataLoggingName');

set_param(portHandle,'DataLogging','on');
set_param(portHandle,'DataLoggingNameMode','Custom');
set_param(portHandle,'DataLoggingName',logName);
end

function restoreTireSignalPorts(ports,oldDL,oldMode,oldName)
for q = numel(ports):-1:1
    try
        set_param(ports(q),'DataLogging',oldDL{q});
        set_param(ports(q),'DataLoggingNameMode',oldMode{q});
        set_param(ports(q),'DataLoggingName',oldName{q});
    catch ME
        warning('Could not restore logging state for one port: %s',ME.message);
    end
end
end
