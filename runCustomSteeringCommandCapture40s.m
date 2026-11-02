%% CAPTURE STEERING COMMANDS ON THE SEPARATE DIAGNOSTIC COPY (40 s)
% Captures:
%   steerCmd       - PurePursuitController output before the Transfer Fcn
%   deltaFiltered  - Transfer Fcn output entering AckermannSteering
%   deltaFL/FR     - AckermannSteering front-wheel angle commands
%   routeInput     - RouteSwitch output entering PurePursuitController
%
% Operates ONLY on the saved UpperStopProbe diagnostic copy. It does not
% change the original Custom model, six-obstacle scenario, controller code,
% solver settings, tires, terrain, or obstacle arrangement. Temporary
% To Workspace blocks are removed by closing the diagnostic copy without
% saving, then reloading it only if it was open before this script started.

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
testDuration = 40.0;

assert(isfile(diagModelFile),'Diagnostic model copy not found: %s',diagModelFile);
assert(isfile(scenarioFile),'Six-obstacle scenario not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    'Original Custom model must remain loaded; this script will not load or alter it.');

sourceDirtyBefore = get_param(sourceModelName,'Dirty');
sourceFileBefore = get_param(sourceModelName,'FileName');
wasDiagLoaded = bdIsLoaded(diagModelName);
if wasDiagLoaded
    assert(strcmp(get_param(diagModelName,'Dirty'),'off'), ...
        ['Diagnostic model is dirty. Refusing to discard existing in-memory edits. ' ...
         'Close/reload only the diagnostic copy without saving, then rerun.']);
else
    load_system(diagModelFile);
end
cleanupDiag = onCleanup(@() restoreDiagnosticCopy(diagModelName,wasDiagLoaded,diagModelFile)); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' STEERING COMMAND CAPTURE — 40 s\n');
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original model dirty flag: %s\n',sourceDirtyBefore);

%% 1. Restore project workspace and six-obstacle inputs
fprintf('\n[1/5] Restoring project workspace and six-obstacle inputs...\n');
run(setupFile);
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
goalX = goalPoseWorkflow(1); %#ok<NASGU>
goalY = goalPoseWorkflow(2); %#ok<NASGU>
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
fprintf('Scenario: %s | obstacles: %s | initial route: %.3f m\n', ...
    scenarioConfig.name,mat2str([obs.id]),initialRouteLength);

%% 2. Resolve the four scalar steering signals and the active waypoint input
fprintf('\n[2/5] Resolving steering and route signal ports...\n');
ppPath = [diagModelName '/PurePursuitController'];
tfPath = [diagModelName '/Transfer Fcn'];
ackPath = [diagModelName '/AckermannSteering'];
routePath = [diagModelName '/RouteSwitch'];

requiredBlocks = {ppPath,tfPath,ackPath,routePath};
for k = 1:numel(requiredBlocks)
    assert(getSimulinkBlockHandle(requiredBlocks{k}) ~= -1, ...
        'Required block not found: %s',requiredBlocks{k});
end
ppPH = get_param(ppPath,'PortHandles');
tfPH = get_param(tfPath,'PortHandles');
ackPH = get_param(ackPath,'PortHandles');
routePH = get_param(routePath,'PortHandles');
assert(numel(ppPH.Outport) >= 1,'PurePursuitController output 1 not found.');
assert(numel(tfPH.Outport) >= 1,'Transfer Fcn output 1 not found.');
assert(numel(ackPH.Outport) >= 2,'AckermannSteering outputs 1 and 2 not found.');
assert(numel(routePH.Outport) >= 1,'RouteSwitch output 1 not found.');

signalSources = { ...
    ppPH.Outport(1), ...
    tfPH.Outport(1), ...
    ackPH.Outport(1), ...
    ackPH.Outport(2), ...
    routePH.Outport(1)};
variableNames = {'steerCmdProbe','deltaFilteredProbe', ...
    'deltaFLProbe','deltaFRProbe','routeInputProbe'};
prettyNames = {'Pure Pursuit steerCmd (rad)', ...
    'filtered delta into Ackermann (rad)', ...
    'Ackermann deltaFL (rad)','Ackermann deltaFR (rad)', ...
    'RouteSwitch active waypoint matrix'};

%% 3. Add temporary To Workspace branches in memory only
fprintf('\n[3/5] Adding temporary signal loggers...\n');
load_system('simulink');
toWorkspaceLib = 'simulink/Sinks/To Workspace';
loggerPaths = cell(size(variableNames));
baseX = 1450; baseY = 100; rowGap = 75;
for k = 1:numel(variableNames)
    loggerPaths{k} = [diagModelName '/Probe_ToWorkspace_' variableNames{k}];
    assert(getSimulinkBlockHandle(loggerPaths{k}) == -1, ...
        'Temporary logger already exists: %s',loggerPaths{k});
    add_block(toWorkspaceLib,loggerPaths{k}, ...
        'Position',[baseX baseY+(k-1)*rowGap baseX+170 baseY+35+(k-1)*rowGap], ...
        'VariableName',variableNames{k}, ...
        'SaveFormat','Timeseries');
    loggerPH = get_param(loggerPaths{k},'PortHandles');
    assert(~isempty(loggerPH.Inport),'Logger has no input port: %s',loggerPaths{k});
    add_line(diagModelName,signalSources{k},loggerPH.Inport(1),'autorouting','on');
    fprintf('Logger connected: %-22s <- %s\n', ...
        variableNames{k},prettyNames{k});
end

%% 4. Update and run the diagnostic copy
fprintf('\n[4/5] Updating and running the 40 s diagnostic copy...\n');
set_param(diagModelName,'SimulationCommand','update');
stamp = datestr(now,'yyyymmdd_HHMMSS');
logName = ['simlog_SteeringCommandCapture_' stamp];
matFile = fullfile(projectRoot,['capturedSteeringCommands40s_' stamp '.mat']);

simIn = Simulink.SimulationInput(diagModelName);
simIn = simIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);
steeringProbeOutput = sim(simIn);
assignin('base','steeringProbeOutput',steeringProbeOutput);

%% 5. Save output, summarize signals and write scalar CSV
fprintf('\n[5/5] Saving and summarizing steering commands...\n');
returnedNames = steeringProbeOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));
try
    execInfo = steeringProbeOutput.SimulationMetadata.ExecutionInfo;
    fprintf('Stop event: %s | stop time: %.12f s\n', ...
        execInfo.StopEvent,execInfo.StopEventTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end
try
    err = char(steeringProbeOutput.ErrorMessage);
    if ~isempty(strtrim(err))
        fprintf('\nCaptured simulation error:\n%s\n',err);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end
save(matFile,'steeringProbeOutput','variableNames','prettyNames','-v7.3');
fprintf('Saved diagnostic SimulationOutput: %s\n',matFile);

% Verify all five loggers were returned.
for k = 1:numel(variableNames)
    assert(any(strcmp(returnedNames,variableNames{k})), ...
        'Expected logged output %s was not returned.',variableNames{k});
end

% Convert scalar signals to a common time base using steerCmd time.
scalarNames = variableNames(1:4);
ts = cell(1,4);
for k = 1:4
    ts{k} = steeringProbeOutput.get(scalarNames{k});
    assert(isa(ts{k},'timeseries'), ...
        'Expected %s to be a timeseries, received %s.',scalarNames{k},class(ts{k}));
end
commonTime = double(ts{1}.Time(:));
scalarData = nan(numel(commonTime),4);
for k = 1:4
    t = double(ts{k}.Time(:));
    d = double(ts{k}.Data);
    d = squeeze(d);
    if isvector(d)
        d = d(:);
    end
    assert(size(d,2) == 1 && size(d,1) == numel(t), ...
        'Expected scalar data for %s; received data size %s.', ...
        scalarNames{k},mat2str(size(ts{k}.Data)));
    if numel(t) == numel(commonTime) && ...
            all(abs(t-commonTime) <= 1e-12*max(1,max(abs(commonTime))))
        scalarData(:,k) = d;
    else
        scalarData(:,k) = interp1(t,d,commonTime,'linear',nan);
    end
end
signalTable = table(commonTime,scalarData(:,1),scalarData(:,2), ...
    scalarData(:,3),scalarData(:,4), ...
    'VariableNames',{'Time_s','steerCmd_rad','deltaFiltered_rad', ...
    'deltaFL_rad','deltaFR_rad'});
csvFile = fullfile(projectRoot,['steeringCommandValues_' stamp '.csv']);
writetable(signalTable,csvFile);
fprintf('Scalar steering samples: %d | CSV: %s\n',height(signalTable),csvFile);

% Summaries use actual sample spacing; ignore near-zero solver steps when
% reporting finite-difference slopes so tiny hmin steps do not dominate.
windows = [24.8 26.5; 29.2 min(testDuration,29.624814507405514)];
windowLabels = {'Steering manoeuvre window','Final solver-failure window'};
for w = 1:size(windows,1)
    idx = find(commonTime >= windows(w,1) & commonTime <= windows(w,2));
    fprintf('\n--- %s [%.6f, %.6f] s ---\n', ...
        windowLabels{w},windows(w,1),windows(w,2));
    if isempty(idx)
        fprintf('No samples in this window.\n');
        continue;
    end
    local = scalarData(idx,:);
    fprintf('steerCmd range : %.6f to %.6f rad (%.3f to %.3f deg)\n', ...
        min(local(:,1),[],'omitnan'),max(local(:,1),[],'omitnan'), ...
        rad2deg(min(local(:,1),[],'omitnan')),rad2deg(max(local(:,1),[],'omitnan')));
    fprintf('filtered delta : %.6f to %.6f rad\n', ...
        min(local(:,2),[],'omitnan'),max(local(:,2),[],'omitnan'));
    fprintf('deltaFL        : %.6f to %.6f rad (%.3f to %.3f deg)\n', ...
        min(local(:,3),[],'omitnan'),max(local(:,3),[],'omitnan'), ...
        rad2deg(min(local(:,3),[],'omitnan')),rad2deg(max(local(:,3),[],'omitnan')));
    fprintf('deltaFR        : %.6f to %.6f rad (%.3f to %.3f deg)\n', ...
        min(local(:,4),[],'omitnan'),max(local(:,4),[],'omitnan'), ...
        rad2deg(min(local(:,4),[],'omitnan')),rad2deg(max(local(:,4),[],'omitnan')));
    % Show beginning/end samples without flooding the command window.
    n = min(8,numel(idx));
    pick = unique([idx(1:n);idx(max(1,numel(idx)-n+1):end)]);
    disp(signalTable(pick,:));
end

% Route matrix: retain full time history in the MAT; print only dimensions.
routeTS = steeringProbeOutput.get('routeInputProbe');
assert(isa(routeTS,'timeseries'),'routeInputProbe is not a timeseries.');
fprintf('\nRouteSwitch log: %d samples | data dimensions %s\n', ...
    numel(routeTS.Time),mat2str(size(routeTS.Data)));

fprintf('\nOriginal model dirty flag before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));
assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original model dirty flag changed unexpectedly.');

fprintf('\n============================================================\n');
fprintf(' STEERING COMMAND CAPTURE FINISHED\n');
fprintf(' Original Custom model and scenario were not modified/saved.\n');
fprintf(' Diagnostic copy will be restored from its saved file.\n');
fprintf(' MAT: %s\nCSV: %s\n',matFile,csvFile);
fprintf('============================================================\n');

%% Local cleanup helper
function restoreDiagnosticCopy(modelName,wasLoaded,modelFile)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
    if wasLoaded
        load_system(modelFile);
    end
end
