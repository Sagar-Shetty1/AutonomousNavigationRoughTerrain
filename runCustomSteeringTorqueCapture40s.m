%% CAPTURE STEERING JOINT TORQUES ON THE SEPARATE DIAGNOSTIC COPY (40 s)
% Enables measurement-only sensing on the front-left and front-right
% Revolute Joints and captures:
%   t  = actuator torque at the revolute primitive
%   tt = total joint torque
%
% All changes are confined to the saved UpperStopProbe diagnostic copy
% in memory. The original Custom model, scenario, terrain, obstacle
% arrangement, controller code, and solver settings are not modified.
% The diagnostic copy is closed without saving during cleanup.

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
fprintf(' FRONT STEERING TORQUE CAPTURE — 40 s\n');
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original model dirty flag: %s\n',sourceDirtyBefore);

%% 1. Restore workspace and six-obstacle scenario inputs
fprintf('\n[1/6] Restoring project workspace and six-obstacle inputs...\n');
run(setupFile);
if ~bdIsLoaded(diagModelName)
    fprintf('Workspace setup unloaded the diagnostic copy; loading its saved file...\n');
    load_system(diagModelFile);
end
assert(strcmp(get_param(diagModelName,'Dirty'),'off'), ...
    'Diagnostic copy is dirty after workspace setup; refusing to overwrite its in-memory state.');

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
fprintf('Scenario: %s | obstacles: %s | initial route: %.3f m\n', ...
    scenarioConfig.name,mat2str([obs.id]),initialRouteLength);

%% 2. Resolve and verify the front steering joints
fprintf('\n[2/6] Resolving front steering joints...\n');
jointNames = {'Steering Revolute FL','Steering Revolute FR'};
jointPaths = cellfun(@(n)[diagModelName '/' n],jointNames,'UniformOutput',false);
for k = 1:numel(jointPaths)
    assert(getSimulinkBlockHandle(jointPaths{k}) ~= -1, ...
        'Required steering joint not found: %s',jointPaths{k});
    fprintf('Found: %s\n',jointPaths{k});
end

%% 3. Enable torque sensing and identify newly exposed ports by port-handle delta
fprintf('\n[3/6] Enabling actuator and total torque sensing...\n');
sensorDefs = { ...
    'SenseTorqueForce','ActuatorTorque','t'; ...
    'SenseTotalTorque','TotalTorque','tt'};
sensorPorts = cell(2,2);
sensorPortSide = cell(2,2);
sensorPortIndex = zeros(2,2);
for j = 1:2
    prevPH = get_param(jointPaths{j},'PortHandles');
    fprintf('\n%s baseline physical ports: LConn=%d, RConn=%d\n', ...
        jointNames{j},numel(prevPH.LConn),numel(prevPH.RConn));
    for s = 1:2
        paramName = sensorDefs{s,1};
        set_param(jointPaths{j},paramName,'on');
        set_param(diagModelName,'SimulationCommand','update');
        nextPH = get_param(jointPaths{j},'PortHandles');

        newL = setdiff(nextPH.LConn(:),prevPH.LConn(:),'stable');
        newR = setdiff(nextPH.RConn(:),prevPH.RConn(:),'stable');
        if numel(newL)+numel(newR) == 1
            if numel(newL) == 1
                sensorPortSide{j,s} = 'LConn';
                sensorPortIndex(j,s) = find(nextPH.LConn(:) == newL(1),1,'first');
            else
                sensorPortSide{j,s} = 'RConn';
                sensorPortIndex(j,s) = find(nextPH.RConn(:) == newR(1),1,'first');
            end
        else
            % If Simulink recreated old handles during update, use the
            % physical-port count delta and the newly appended port index.
            dL = numel(nextPH.LConn)-numel(prevPH.LConn);
            dR = numel(nextPH.RConn)-numel(prevPH.RConn);
            assert((dL == 1 && dR == 0) || (dL == 0 && dR == 1), ...
                ['Expected exactly one new physical-signal port after enabling ' ...
                 '%s on %s. Port deltas: LConn=%d, RConn=%d.'], ...
                 paramName,jointNames{j},dL,dR);
            if dL == 1
                sensorPortSide{j,s} = 'LConn';
                sensorPortIndex(j,s) = numel(nextPH.LConn);
            else
                sensorPortSide{j,s} = 'RConn';
                sensorPortIndex(j,s) = numel(nextPH.RConn);
            end
        end
        prevPH = nextPH;
        fprintf('  %-18s (%s) -> %s(%d) exposed\n', ...
            sensorDefs{s,2},sensorDefs{s,3}, ...
            sensorPortSide{j,s},sensorPortIndex(j,s));
    end

    % Re-read handles after the final update, so the logger receives current
    % port handles even if Simulink regenerated them during model update.
    finalPH = get_param(jointPaths{j},'PortHandles');
    for s = 1:2
        sideName = sensorPortSide{j,s};
        portIndex = sensorPortIndex(j,s);
        finalSidePorts = finalPH.(sideName);
        assert(numel(finalSidePorts) >= portIndex, ...
            'Mapped sensing port disappeared: %s %s.',jointNames{j},sensorDefs{s,3});
        newPort = finalSidePorts(portIndex);
        lineHandle = get_param(newPort,'Line');
        assert(isempty(lineHandle) || lineHandle == -1, ...
            'New sensing port %s on %s is already connected; refusing to alter it.', ...
            sensorDefs{s,3},jointNames{j});
        sensorPorts{j,s} = newPort;
    end
end

%% 4. Add PS-Simulink converters and To Workspace loggers in memory only
fprintf('\n[4/6] Connecting torque outputs to timeseries loggers...\n');
load_system('nesl_utility');
load_system('simulink');
converterLib = 'nesl_utility/PS-Simulink Converter';
toWorkspaceLib = 'simulink/Sinks/To Workspace';

variableNames = cell(1,4);
prettyNames = cell(1,4);
loggerCounter = 0;
for j = 1:2
    side = 'FL';
    if j == 2, side = 'FR'; end
    for s = 1:2
        loggerCounter = loggerCounter + 1;
        signalName = sensorDefs{s,2};
        variableNames{loggerCounter} = ...
            sprintf('steering%s%sProbe',side,signalName);
        prettyNames{loggerCounter} = ...
            sprintf('%s %s (%s; N*m)',side,signalName,sensorDefs{s,3});

        converterPath = [diagModelName '/Probe_PS_' side '_' signalName];
        loggerPath = [diagModelName '/Probe_ToWorkspace_' side '_' signalName];
        assert(getSimulinkBlockHandle(converterPath) == -1, ...
            'Temporary converter already exists: %s',converterPath);
        assert(getSimulinkBlockHandle(loggerPath) == -1, ...
            'Temporary logger already exists: %s',loggerPath);

        x0 = 1450; y0 = 120 + (loggerCounter-1)*75;
        add_block(converterLib,converterPath, ...
            'Position',[x0 y0 x0+135 y0+40]);
        add_block(toWorkspaceLib,loggerPath, ...
            'Position',[x0+190 y0 x0+350 y0+40], ...
            'VariableName',variableNames{loggerCounter}, ...
            'SaveFormat','Timeseries');

        % Set the PS-Simulink converter output unit explicitly to N*m.
        dialogParams = get_param(converterPath,'DialogParameters');
        if isfield(dialogParams,'Unit')
            set_param(converterPath,'Unit','N*m');
        elseif isfield(dialogParams,'OutputSignalUnit')
            set_param(converterPath,'OutputSignalUnit','N*m');
        else
            error('Could not identify the output-unit dialog parameter on %s.',converterPath);
        end

        converterPH = get_param(converterPath,'PortHandles');
        if ~isempty(converterPH.LConn)
            psInputPort = converterPH.LConn(1);
        elseif ~isempty(converterPH.RConn)
            psInputPort = converterPH.RConn(1);
        else
            error('PS-Simulink Converter has no physical-signal input port.');
        end
        loggerPH = get_param(loggerPath,'PortHandles');
        assert(~isempty(converterPH.Outport) && ~isempty(loggerPH.Inport), ...
            'Converter/logger Simulink ports are missing for %s.',variableNames{loggerCounter});

        add_line(diagModelName,sensorPorts{j,s},psInputPort,'autorouting','on');
        add_line(diagModelName,converterPH.Outport(1),loggerPH.Inport(1), ...
            'autorouting','on');
        fprintf('  Logger connected: %-34s <- %s\n', ...
            variableNames{loggerCounter},prettyNames{loggerCounter});
    end
end

%% 5. Update and run the 40-second diagnostic copy
fprintf('\n[5/6] Updating and running the 40 s torque diagnostic...\n');
set_param(diagModelName,'SimulationCommand','update');
stamp = datestr(now,'yyyymmdd_HHMMSS');
logName = ['simlog_SteeringTorqueCapture_' stamp];
matFile = fullfile(projectRoot,['capturedSteeringTorques40s_' stamp '.mat']);
csvFile = fullfile(projectRoot,['steeringJointTorques_' stamp '.csv']);

testIn = Simulink.SimulationInput(diagModelName);
testIn = testIn.setModelParameter( ...
    'StopTime',sprintf('%.15g',testDuration), ...
    'CaptureErrors','on', ...
    'SimscapeLogType','all', ...
    'SimscapeLogName',logName, ...
    'SimscapeLogLimitData','on', ...
    'SimscapeLogDataHistory',200000);
torqueProbeOutput = sim(testIn);
assignin('base','torqueProbeOutput',torqueProbeOutput);

%% 6. Save SimulationOutput and summarize measured torques
fprintf('\n[6/6] Saving and summarizing steering torques...\n');
returnedNames = torqueProbeOutput.who;
fprintf('Returned outputs: %s\n',strjoin(returnedNames,', '));
try
    execInfo = torqueProbeOutput.SimulationMetadata.ExecutionInfo;
    fprintf('Stop event: %s | stop time: %.12f s\n', ...
        execInfo.StopEvent,execInfo.StopEventTime);
catch ME
    fprintf('Could not read execution metadata: %s\n',ME.message);
end
try
    err = char(torqueProbeOutput.ErrorMessage);
    if ~isempty(strtrim(err))
        fprintf('\nCaptured simulation error:\n%s\n',err);
    else
        fprintf('No simulation error was captured.\n');
    end
catch
end
save(matFile,'torqueProbeOutput','variableNames','prettyNames','sensorDefs','-v7.3');
fprintf('Saved diagnostic SimulationOutput: %s\n',matFile);

% Convert all four vector torque timeseries to a common time base.
tsData = cell(1,4);
for k = 1:4
    assert(any(strcmp(returnedNames,variableNames{k})), ...
        'Expected torque output %s was not returned.',variableNames{k});
    ts = torqueProbeOutput.get(variableNames{k});
    assert(isa(ts,'timeseries'), ...
        'Expected %s to be a timeseries; received %s.',variableNames{k},class(ts));
    [tt,dd] = normalizeTorqueTimeseries(ts);
    tsData{k} = struct('Time',tt,'Data',dd);
end
commonTime = tsData{1}.Time;
columns = cell(1,1+12);
columns{1} = commonTime;
columnNames = {'Time_s'};
col = 1;
for k = 1:4
    data = tsData{k}.Data;
    if numel(tsData{k}.Time) ~= numel(commonTime) || ...
            any(abs(tsData{k}.Time-commonTime) > 1e-12*max(1,max(abs(commonTime))))
        aligned = nan(numel(commonTime),3);
        for c = 1:3
            aligned(:,c) = interp1(tsData{k}.Time,data(:,c),commonTime,'linear',nan);
        end
        data = aligned;
    end
    name = variableNames{k};
    for c = 1:3
        col = col + 1;
        columns{col} = data(:,c);
        axisName = 'X'; if c == 2, axisName = 'Y'; elseif c == 3, axisName = 'Z'; end
        columnNames{col} = [name '_' axisName '_Nm']; %#ok<AGROW>
    end
end
torqueTable = table(columns{:},'VariableNames',columnNames);
writetable(torqueTable,csvFile);
fprintf('Torque CSV saved: %s\n',csvFile);

failureWindow = commonTime >= 29.2 & commonTime <= commonTime(end);
fprintf('\nPeak torque magnitudes in final window [29.2, %.6f] s:\n',commonTime(end));
for k = 1:4
    d = tsData{k}.Data;
    local = d(failureWindow,:);
    if isempty(local), continue; end
    mags = sqrt(sum(local.^2,2));
    fprintf('  %-34s max |T| = %.6g N*m | max |Tz| = %.6g N*m\n', ...
        variableNames{k},max(mags,[],'omitnan'),max(abs(local(:,3)),[],'omitnan'));
end
fprintf('\nOriginal model dirty flag before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));
assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original model dirty flag changed unexpectedly.');

fprintf('\n============================================================\n');
fprintf(' STEERING TORQUE CAPTURE FINISHED\n');
fprintf(' Original Custom model and scenario were not modified/saved.\n');
fprintf(' Diagnostic copy changes were made in memory only.\n');
fprintf(' MAT: %s\nCSV: %s\n',matFile,csvFile);
fprintf('============================================================\n');

%% Local functions
function [time,data] = normalizeTorqueTimeseries(ts)
    time = double(ts.Time(:));
    raw = squeeze(double(ts.Data));
    n = numel(time);
    if isvector(raw) && numel(raw) == n
        % Defensive fallback if a single scalar component is returned.
        data = [raw(:), zeros(n,2)];
        return;
    end
    if size(raw,1) == n && size(raw,2) == 3
        data = raw;
    elseif size(raw,2) == n && size(raw,1) == 3
        data = raw.';
    else
        error('Unexpected torque data dimensions %s for %d time samples.', ...
            mat2str(size(raw)),n);
    end
end

function restoreDiagnosticCopy(modelName,wasLoaded,modelFile)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
    if wasLoaded
        load_system(modelFile);
    end
end
