%% SIX-OBSTACLE ROVER — SOLVER PROFILER CAPTURE (READ-ONLY DIAGNOSTIC)
% Profiles the existing solver on the disposable UpperStopProbe model copy.
% Captures solver exceptions, zero crossings, continuous states, Simscape
% states and Jacobian updates. No physical model parameters are changed.
%
% Output:
%   solverProfile_CustomSixObstacle_YYYYMMDD_HHMMSS.mat
%
% Run from the project root:
%   runCustomSolverProfiler40s

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

profileStopTime = 40.0;

assert(isfile(diagModelFile),'Diagnostic model copy not found: %s',diagModelFile);
assert(isfile(scenarioFile),'Six-obstacle scenario file not found: %s',scenarioFile);
assert(isfile(setupFile),'Workspace setup script not found: %s',setupFile);
assert(bdIsLoaded(sourceModelName), ...
    'Original Custom model must remain loaded. This script will not load or close it.');
assert(exist('solverprofiler.profileModel','file') ~= 0, ...
    'solverprofiler.profileModel is unavailable in this MATLAB installation.');

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
cleanupDiag = onCleanup(@() restoreDiagnosticCopy( ...
    diagModelName,wasDiagLoaded,diagModelFile)); %#ok<NASGU>

fprintf('\n============================================================\n');
fprintf(' SIX-OBSTACLE ROVER — SOLVER PROFILER CAPTURE\n');
fprintf('============================================================\n');
fprintf('Diagnostic model: %s\n',diagModelName);
fprintf('Original Custom Dirty: %s\n',sourceDirtyBefore);
fprintf('Diagnostic model loaded at start: %d\n',wasDiagLoaded);

%% 1. Restore workspace variables and scenario inputs
fprintf('\n[1/4] Restoring project workspace and six-obstacle inputs...\n');
run(setupFile);

% Workspace setup can unload Simulink models; reload only the diagnostic copy.
if ~bdIsLoaded(diagModelName)
    load_system(diagModelFile);
end
assert(strcmp(get_param(diagModelName,'Dirty'),'off'), ...
    'Diagnostic copy became dirty during workspace setup.');

assert(bdIsLoaded(sourceModelName), ...
    'Workspace setup unexpectedly unloaded the original Custom model.');
assert(strcmp(get_param(sourceModelName,'Dirty'),sourceDirtyBefore), ...
    'Workspace setup changed the original Custom model dirty flag.');

S = load(scenarioFile,'scenarioConfig');
assert(isfield(S,'scenarioConfig'),'scenarioConfig missing from scenario MAT file.');
scenarioConfig = S.scenarioConfig;
obs = scenarioConfig.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);

assert(isequal([obs.id],1:6),'Expected obstacle IDs 1 through 6.');
assert(isequal(logical([obs.isDynamic]), ...
    [false false true false true false]), ...
    'Static/dynamic obstacle roles differ from the established scenario.');

assert(abs(double(obs(3).spawnTime)-7) < 1e-9 && ...
       abs(double(obs(5).spawnTime)-15) < 1e-9, ...
       'Expected obstacle 3 activation at 7 s and obstacle 5 at 15 s.');

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
fprintf('Obstacle IDs: %s | initial route: %.3f m | %d waypoints\n', ...
    mat2str([obs.id]),initialRouteLength,initialWaypointCount);
fprintf('Physical bodies: IDs 1,2,3 | virtual planner obstacles: IDs 4,5,6\n');

%% 2. Confirm the diagnostic model and current solver configuration
fprintf('\n[2/4] Checking diagnostic model and solver configuration...\n');
solverBlocks = find_system(diagModelName, ...
    'SearchDepth',1,'LookUnderMasks','all','FollowLinks','on', ...
    'MaskType','Solver Configuration');

if isempty(solverBlocks)
    solverBlocks = find_system(diagModelName, ...
        'SearchDepth',1,'LookUnderMasks','all','FollowLinks','on', ...
        'ReferenceBlock','simscape/Utilities/Solver Configuration');
end
assert(~isempty(solverBlocks), ...
    'Could not locate the top-level Simscape Solver Configuration block.');

solverBlock = solverBlocks{1};
fprintf('Solver Configuration block: %s\n',solverBlock);
try
    fprintf('Simscape local solver enabled: %s\n', ...
        get_param(solverBlock,'UseLocalSolver'));
catch
    fprintf('Simscape local solver setting: unavailable via mask parameter\n');
end

try
    fprintf('Partition method setting: %s\n', ...
        get_param(solverBlock,'PartitionMethod'));
catch
    fprintf('Partition method setting: unavailable\n');
end

fprintf('Simulink solver: %s\n',get_param(diagModelName,'Solver'));
fprintf('Simulink solver type: %s\n',get_param(diagModelName,'SolverType'));
fprintf('Relative tolerance: %s\n',get_param(diagModelName,'RelTol'));
fprintf('Maximum step size: %s\n',get_param(diagModelName,'MaxStep'));

%% 3. Run Solver Profiler with all relevant state/event capture enabled
fprintf('\n[3/4] Running Solver Profiler to %.1f s...\n',profileStopTime);
stamp = datestr(now,'yyyymmdd_HHMMSS');
profileMatFile = fullfile(projectRoot, ...
    ['solverProfile_CustomSixObstacle_' stamp '.mat']);

profileResult = [];
profileError = [];

try
    profileResult = solverprofiler.profileModel( ...
        diagModelName, ...
        'StartTime',0, ...
        'StopTime',profileStopTime, ...
        'SaveStates','on', ...
        'SaveSimscapeStates','on', ...
        'SaveZCSignals','on', ...
        'SaveJacobian','on', ...
        'BufferSize',100000, ...
        'OpenSP','off', ...
        'DataFullFile',profileMatFile);

    assignin('base','customSolverProfileResult',profileResult);
catch ME
    profileError = ME;
    fprintf(2,'Solver Profiler returned an error:\n%s\n',ME.message);
end

%% 4. Report saved profile data and preserve original model state
fprintf('\n[4/4] Reporting profiling result...\n');
if ~isempty(profileResult) && isfield(profileResult,'summary')
    s = profileResult.summary;
    fieldsToPrint = {'solver','tStart','tStop','absTol','relTol', ...
        'hMax','hAverage','steps','profileTime','zcNumber', ...
        'resetNumber','jacobianNumber','exceptionNumber'};
    for k = 1:numel(fieldsToPrint)
        field = fieldsToPrint{k};
        if isfield(s,field)
            value = s.(field);
            if isnumeric(value) && isscalar(value)
                fprintf('%-18s : %.12g\n',field,value);
            else
                fprintf('%-18s : %s\n',field,string(value));
            end
        end
    end
end

if isfile(profileMatFile)
    fprintf('\nProfiler MAT file saved:\n%s\n',profileMatFile);
else
    if ~isempty(profileResult) && isfield(profileResult,'file') && isfile(profileResult.file)
        profileMatFile = profileResult.file;
        fprintf('\nProfiler MAT file saved:\n%s\n',profileMatFile);
    else
        fprintf(2,['No profiler MAT file was found at the expected location. ' ...
            'See any error above.\n']);
    end
end

assert(strcmp(sourceFileBefore,get_param(sourceModelName,'FileName')), ...
    'Original Custom model file identity changed unexpectedly.');
assert(strcmp(sourceDirtyBefore,get_param(sourceModelName,'Dirty')), ...
    'Original Custom model dirty flag changed unexpectedly.');

fprintf('\nOriginal Custom Dirty before: %s | after: %s\n', ...
    sourceDirtyBefore,get_param(sourceModelName,'Dirty'));

fprintf('\n============================================================\n');
fprintf(' SOLVER PROFILER CAPTURE FINISHED\n');
fprintf(' No solver settings, tire equations, obstacle positions or model parameters were changed.\n');
fprintf(' Original Custom model and scenario were not saved or modified.\n');
if ~isempty(profileError)
    fprintf(' Profiler reported an error; inspect the message above and any saved MAT file.\n');
end
fprintf('============================================================\n');

%% Local cleanup: restore only the disposable diagnostic copy
function restoreDiagnosticCopy(modelName,wasLoaded,modelFile)
    if bdIsLoaded(modelName)
        close_system(modelName,0);
    end
    if wasLoaded
        load_system(modelFile);
    end
end
