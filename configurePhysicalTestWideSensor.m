function configurePhysicalTestWideSensor()
% configurePhysicalTestWideSensor
% Permanently set the validated 18 m sensor range on the isolated physical
% spawn test model and patch its builder so future test-model copies retain it.
% Does NOT modify the original checkpoint or scenario-integration model and
% does NOT run a simulation.

projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
if ~strcmpi(pwd, projectRoot), cd(projectRoot); end
addpath(projectRoot);

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest';
modelFile = fullfile(projectRoot, [modelName '.slx']);
builderFile = fullfile(projectRoot, 'createPhysicalScenarioSpawnTestModel.m');
assert(isfile(modelFile), 'Physical test model not found: %s', modelFile);
assert(isfile(builderFile), 'Physical test-model builder not found: %s', builderFile);

% Prepare the builder patch in memory first; fail before changing the model if
% the expected injection point is not present or ambiguous.
builderText = fileread(builderFile);
patchMarker = 'PHYSICAL_TEST_SENSOR_RANGE_18M_PATCH';
builderAlreadyPatched = contains(builderText, patchMarker);
if ~builderAlreadyPatched
    insertionPoint = '    createdModel = true;';
    occurrences = numel(strfind(builderText, insertionPoint)); %#ok<STREMP>
    assert(occurrences == 1, ...
        ['Cannot safely patch createPhysicalScenarioSpawnTestModel.m: ', ...
         'expected one "createdModel = true;" insertion point, found %d.'], occurrences);

    injected = sprintf([ ...
        '\n\n    %% %s: retain the validated 18 m sensor range in future copies.\n', ...
        '    sfrootWideSensor = sfroot;\n', ...
        '    emChartsWideSensor = find(sfrootWideSensor, ''-isa'', ''Stateflow.EMChart'');\n', ...
        '    targetChartPathWideSensor = [testModel ''/ObstacleSensorLogic''];\n', ...
        '    targetChartWideSensor = [];\n', ...
        '    for wideSensorIndex = 1:numel(emChartsWideSensor)\n', ...
        '        if strcmp(emChartsWideSensor(wideSensorIndex).Path, targetChartPathWideSensor)\n', ...
        '            targetChartWideSensor = emChartsWideSensor(wideSensorIndex);\n', ...
        '            break;\n', ...
        '        end\n', ...
        '    end\n', ...
        '    assert(~isempty(targetChartWideSensor), ...\n', ...
        '        ''ObstacleSensorLogic chart not found in physical test copy.'');\n', ...
        '    chartSourceWideSensor = char(targetChartWideSensor.Script);\n', ...
        '    sensorRangePatternWideSensor = ''(?m)^[ \\t]*sensorRange\\s*=\\s*[0-9.]+[ \\t]*;'';\n', ...
        '    assert(~isempty(regexp(chartSourceWideSensor, sensorRangePatternWideSensor, ''once'')), ...\n', ...
        '        ''Could not find the sensorRange assignment in ObstacleSensorLogic.'');\n', ...
        '    chartSourceWideSensor = regexprep(chartSourceWideSensor, ...\n', ...
        '        sensorRangePatternWideSensor, ''sensorRange = 18.0;'', ''once'');\n', ...
        '    targetChartWideSensor.Script = chartSourceWideSensor;\n'], patchMarker);
    builderTextPatched = strrep(builderText, insertionPoint, ...
        [insertionPoint injected]);
else
    builderTextPatched = builderText;
end

% Locate the MATLAB Function block in the existing physical test model.
if ~bdIsLoaded(modelName)
    load_system(modelFile);
end
chartPath = [modelName '/ObstacleSensorLogic'];
charts = find(sfroot, '-isa', 'Stateflow.EMChart');
chart = [];
for k = 1:numel(charts)
    if strcmp(charts(k).Path, chartPath)
        chart = charts(k);
        break;
    end
end
assert(~isempty(chart), 'Could not find Stateflow chart: %s', chartPath);

originalScript = char(chart.Script);
pattern = '(?m)^[ \t]*sensorRange\s*=\s*[0-9.]+[ \t]*;';
matches = regexp(originalScript, pattern, 'match');
assert(numel(matches) == 1, ...
    'Expected exactly one sensorRange assignment in %s; found %d.', chartPath, numel(matches));
updatedScript = regexprep(originalScript, pattern, 'sensorRange = 18.0;', 'once');
assert(contains(updatedScript, 'sensorRange = 18.0;'), ...
    'Failed to prepare the 18 m sensor update.');

% Timestamped backups use copyfile: this does not rename the loaded model.
stamp = datestr(now, 'yyyymmdd_HHMMSS');
modelBackup = fullfile(projectRoot, ...
    [modelName '_preWideSensor_' stamp '.slx']);
builderBackup = '';
assert(copyfile(modelFile, modelBackup), 'Could not create model backup: %s', modelBackup);
fprintf('Model backup created: %s\n', modelBackup);

if ~builderAlreadyPatched
    builderBackup = fullfile(projectRoot, ...
        ['createPhysicalScenarioSpawnTestModel_preWideSensor_' stamp '.m']);
    assert(copyfile(builderFile, builderBackup), ...
        'Could not create builder backup: %s', builderBackup);
    fprintf('Builder backup created: %s\n', builderBackup);
end

% Update and validate the current physical test model before writing changes.
try
    chart.Script = updatedScript;
    set_param(modelName, 'SimulationCommand', 'update');
    save_system(modelName, modelFile);
catch ME
    try
        chart.Script = originalScript;
        set_param(modelName, 'SimulationCommand', 'update');
    catch
    end
    rethrow(ME);
end

% Persist the same setting in the builder for any future model recreation.
if ~builderAlreadyPatched
    fid = fopen(builderFile, 'w');
    assert(fid >= 0, 'Could not open builder for writing: %s', builderFile);
    fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fwrite(fid, builderTextPatched, 'char');
    clear fileCleanup;
end

% Final checks in memory and on disk.
verifyScript = char(chart.Script);
assert(contains(verifyScript, 'sensorRange = 18.0;'), ...
    'The physical test model did not retain sensorRange = 18.0.');
verifyBuilder = fileread(builderFile);
assert(contains(verifyBuilder, patchMarker), ...
    'The physical test-model builder does not contain the 18 m sensor patch.');

fprintf('\n=== PHYSICAL TEST WIDE-SENSOR SETTING SAVED ===\n');
fprintf('Current model: %s\n', modelFile);
fprintf('Sensor range: 18.0 m\n');
fprintf('Builder patched: %s\n', builderFile);
if builderAlreadyPatched
    fprintf('Builder already contained the persistent patch; it was not rewritten.\n');
end
fprintf('Model update succeeded. No simulation was started.\n');
fprintf('Original checkpoint and ScenarioIntegrationTest model were not modified.\n');
end
