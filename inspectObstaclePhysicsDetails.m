%% READ-ONLY INSPECTION: OBSTACLE PHYSICS

projectRoot = ...
    'C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain';

modelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2';

modelFile = fullfile(projectRoot, [modelName '.slx']);

wasLoaded = bdIsLoaded(modelName);

if ~wasLoaded
    load_system(modelFile);
end

fprintf('\n========================================\n');
fprintf(' OBSTACLE PHYSICS PARAMETERS\n');
fprintf('========================================\n');

allBlocks = find_system(modelName, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'off', ...
    'Type', 'Block');

targets = { ...
    'Mechanism Configuration', ...
    'Spatial Contact Force', ...
    'Spatial Contact Force Obstacle2', ...
    'Spatial Contact Force Obstacle3', ...
    'Obstacle2SpawnJoint', ...
    'Obstacle3SpawnJoint', ...
    'Obstacle2SpawnModeStep', ...
    'Obstacle3SpawnModeStep', ...
    'Grid Surface'};

for k = 1:numel(allBlocks)

    blockPath = allBlocks{k};
    blockName = get_param(blockPath, 'Name');

    matched = false;

    for j = 1:numel(targets)
        if strcmpi(blockName, targets{j})
            matched = true;
            break;
        end
    end

    if ~matched
        continue;
    end

    fprintf('\n----------------------------------------\n');
    fprintf('BLOCK: %s\n', blockPath);

    try
        fprintf('TYPE: %s\n', get_param(blockPath, 'BlockType'));
    catch
    end

    try
        params = get_param(blockPath, 'DialogParameters');
        fields = fieldnames(params);

        for j = 1:numel(fields)
            parameter = fields{j};

            try
                value = get_param(blockPath, parameter);
                valueText = strtrim(evalc('disp(value)'));

                fprintf('%s = %s\n', parameter, valueText);
            catch
            end
        end
    catch ME
        fprintf('Parameter inspection issue: %s\n', ME.message);
    end
end

fprintf('\n========================================\n');
fprintf(' PHYSICS INSPECTION COMPLETE\n');
fprintf(' NO MODEL CHANGES MADE\n');
fprintf('========================================\n');

if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
