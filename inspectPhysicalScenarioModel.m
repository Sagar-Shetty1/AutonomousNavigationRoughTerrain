
%% READ-ONLY INSPECTION OF V2 PHYSICAL MODEL
% Does not modify, update, save, or simulate the model.

projectRoot = 'C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain';
modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_v2';
modelFile = fullfile(projectRoot, [modelName '.slx']);

assert(isfile(modelFile), 'Model file not found: %s', modelFile);

wasLoaded = bdIsLoaded(modelName);

if ~wasLoaded
    load_system(modelFile);
end

fprintf('\n============================================\n');
fprintf(' READ-ONLY PHYSICAL MODEL INSPECTION\n');
fprintf('============================================\n');

fprintf('\nModel: %s\n', modelName);
fprintf('Stop time: %s\n', get_param(modelName, 'StopTime'));
fprintf('Solver: %s\n', get_param(modelName, 'Solver'));
fprintf('Dirty state: %s\n', get_param(modelName, 'Dirty'));

%% 1. Find relevant blocks

allBlocks = find_system(modelName, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'off', ...
    'Type', 'Block');

if ischar(allBlocks)
    allBlocks = {allBlocks};
end

blockNames = cell(size(allBlocks));

for k = 1:numel(allBlocks)
    blockNames{k} = lower(allBlocks{k});
end

isRelevant = false(size(allBlocks));

for k = 1:numel(blockNames)
    s = blockNames{k};

    isRelevant(k) = ...
        contains(s, 'obstacle') || ...
        contains(s, 'spawn') || ...
        contains(s, 'joint') || ...
        contains(s, 'contact') || ...
        contains(s, '6-dof');
end

relevantBlocks = allBlocks(isRelevant);

%% 2. Print block inventory

fprintf('\n============================================\n');
fprintf(' RELEVANT BLOCK INVENTORY\n');
fprintf('============================================\n');

for k = 1:numel(relevantBlocks)

    b = relevantBlocks{k};

    fprintf('\n%s\n', b);

    try
        fprintf('  BlockType: %s\n', get_param(b, 'BlockType'));
    catch
    end

    try
        maskType = get_param(b, 'MaskType');
        if ~isempty(maskType)
            fprintf('  MaskType: %s\n', maskType);
        end
    catch
    end
end

%% 3. Print selected block parameters

fprintf('\n============================================\n');
fprintf(' BLOCK PARAMETERS\n');
fprintf('============================================\n');

parameterPattern = ...
    'brick|dimension|based|mass|inertia|translation|rotation|jointmode|actuation|positiontarget|limit|time|before|after|unit|sense';

for k = 1:numel(relevantBlocks)

    b = relevantBlocks{k};

    try
        dialogParams = get_param(b, 'DialogParameters');
        parameterNames = fieldnames(dialogParams);
    catch
        continue;
    end

    fprintf('\n%s\n', b);

    for j = 1:numel(parameterNames)

        p = parameterNames{j};

        if isempty(regexpi(p, parameterPattern))
            continue;
        end

        try
            value = get_param(b, p);
            valueText = strtrim(evalc('disp(value)'));

            fprintf('  %s = %s\n', p, valueText);
        catch
            fprintf('  %s = [Could not read]\n', p);
        end
    end
end

%% 4. Print connections

fprintf('\n============================================\n');
fprintf(' BLOCK CONNECTIONS\n');
fprintf('============================================\n');

for k = 1:numel(relevantBlocks)

    b = relevantBlocks{k};

    try
        connectivity = get_param(b, 'PortConnectivity');
    catch
        continue;
    end

    fprintf('\n%s\n', b);

    for p = 1:numel(connectivity)

        fprintf('  Port %d [%s]\n', ...
            p, connectivity(p).Type);

        % Connected source blocks
        handles = connectivity(p).SrcBlock;

        for q = 1:numel(handles)
            h = handles(q);

            try
                fprintf('    Source: %s\n', getfullname(h));
            catch
            end
        end

        % Connected destination blocks
        handles = connectivity(p).DstBlock;

        for q = 1:numel(handles)
            h = handles(q);

            try
                fprintf('    Destination: %s\n', getfullname(h));
            catch
            end
        end
    end
end

%% 5. Finish

fprintf('\n============================================\n');
fprintf(' INSPECTION COMPLETE\n');
fprintf(' No model changes were made.\n');
fprintf('============================================\n');

% Close the model only if this script opened it.
if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
