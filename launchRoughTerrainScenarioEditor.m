function launchRoughTerrainScenarioEditor
% launchRoughTerrainScenarioEditor
% Interactive 2-D scenario editor for the Autonomous Navigation Rough Terrain
% project. Uses defaultRoverScenario and, when available, the existing
% onlineReplanFromPoseGeometryAware function for before/after-spawn routes.
%
% The editor keeps edits local until the user applies or saves them.
% "Save + Run Physical Workflow" writes scenarioConfig.mat in the project
% root and launches the isolated physical-test runner after compatibility checks.
% The original checkpoint model is not used by that runner.
%
% Current planner scope: axis-aligned box obstacles only.

%% Resolve project files and initialize terrain dependencies
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot = pwd; end
addpath(projectRoot);
savedScenarioFile = fullfile(projectRoot, 'scenarioConfig.mat');
assert(exist('defaultRoverScenario', 'file') == 2, ...
    'defaultRoverScenario.m was not found in the project folder.');

hasTerrainExpr = '(exist(''X_mb'',''var'')==1 && exist(''Y_mb'',''var'')==1 && exist(''Z_mb'',''var'')==1) || (exist(''X'',''var'')==1 && exist(''Y'',''var'')==1 && exist(''Z_multibody'',''var'')==1)';
hasTerrain = evalin('base', hasTerrainExpr);
if ~hasTerrain
    setupFile = fullfile(projectRoot, 'setupOffRoadProject.m');
    assert(isfile(setupFile), 'Terrain setup script not found: %s', setupFile);
    setupLiteral = strrep(setupFile, '''', '''''');
    evalin('base', sprintf('run(''%s'')', setupLiteral));
end

%% Load persisted scenario first; use base-workspace scenario only as fallback
scenario = defaultRoverScenario();
loadedSavedScenario = false;
if isfile(savedScenarioFile)
    try
        savedDataEditor = load(savedScenarioFile, 'scenarioConfig');
        if isfield(savedDataEditor, 'scenarioConfig') && ...
                isstruct(savedDataEditor.scenarioConfig) && ...
                isfield(savedDataEditor.scenarioConfig, 'startPose') && ...
                isfield(savedDataEditor.scenarioConfig, 'goalPose') && ...
                isfield(savedDataEditor.scenarioConfig, 'obstacles')
            scenario = savedDataEditor.scenarioConfig;
            loadedSavedScenario = true;
        end
    catch
        % Fall back to a valid workspace scenario or the defaults.
    end
end
if ~loadedSavedScenario && evalin('base', 'exist(''scenarioConfig'',''var'') == 1')
    try
        existingScenario = evalin('base', 'scenarioConfig');
        if isstruct(existingScenario) && isfield(existingScenario, 'startPose') && ...
                isfield(existingScenario, 'goalPose') && isfield(existingScenario, 'obstacles')
            scenario = existingScenario;
        end
    catch
        % Fall back to the default configuration.
    end
end

terrain = readTerrainFromBase();
selectedObstacleIndex = min(1, numel(scenario.obstacles));
pickMode = 'none';
routes = struct('pre', [], 'post', [], 'preLength', NaN, ...
    'postLength', NaN, 'preCount', 0, 'postCount', 0);

%% Window and layout
fig = uifigure('Name', 'Rough-Terrain Scenario Editor', ...
    'Position', [70 55 1480 900], 'Color', [0.96 0.97 0.98]);
fig.WindowButtonDownFcn = @onFigureMouseDown;

panel = uipanel(fig, 'Title', 'Scenario controls', ...
    'Position', [10 10 440 880], 'FontWeight', 'bold');
ax = uiaxes(fig, 'Position', [465 65 1000 800]);
ax.Box = 'on';
ax.Toolbar.Visible = 'on';
ax.XLabel.String = 'X (m)';
ax.YLabel.String = 'Y (m)';
ax.Title.String = 'Elevation-colored terrain';
ax.NextPlot = 'add';

% Scenario name
makeLabel(panel, 15, 838, 150, 22, 'Scenario name');
scenarioNameField = uieditfield(panel, 'text', ...
    'Position', [15 812 410 25], 'Value', scenario.name);

% Start and goal fields
makeLabel(panel, 15, 780, 140, 22, 'Start position (m)');
makeLabel(panel, 15, 750, 42, 22, 'X');
startXField = uieditfield(panel, 'numeric', 'Position', [42 749 115 25], ...
    'Value', scenario.startPose(1));
makeLabel(panel, 175, 750, 42, 22, 'Y');
startYField = uieditfield(panel, 'numeric', 'Position', [202 749 115 25], ...
    'Value', scenario.startPose(2));

makeLabel(panel, 15, 716, 140, 22, 'Goal position (m)');
makeLabel(panel, 15, 686, 42, 22, 'X');
goalXField = uieditfield(panel, 'numeric', 'Position', [42 685 115 25], ...
    'Value', scenario.goalPose(1));
makeLabel(panel, 175, 686, 42, 22, 'Y');
goalYField = uieditfield(panel, 'numeric', 'Position', [202 685 115 25], ...
    'Value', scenario.goalPose(2));

btnPickStart = uibutton(panel, 'push', 'Text', 'Pick start on map', ...
    'Position', [15 648 195 28], 'ButtonPushedFcn', @onPickStart);
btnPickGoal = uibutton(panel, 'push', 'Text', 'Pick goal on map', ...
    'Position', [225 648 200 28], 'ButtonPushedFcn', @onPickGoal);

% Obstacle selection and editor
makeLabel(panel, 15, 612, 250, 22, 'Obstacles (current planner supports boxes)');
obsDropdown = uidropdown(panel, 'Position', [15 583 265 25], ...
    'Items', obstacleNames(scenario.obstacles), ...
    'ValueChangedFcn', @onObstacleSelectionChanged);
btnPickObstacle = uibutton(panel, 'push', 'Text', 'Pick selected obstacle center on map', ...
    'Position', [15 548 410 28], 'ButtonPushedFcn', @onPickObstacle, ...
    'FontWeight', 'bold');

makeLabel(panel, 15, 520, 70, 22, 'Shape');
shapeDropdown = uidropdown(panel, 'Position', [65 519 120 25], ...
    'Items', {'box'}, 'Value', 'box', 'Enable', 'off');
makeLabel(panel, 205, 520, 60, 22, 'Center X');
obsXField = uieditfield(panel, 'numeric', 'Position', [267 519 75 25], 'Value', 9);
makeLabel(panel, 345, 520, 30, 22, 'Y');
obsYField = uieditfield(panel, 'numeric', 'Position', [370 519 55 25], 'Value', 6.5);

makeLabel(panel, 15, 486, 90, 22, 'Length (m)');
obsLengthField = uieditfield(panel, 'numeric', 'Position', [92 485 75 25], 'Value', 1);
makeLabel(panel, 175, 486, 80, 22, 'Width (m)');
obsWidthField = uieditfield(panel, 'numeric', 'Position', [245 485 75 25], 'Value', 1);
makeLabel(panel, 328, 486, 42, 22, 'Height');
obsHeightField = uieditfield(panel, 'numeric', 'Position', [370 485 55 25], 'Value', 0.5);

dynamicCheckbox = uicheckbox(panel, 'Text', 'Dynamic obstacle', ...
    'Position', [15 452 170 24], 'Value', false, ...
    'ValueChangedFcn', @onDynamicChanged);
makeLabel(panel, 205, 452, 90, 22, 'Spawn time (s)');
spawnTimeField = uieditfield(panel, 'numeric', 'Position', [302 451 95 25], ...
    'Value', 8, 'Enable', 'off');

btnApplyObstacle = uibutton(panel, 'push', 'Text', 'Apply obstacle edits', ...
    'Position', [15 415 195 28], 'ButtonPushedFcn', @onApplyObstacle);
btnAddObstacle = uibutton(panel, 'push', 'Text', 'Add box', ...
    'Position', [225 415 95 28], 'ButtonPushedFcn', @onAddObstacle);
btnRemoveObstacle = uibutton(panel, 'push', 'Text', 'Remove', ...
    'Position', [330 415 95 28], 'ButtonPushedFcn', @onRemoveObstacle);

% Route and scenario actions
btnPreview = uibutton(panel, 'push', 'Text', 'Preview routes (A*)', ...
    'Position', [15 369 410 32], 'FontWeight', 'bold', ...
    'ButtonPushedFcn', @onPreviewRoutes);
btnApplyWorkspace = uibutton(panel, 'push', 'Text', 'Apply to workspace', ...
    'Position', [15 327 195 28], 'ButtonPushedFcn', @onApplyToWorkspace);
btnSave = uibutton(panel, 'push', 'Text', 'Save scenario MAT', ...
    'Position', [225 327 200 28], 'ButtonPushedFcn', @onSaveScenario);
btnRunPhysical = uibutton(panel, 'push', ...
    'Text', 'Save + Run Physical Workflow', ...
    'Position', [15 284 410 31], ...
    'FontWeight', 'bold', ...
    'Tooltip', ['Saves scenarioConfig.mat, runs the calibrated physical test model, and opens the viewer. ' ...
                'Obstacles 1-3 are the calibrated physical core; obstacles 4-10 are virtual sensor/planner/viewer obstacles only.'], ...
    'ButtonPushedFcn', @onRunPhysicalWorkflow);

makeLabel(panel, 15, 248, 400, 22, 'Map-click mode');
pickModeLabel = uilabel(panel, 'Position', [15 222 410 24], ...
    'Text', 'None — select a map-pick button to place a point.');
makeLabel(panel, 15, 177, 410, 39, ...
    'Preview shows routes before spawn and after all configured obstacles appear. It uses the existing geometry-aware planner.');
makeLabel(panel, 15, 135, 410, 36, ...
    'Apply updates the MATLAB workspace. Save + Run saves scenarioConfig.mat and launches the isolated physical test workflow.');
statusLabel = uilabel(panel, 'Position', [15 20 410 105], ...
    'Text', 'Ready. Edit the scenario, preview routes, or save and run the physical workflow.', ...
    'WordWrap', 'on', 'FontColor', [0.15 0.20 0.25]);

% Initialize the obstacle controls and scene
if isempty(scenario.obstacles)
    selectedObstacleIndex = 0;
else
    selectedObstacleIndex = 1;
end
refreshObstacleControls();
redrawScene();

%% Nested callbacks and helpers
    function onObstacleSelectionChanged(~, ~)
        items = obsDropdown.Items;
        if isempty(items) || isempty(scenario.obstacles)
            selectedObstacleIndex = 0;
            return;
        end
        idx = find(strcmp(items, obsDropdown.Value), 1, 'first');
        if isempty(idx)
            idx = 1;
        end
        selectedObstacleIndex = idx;
        populateObstacleFields();
    end

    function refreshObstacleControls()
        names = obstacleNames(scenario.obstacles);
        if isempty(names)
            names = {'No obstacles'};
            obsDropdown.Items = names;
            obsDropdown.Value = names{1};
            selectedObstacleIndex = 0;
            setObstacleFieldsEnabled(false);
        else
            obsDropdown.Items = names;
            selectedObstacleIndex = max(1, min(selectedObstacleIndex, numel(names)));
            obsDropdown.Value = names{selectedObstacleIndex};
            setObstacleFieldsEnabled(true);
            populateObstacleFields();
        end
        btnRemoveObstacle.Enable = onOff(numel(scenario.obstacles) > 1);
    end

    function setObstacleFieldsEnabled(enableFields)
        state = onOff(enableFields);
        shapeDropdown.Enable = 'off';
        obsXField.Enable = state;
        obsYField.Enable = state;
        obsLengthField.Enable = state;
        obsWidthField.Enable = state;
        obsHeightField.Enable = state;
        dynamicCheckbox.Enable = state;
        btnPickObstacle.Enable = state;
        btnApplyObstacle.Enable = state;
        if ~enableFields
            spawnTimeField.Enable = 'off';
        end
    end

    function populateObstacleFields()
        if selectedObstacleIndex < 1 || selectedObstacleIndex > numel(scenario.obstacles)
            return;
        end
        o = scenario.obstacles(selectedObstacleIndex);
        obsXField.Value = o.centerXY(1);
        obsYField.Value = o.centerXY(2);
        obsLengthField.Value = o.dimensions(1);
        obsWidthField.Value = o.dimensions(2);
        obsHeightField.Value = o.dimensions(3);
        dynamicCheckbox.Value = logical(o.isDynamic);
        if isfinite(o.spawnTime)
            spawnTimeField.Value = o.spawnTime;
        else
            spawnTimeField.Value = 8;
        end
        spawnTimeField.Enable = onOff(logical(o.isDynamic));
        shapeDropdown.Value = 'box';
    end

    function onDynamicChanged(~, ~)
        spawnTimeField.Enable = onOff(dynamicCheckbox.Value);
    end

    function onApplyObstacle(~, ~)
        try
            syncSelectedObstacle();
            clearRouteCache();
            redrawScene();
            setStatus(sprintf('Applied edits to %s. Click Preview routes (A*) to recalculate.', ...
                scenario.obstacles(selectedObstacleIndex).name));
        catch ME
            uialert(fig, ME.message, 'Invalid obstacle settings');
        end
    end

    function syncSelectedObstacle()
        if selectedObstacleIndex < 1 || selectedObstacleIndex > numel(scenario.obstacles)
            error('Select or add an obstacle before editing it.');
        end
        x = obsXField.Value;
        y = obsYField.Value;
        L = obsLengthField.Value;
        W = obsWidthField.Value;
        H = obsHeightField.Value;
        if any(~isfinite([x y L W H])) || any([L W H] <= 0)
            error('Obstacle centre coordinates must be finite and dimensions must be positive.');
        end
        o = scenario.obstacles(selectedObstacleIndex);
        o.centerXY = [x y];
        o.dimensions = [L W H];
        o.shape = 'box';
        o.isDynamic = logical(dynamicCheckbox.Value);
        if o.isDynamic
            if ~isfinite(spawnTimeField.Value) || spawnTimeField.Value < 0
                error('Dynamic-obstacle spawn time must be finite and nonnegative.');
            end
            o.spawnTime = spawnTimeField.Value;
        else
            o.spawnTime = NaN;
        end
        scenario.obstacles(selectedObstacleIndex) = o;
        scenario.name = char(scenarioNameField.Value);
        refreshObstacleControls();
    end

    function onAddObstacle(~, ~)
        try
            syncBasicFields();
            if isempty(scenario.obstacles)
                template = defaultRoverScenario();
                o = template.obstacles(1);
            else
                o = scenario.obstacles(end);
            end
            nextId = 1;
            if ~isempty(scenario.obstacles)
                nextId = max([scenario.obstacles.id]) + 1;
            end
            o.id = nextId;
            o.name = sprintf('Obstacle %d', nextId);
            o.shape = 'box';
            o.centerXY = [(terrain.x(1)+terrain.x(end))/2, (terrain.y(1)+terrain.y(end))/2];
            o.dimensions = [1 1 0.5];
            o.isDynamic = false;
            o.spawnTime = NaN;
            if isempty(scenario.obstacles)
                scenario.obstacles = o;
            else
                scenario.obstacles(end+1,1) = o;
            end
            selectedObstacleIndex = numel(scenario.obstacles);
            refreshObstacleControls();
            clearRouteCache();
            redrawScene();
            setStatus(sprintf('Added %s. Move it on the map or edit its centre and dimensions.', o.name));
        catch ME
            uialert(fig, ME.message, 'Could not add obstacle');
        end
    end

    function onRemoveObstacle(~, ~)
        if numel(scenario.obstacles) <= 1 || selectedObstacleIndex < 1
            return;
        end
        removedName = scenario.obstacles(selectedObstacleIndex).name;
        scenario.obstacles(selectedObstacleIndex) = [];
        selectedObstacleIndex = min(selectedObstacleIndex, numel(scenario.obstacles));
        refreshObstacleControls();
        clearRouteCache();
        redrawScene();
        setStatus(sprintf('Removed %s. Click Preview routes (A*) to recalculate.', removedName));
    end

    function onPickStart(~, ~)
        pickMode = 'start';
        pickModeLabel.Text = 'Click a location on the terrain map to set the start.';
        setStatus('Map-click mode: click the terrain map to set the start position.');
    end

    function onPickGoal(~, ~)
        pickMode = 'goal';
        pickModeLabel.Text = 'Click a location on the terrain map to set the goal.';
        setStatus('Map-click mode: click the terrain map to set the goal position.');
    end

    function onPickObstacle(~, ~)
        if selectedObstacleIndex < 1
            return;
        end
        pickMode = 'obstacle';
        pickModeLabel.Text = sprintf('Click the map to place %s.', ...
            scenario.obstacles(selectedObstacleIndex).name);
        setStatus('Map-click mode: click a terrain location to move the selected obstacle centre.');
    end

    function onFigureMouseDown(~, ~)
        if strcmp(pickMode, 'none') || ~isvalid(ax)
            return;
        end
        clickedObject = hittest(fig);
        hitAxes = [];
        try
            hitAxes = ancestor(clickedObject, 'matlab.ui.control.UIAxes');
        catch
        end
        if isempty(hitAxes)
            try
                if isequal(clickedObject, ax)
                    hitAxes = ax;
                end
            catch
            end
        end
        if isempty(hitAxes) || ~isequal(hitAxes, ax)
            return;
        end
        cp = ax.CurrentPoint;
        x = cp(1,1);
        y = cp(1,2);
        if ~isfinite(x) || ~isfinite(y) || x < min(terrain.x) || ...
                x > max(terrain.x) || y < min(terrain.y) || y > max(terrain.y)
            return;
        end
        switch pickMode
            case 'start'
                startXField.Value = x;
                startYField.Value = y;
                syncBasicFields();
                setStatus(sprintf('Start position set to (%.2f, %.2f) m.', x, y));
            case 'goal'
                goalXField.Value = x;
                goalYField.Value = y;
                syncBasicFields();
                setStatus(sprintf('Goal position set to (%.2f, %.2f) m.', x, y));
            case 'obstacle'
                obsXField.Value = x;
                obsYField.Value = y;
                syncSelectedObstacle();
                setStatus(sprintf('Obstacle centre set to (%.2f, %.2f) m.', x, y));
        end
        pickMode = 'none';
        pickModeLabel.Text = 'None — select a map-pick button to place a point.';
        clearRouteCache();
        redrawScene();
    end

    function syncBasicFields()
        v = [startXField.Value, startYField.Value, goalXField.Value, goalYField.Value];
        if any(~isfinite(v))
            error('Start and goal coordinates must be finite numbers.');
        end
        scenario.startPose(1:2) = v(1:2);
        scenario.goalPose(1:2) = v(3:4);
        scenario.name = char(scenarioNameField.Value);
    end

    function syncAllFields()
        syncBasicFields();
        if selectedObstacleIndex >= 1 && selectedObstacleIndex <= numel(scenario.obstacles)
            syncSelectedObstacle();
        end
        validateScenarioForPreview(scenario);
    end

    function onPreviewRoutes(~, ~)
        try
            syncAllFields();
            staticObs = scenario.obstacles(~[scenario.obstacles.isDynamic]);
            allObs = scenario.obstacles;
            setStatus('Calculating routes with the existing geometry-aware A* planner...');
            drawnow;

            preRoute = [];
            postRoute = [];
            preLength = NaN;
            postLength = NaN;
            preCount = 0;
            postCount = 0;
            if ~isempty(staticObs)
                [preRoute, preCount, preLength] = calculateRoute(staticObs);
            end
            if ~isempty(allObs)
                [postRoute, postCount, postLength] = calculateRoute(allObs);
            end
            routes.pre = preRoute;
            routes.post = postRoute;
            routes.preLength = preLength;
            routes.postLength = postLength;
            routes.preCount = preCount;
            routes.postCount = postCount;
            redrawScene();

            if isempty(staticObs)
                beforeText = 'Before dynamic spawn: no static obstacles; route omitted';
            else
                beforeText = sprintf('Before dynamic spawn: %d waypoints, %.2f m', preCount, preLength);
            end
            if isempty(allObs)
                afterText = 'After spawn: no obstacles';
            else
                afterText = sprintf('After all configured obstacles spawn: %d waypoints, %.2f m', ...
                    postCount, postLength);
            end
            setStatus(sprintf('%s\n%s\nRoutes use the geometry-aware planner. The Simulink physical geometry is not changed.', ...
                beforeText, afterText));
        catch ME
            setStatus(sprintf('Route preview failed: %s', ME.message));
            uialert(fig, sprintf(['The editor kept your scenario changes, but route preview failed.\n\n' ...
                '%s\n\nCheck that plannerFootprint and vehicleClearanceRadius are initialized, ' ...
                'and that onlineReplanFromPoseGeometryAware.m is on the MATLAB path.'], ME.message), ...
                'A* preview failed');
        end
    end

    function [route, count, routeLength] = calculateRoute(obs)
        if exist('onlineReplanFromPoseGeometryAware', 'file') ~= 2
            error('onlineReplanFromPoseGeometryAware.m is not on the MATLAB path.');
        end
        if evalin('base', 'exist(''plannerFootprint'',''var'') ~= 1')
            error('plannerFootprint is not in the base workspace. Run startRoughTerrainProject first.');
        end
        if evalin('base', 'exist(''vehicleClearanceRadius'',''var'') ~= 1')
            error('vehicleClearanceRadius is not in the base workspace. Run startRoughTerrainProject first.');
        end
        centers = vertcat(obs.centerXY);
        cfgForPlanner = scenario;
        cfgForPlanner.obstacles = obs;
        hadOldConfig = evalin('base', 'exist(''scenarioConfig'',''var'') == 1');
        if hadOldConfig
            oldConfig = evalin('base', 'scenarioConfig');
        else
            oldConfig = [];
        end
        assignin('base', 'scenarioConfig', cfgForPlanner);
        restoreConfig = onCleanup(@() restoreBaseScenarioConfig(hadOldConfig, oldConfig));
        [paddedRoute, count, ~, routeLength] = onlineReplanFromPoseGeometryAware( ...
            scenario.startPose(1:2), scenario.goalPose(1:2), ...
            centers(:,1), centers(:,2), scenario.planning.obstacleClearance, ...
            scenario.planning.maxWaypoints);
        clear restoreConfig;
        count = max(0, min(round(double(count)), size(paddedRoute,1)));
        route = paddedRoute(1:count, :);
        route = route(all(isfinite(route),2), :);
        count = size(route,1);
        if count < 2
            error('The planner returned fewer than two finite waypoints.');
        end
    end

    function onApplyToWorkspace(~, ~)
        try
            syncAllFields();
            assignin('base', 'scenarioConfig', scenario);
            setStatus(['Applied the edited scenario to base-workspace variable scenarioConfig. ' ...
                'No Simulink model or physical obstacle block was modified.']);
        catch ME
            uialert(fig, ME.message, 'Could not apply scenario');
        end
    end

    function onSaveScenario(~, ~)
        try
            syncAllFields();
            defaultFile = savedScenarioFile;
            [file, folder] = uiputfile('*.mat', 'Save scenario configuration', defaultFile);
            if isequal(file, 0)
                return;
            end
            scenarioConfig = scenario; %#ok<NASGU>
            save(fullfile(folder, file), 'scenarioConfig');
            setStatus(sprintf('Saved scenario configuration to:\n%s', fullfile(folder, file)));
        catch ME
            uialert(fig, ME.message, 'Could not save scenario');
        end
    end

    function onRunPhysicalWorkflow(~, ~)
        try
            syncAllFields();
            validateScenarioForPhysicalWorkflow(scenario, terrain);
            runnerFile = fullfile(projectRoot, 'runPhysicalScenarioWorkflow.m');
            assert(isfile(runnerFile), ...
                'Physical workflow runner was not found: %s', runnerFile);

            prompt = sprintf([ ...
                'This will save the edited scenario to:\n%s\n\n' ...
                'Then it will update/run the separate physical test model for %.1f s and open the viewer.\n\n' ...
                'The original checkpoint is not used. Continue?'], ...
                savedScenarioFile, double(scenario.simulation.stopTime));
            selection = uiconfirm(fig, prompt, 'Run Physical Workflow', ...
                'Options', {'Run physical workflow', 'Cancel'}, ...
                'DefaultOption', 2, 'CancelOption', 2);
            if ~strcmp(selection, 'Run physical workflow')
                setStatus('Physical workflow cancelled. Scenario was not saved by this action.');
                return;
            end

            setStatus(sprintf('Saving scenario to:\n%s\n\nLaunching physical workflow. MATLAB may be busy while the simulation runs...', ...
                savedScenarioFile));
            drawnow;

            % Write the exact MAT variable name expected by the runner.
            scenarioConfig = scenario; %#ok<NASGU>
            save(savedScenarioFile, 'scenarioConfig');
            assignin('base', 'scenarioConfig', scenario);

            % The runner is a script designed to run from the base workspace.
            runnerLiteral = strrep(runnerFile, '''', '''''');
            evalin('base', sprintf('run(''%s'')', runnerLiteral));

            setStatus(sprintf(['Physical workflow completed.\nScenario saved to:\n%s\n\n' ...
                'Review the command-window report and the newly opened elevation viewer.'], ...
                savedScenarioFile));
        catch ME
            setStatus(sprintf('Physical workflow stopped: %s', ME.message));
            try
                uialert(fig, sprintf('The physical workflow did not complete.\n\n%s', ...
                    getReport(ME, 'basic', 'hyperlinks', 'off')), ...
                    'Physical workflow stopped');
            catch
                uialert(fig, ME.message, 'Physical workflow stopped');
            end
        end
    end

    function clearRouteCache()
        routes.pre = [];
        routes.post = [];
        routes.preLength = NaN;
        routes.postLength = NaN;
        routes.preCount = 0;
        routes.postCount = 0;
    end

    function redrawScene()
        cla(ax);
        ax.NextPlot = 'add';
        hTerrain = imagesc(ax, terrain.x, terrain.y, terrain.z);
        ax.YDir = 'normal';
        try
            hTerrain.HitTest = 'off';
            hTerrain.PickableParts = 'none';
        catch
        end
        colormap(ax, turbo(256));
        ax.XLim = [min(terrain.x), max(terrain.x)];
        ax.YLim = [min(terrain.y), max(terrain.y)];
        axis(ax, 'equal');
        ax.Box = 'on';
        ax.XGrid = 'on';
        ax.YGrid = 'on';
        ax.GridAlpha = 0.18;
        ax.XLabel.String = 'X (m)';
        ax.YLabel.String = 'Y (m)';
        ax.Title.String = sprintf('%s — elevation map', scenario.name);
        try
            colorbar(ax);
        catch
        end
        hold(ax, 'on');

        % Obstacle boxes
        for jj = 1:numel(scenario.obstacles)
            o = scenario.obstacles(jj);
            if logical(o.isDynamic)
                edge = [0.72 0.12 0.72];
                style = '--';
            else
                edge = [0.80 0.20 0.16];
                style = '-';
            end
            hRect = rectangle(ax, 'Position', ...
                [o.centerXY(1)-o.dimensions(1)/2, ...
                 o.centerXY(2)-o.dimensions(2)/2, ...
                 o.dimensions(1), o.dimensions(2)], ...
                'EdgeColor', edge, 'LineWidth', 1.8, ...
                'LineStyle', style, 'FaceColor', 'none');
            try
                hRect.HitTest = 'off';
                hRect.PickableParts = 'none';
            catch
            end
            if logical(o.isDynamic)
                labelText = sprintf('%s (t=%.2g s)', o.name, o.spawnTime);
            else
                labelText = o.name;
            end
            hText = text(ax, o.centerXY(1), o.centerXY(2), labelText, ...
                'Color', edge, 'FontSize', 9, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
            try
                hText.HitTest = 'off';
                hText.PickableParts = 'none';
            catch
            end
        end

        % Routes are drawn above terrain and obstacle outlines.
        if ~isempty(routes.pre)
            h = plot(ax, routes.pre(:,1), routes.pre(:,2), '-', ...
                'Color', [0.05 0.30 0.95], 'LineWidth', 2.0);
            makeUnpickable(h);
        end
        if ~isempty(routes.post)
            h = plot(ax, routes.post(:,1), routes.post(:,2), '--', ...
                'Color', [0.05 0.72 0.82], 'LineWidth', 2.2);
            makeUnpickable(h);
        end

        hStart = plot(ax, scenario.startPose(1), scenario.startPose(2), ...
            'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.10 0.70 0.25], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.1);
        hGoal = plot(ax, scenario.goalPose(1), scenario.goalPose(2), ...
            'p', 'MarkerSize', 12, 'MarkerFaceColor', [1.00 0.78 0.12], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.1);
        makeUnpickable(hStart); makeUnpickable(hGoal);
        hStaticProxy = plot(ax, NaN, NaN, '-', 'Color', [0.80 0.20 0.16], 'LineWidth', 1.8);
        hDynamicProxy = plot(ax, NaN, NaN, '--', 'Color', [0.72 0.12 0.72], 'LineWidth', 1.8);
        hPreProxy = plot(ax, NaN, NaN, '-', 'Color', [0.05 0.30 0.95], 'LineWidth', 2.0);
        hPostProxy = plot(ax, NaN, NaN, '--', 'Color', [0.05 0.72 0.82], 'LineWidth', 2.2);
        makeUnpickable([hStaticProxy hDynamicProxy hPreProxy hPostProxy]);
        legend(ax, [hStart hGoal hStaticProxy hDynamicProxy hPreProxy hPostProxy], ...
            {'Start', 'Goal', 'Static box', 'Dynamic box', 'Before spawn route', 'After spawn route'}, ...
            'Location', 'northoutside', 'Orientation', 'horizontal');
        hold(ax, 'off');
    end

    function makeUnpickable(h)
        try
            set(h, 'HitTest', 'off');
            set(h, 'PickableParts', 'none');
        catch
        end
    end

    function setStatus(message)
        statusLabel.Text = char(message);
        drawnow limitrate;
    end
end

function terrain = readTerrainFromBase()
% Prefer the project's 251x251 multibody terrain vectors when available.
hasMB = evalin('base', ...
    'exist(''X_mb'',''var'')==1 && exist(''Y_mb'',''var'')==1 && exist(''Z_mb'',''var'')==1');
if hasMB
    x = double(evalin('base', 'X_mb'));
    y = double(evalin('base', 'Y_mb'));
    z = double(evalin('base', 'Z_mb'));
    x = x(:)';
    y = y(:);
    % The project's elevation viewer uses the transposed Z_mb orientation.
    if isequal(size(z), [numel(x), numel(y)])
        z = z.';
    elseif ~isequal(size(z), [numel(y), numel(x)])
        error('Z_mb size does not match X_mb and Y_mb coordinate vectors.');
    end
elseif evalin('base', ...
        'exist(''X'',''var'')==1 && exist(''Y'',''var'')==1 && exist(''Z_multibody'',''var'')==1')
    X = double(evalin('base', 'X'));
    Y = double(evalin('base', 'Y'));
    z = double(evalin('base', 'Z_multibody'));
    if ~isequal(size(X), size(Y))
        error('X and Y terrain grids have inconsistent sizes.');
    end
    x = X(1,:);
    y = Y(:,1);
    if ~isequal(size(z), [numel(y), numel(x)])
        if isequal(size(z.'), [numel(y), numel(x)])
            z = z.';
        else
            error('Z_multibody size does not match the X/Y grids.');
        end
    end
else
    error(['Terrain variables were not found. Run setupOffRoadProject.m or ' ...
        'startRoughTerrainProject.m first so X_mb, Y_mb, and Z_mb are loaded.']);
end

if numel(x) < 2 || numel(y) < 2 || any(~isfinite(x)) || any(~isfinite(y))
    error('Terrain coordinate vectors must contain finite grid coordinates.');
end
if ~isequal(size(z), [numel(y), numel(x)])
    error('Terrain elevation matrix dimensions do not match coordinate vectors.');
end
terrain = struct('x', x, 'y', y, 'z', z);
end

function names = obstacleNames(obstacles)
if isempty(obstacles)
    names = {};
    return;
end
names = cell(1, numel(obstacles));
for k = 1:numel(obstacles)
    names{k} = char(obstacles(k).name);
end
% Make dropdown values unique while preserving the displayed obstacle names.
for k = 1:numel(names)
    if sum(strcmp(names, names{k})) > 1
        names{k} = sprintf('%s (ID %d)', names{k}, obstacles(k).id);
    end
end
end

function validateScenarioForPreview(scenario)
if numel(scenario.startPose) < 2 || numel(scenario.goalPose) < 2 || ...
        any(~isfinite([scenario.startPose(1:2), scenario.goalPose(1:2)]))
    error('Start and goal positions must contain finite XY coordinates.');
end
if isempty(scenario.obstacles)
    error('Keep at least one obstacle configured to use the current planner.');
end
for k = 1:numel(scenario.obstacles)
    o = scenario.obstacles(k);
    if ~strcmpi(o.shape, 'box')
        error('The current geometry-aware planner supports box obstacles only.');
    end
    if numel(o.centerXY) ~= 2 || numel(o.dimensions) ~= 3 || ...
            any(~isfinite([o.centerXY o.dimensions])) || any(o.dimensions <= 0)
        error('Obstacle %d has invalid centre coordinates or dimensions.', o.id);
    end
    if logical(o.isDynamic) && (~isfinite(o.spawnTime) || o.spawnTime < 0)
        error('Dynamic obstacle %d must have a nonnegative finite spawn time.', o.id);
    end
end
if ~isfield(scenario, 'planning') || ~isfield(scenario.planning, 'maxWaypoints') || ...
        ~isfield(scenario.planning, 'obstacleClearance')
    error('Scenario is missing planning.maxWaypoints or planning.obstacleClearance.');
end
end

function validateScenarioForPhysicalWorkflow(scenario, terrain)
% The current physical runner contains calibrated world-Z offsets for the
% specific dynamic-obstacle centres and dimensions that were tested. Additional
% configured boxes (IDs >= 4) are virtual sensor/planner/viewer obstacles only.
validateScenarioForPreview(scenario);
if ~isfield(scenario, 'simulation') || ~isfield(scenario.simulation, 'stopTime') || ...
        ~isscalar(scenario.simulation.stopTime) || ...
        ~isfinite(scenario.simulation.stopTime) || scenario.simulation.stopTime <= 0
    error('Scenario must have a positive finite simulation.stopTime.');
end
obs = scenario.obstacles(:);
if numel(obs) < 3
    error(['The physical workflow requires the calibrated core obstacles with IDs 1, 2, and 3. ' ...
        'Add or restore those three obstacles before running the physical workflow.']);
end
if numel(obs) > 10
    error(['The current online sensor/replanning model supports at most 10 configured obstacles. ' ...
        'Reduce the scenario to 10 or fewer boxes.']);
end
[~, ix] = sort([obs.id]);
obs = obs(ix);
ids = [obs.id];
if numel(unique(ids)) ~= numel(ids) || ~all(isfinite(ids)) || ...
        any(ids < 1) || any(ids ~= fix(ids)) || ~all(ismember([1 2 3], ids))
    error('Physical workflow requires unique positive integer obstacle IDs and core IDs 1, 2, and 3.');
end
if ~isequal(logical([obs(1:3).isDynamic]), [false true true]) || ...
        ~isequal([obs(1:3).id], [1 2 3])
    error('Core obstacle roles are fixed: obstacle 1 static, obstacles 2 and 3 dynamic.');
end
if numel(obs) > 3
    fprintf(['Note: obstacles with IDs above 3 are virtual sensor/planner/viewer obstacles; ' ...
        'they do not have separate Simscape collision bodies.\n']);
end
if ~all(arrayfun(@(o) strcmpi(o.shape, 'box'), obs))
    error('Physical workflow currently supports box obstacles only.');
end
if ~all(isfinite([obs(2).spawnTime obs(3).spawnTime])) || ...
        any([obs(2).spawnTime obs(3).spawnTime] <= 0)
    error('Dynamic obstacle spawn times must be positive and finite.');
end
calibratedCenters = [23.7161198288 22.8601997147; ...
                     16.7974322397 36.0556348074];
calibratedDims = [2 23 1; 4 4 0.5];
currentCenters = [obs(2).centerXY; obs(3).centerXY];
currentDims = [obs(2).dimensions; obs(3).dimensions];
centerMatch = abs(currentCenters-calibratedCenters) < 1e-3;
dimensionMatch = abs(currentDims-calibratedDims) < 1e-9;
if ~all(centerMatch(:)) || ~all(dimensionMatch(:))
    error(['Physical spawn calibration is only validated for obstacle 2 center=[23.7161 22.8602], ' ...
        'size=[2 23 1] and obstacle 3 center=[16.7974 36.0556], size=[4 4 0.5]. ' ...
        'Save the scenario for virtual workflow, or restore these geometries before physical execution.']);
end
startXY = double(scenario.startPose(1:2));
goalXY = double(scenario.goalPose(1:2));
if startXY(1) < min(terrain.x) || startXY(1) > max(terrain.x) || ...
   startXY(2) < min(terrain.y) || startXY(2) > max(terrain.y)
    error('Start position lies outside the terrain bounds.');
end
if goalXY(1) < min(terrain.x) || goalXY(1) > max(terrain.x) || ...
   goalXY(2) < min(terrain.y) || goalXY(2) > max(terrain.y)
    error('Goal position lies outside the terrain bounds.');
end
% Match the runner's nominal free-fall lead-in, so it will not generate
% a nonpositive release time for the guided-stop approximation.
embedDepth = 0.05;
for id = 2:3
    qInitial = -(obs(id).dimensions(3)/2 + embedDepth);
    releaseTime = obs(id).spawnTime - sqrt(2*abs(qInitial)/9.81);
    if releaseTime <= 0
        error('Obstacle %d spawn time is too early for the current physical release approximation.', id);
    end
end
end

function restoreBaseScenarioConfig(hadOldConfig, oldConfig)
if hadOldConfig
    assignin('base', 'scenarioConfig', oldConfig);
else
    evalin('base', 'clear scenarioConfig');
end
end

function label = makeLabel(parent, x, y, w, h, textValue)
label = uilabel(parent, 'Position', [x y w h], 'Text', textValue);
end

function value = onOff(tf)
if tf
    value = 'on';
else
    value = 'off';
end
end
