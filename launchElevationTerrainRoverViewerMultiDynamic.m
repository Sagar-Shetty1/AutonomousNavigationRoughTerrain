function fig = launchElevationTerrainRoverViewerMultiDynamic()
% launchElevationTerrainRoverViewerMultiDynamic
% Custom elevation-colored terrain viewer with a simple rover graphic.
%
% Required in MATLAB base workspace:
%   X_mb, Y_mb, Z_mb
%
% Optional in base workspace:
%   simOut: SimulationOutput containing mb_px and mb_py timeseries. If
%           present, the viewer automatically replays recorded XY motion.
%   roverPoseLog: N-by-4 [time, x, y, yaw_rad], overrides simOut if present.
%   originalPath or originalPathPadded: N-by-2 planned XY path fallback.
%   scenarioConfig: canonical obstacle definitions and geometry dimensions.
%   Supports any number of dynamic obstacles, each shown at its own spawn time.
%
% This viewer is separate from Simulink and does not modify the physics model.
% It estimates heading from XY motion when the logged heading is unavailable.

% Read terrain arrays without changing the user's base workspace.
required = {'X_mb','Y_mb','Z_mb'};
for k = 1:numel(required)
    if evalin('base', sprintf('exist(''%s'',''var'')', required{k})) ~= 1
        error('ElevationViewer:MissingTerrain', ...
            'Base workspace variable %s is missing. Load the terrain setup first.', required{k});
    end
end
X = double(evalin('base','X_mb'));
Y = double(evalin('base','Y_mb'));
ZGridSurface = double(evalin('base','Z_mb'));
X = X(:)';
Y = Y(:);
% setupOffRoadProject stores Z_mb transposed for Simscape Grid Surface.
% MATLAB surf/interp2 instead require rows aligned with Y and columns with X,
% so transpose back here before drawing or interpolating the viewer terrain.
Z = ZGridSurface.';
if ~isequal(size(Z), [numel(Y), numel(X)])
    error('ElevationViewer:TerrainSize', ...
        ['After converting from Simscape Grid Surface orientation, Z_mb must ', ...
         'have size [numel(Y_mb), numel(X_mb)].']);
end
if any(~isfinite(X)) || any(~isfinite(Y)) || any(~isfinite(Z(:)))
    error('ElevationViewer:TerrainFinite', 'Terrain data contains NaN or Inf values.');
end

hasPoseLog = evalin('base', 'exist(''roverPoseLog'',''var'')') == 1;
hasSimOut = evalin('base', 'exist(''simOut'',''var'')') == 1;
isTimeBasedPlayback = hasPoseLog || hasSimOut;

if hasPoseLog
    raw = double(evalin('base','roverPoseLog'));
    if size(raw,2) < 4
        error('ElevationViewer:PoseLogShape', ...
            'roverPoseLog must be N-by-4: [time, x, y, yaw_radians].');
    end
    raw = raw(:,1:4);
    valid = all(isfinite(raw),2) & ...
        raw(:,2) >= min(X) & raw(:,2) <= max(X) & ...
        raw(:,3) >= min(Y) & raw(:,3) <= max(Y);
    raw = raw(valid,:);
    if size(raw,1) < 2
        error('ElevationViewer:PoseLogRange', ...
            'roverPoseLog has fewer than two valid poses inside the terrain bounds.');
    end
    [~, keepOrder] = unique(raw(:,1), 'stable');
    raw = raw(sort(keepOrder),:);
    frameTime = raw(:,1);
    poseXY = raw(:,2:3);
    yaw = unwrap(raw(:,4));
    playbackDescription = 'Recorded simulation pose';

elseif hasSimOut
    % Prefer actual measured chassis XY outputs from the completed simulation.
    so = evalin('base','simOut');
    if ~isa(so,'Simulink.SimulationOutput')
        error('ElevationViewer:SimulationOutputType', ...
            'The base-workspace variable simOut is not a Simulink.SimulationOutput.');
    end
    available = so.who;
    if ~all(ismember({'mb_px','mb_py'}, available))
        error('ElevationViewer:MissingPositionSignals', ...
            ['simOut exists but does not contain both mb_px and mb_py. ' ...
             'Available outputs: %s'], strjoin(available, ', '));
    end
    pxTS = so.get('mb_px');
    pyTS = so.get('mb_py');
    if ~isa(pxTS,'timeseries') || ~isa(pyTS,'timeseries')
        error('ElevationViewer:PositionSignalType', ...
            'simOut.mb_px and simOut.mb_py must be timeseries objects.');
    end

    tx = double(pxTS.Time(:));
    xData = double(squeeze(pxTS.Data));
    xData = xData(:);
    ty = double(pyTS.Time(:));
    yData = double(squeeze(pyTS.Data));
    yData = yData(:);
    if numel(tx) ~= numel(xData) || numel(ty) ~= numel(yData) || ...
            numel(tx) < 2 || numel(ty) < 2
        error('ElevationViewer:PositionSignalSize', ...
            'The mb_px/mb_py time and data vectors are inconsistent.');
    end

    % Align Y onto the X timestamps if the logged sample times differ.
    [ty, iy] = unique(ty,'stable');
    yData = yData(iy);
    [tx, ix] = unique(tx,'stable');
    xData = xData(ix);
    yAligned = interp1(ty,yData,tx,'linear',NaN);
    valid = isfinite(tx) & isfinite(xData) & isfinite(yAligned) & ...
        xData >= min(X) & xData <= max(X) & ...
        yAligned >= min(Y) & yAligned <= max(Y);
    tx = tx(valid);
    xData = xData(valid);
    yAligned = yAligned(valid);
    if numel(tx) < 2
        error('ElevationViewer:SimulationTrajectoryRange', ...
            'Fewer than two logged poses fall within the terrain bounds.');
    end

    % Resample to approximately 30 frames/second so playback is smooth and
    % follows simulation time instead of requiring thousands of plot updates.
    duration = tx(end)-tx(1);
    targetCount = max(2,min(600,round(duration*30)+1));
    frameTime = linspace(tx(1),tx(end),targetCount)';
    poseXY = [interp1(tx,xData,frameTime,'linear'), ...
              interp1(tx,yAligned,frameTime,'linear')];

    % Estimate chassis heading from the direction of travel. Smooth the
    % position slightly first; this avoids heading jitter from small solver noise.
    smoothWindow = min(7, targetCount);
    if mod(smoothWindow,2)==0
        smoothWindow = max(3,smoothWindow-1);
    end
    xSmooth = smoothdata(poseXY(:,1),'movmean',smoothWindow);
    ySmooth = smoothdata(poseXY(:,2),'movmean',smoothWindow);
    vx = gradient(xSmooth,frameTime);
    vy = gradient(ySmooth,frameTime);
    speed = hypot(vx,vy);
    moving = speed > 0.03;
    if nnz(moving) >= 2
        movingYaw = unwrap(atan2(vy(moving),vx(moving)));
        yaw = interp1(frameTime(moving),movingYaw,frameTime,'linear','extrap');
        yaw = unwrap(yaw(:));
    elseif nnz(moving)==1
        yaw = repmat(atan2(vy(find(moving,1)),vx(find(moving,1))),targetCount,1);
    else
        yaw = zeros(targetCount,1);
    end
    playbackDescription = sprintf('Actual simulated XY motion (%.2f s)',duration);

else
    if evalin('base','exist(''originalPath'',''var'')') == 1
        pathXY = double(evalin('base','originalPath'));
    elseif evalin('base','exist(''originalPathPadded'',''var'')') == 1
        pathXY = double(evalin('base','originalPathPadded'));
    else
        error('ElevationViewer:MissingPath', ...
            ['No simulation output or path found. Keep simOut in the base workspace, ' ...
             'or load originalPath/originalPathPadded.']);
    end
    if size(pathXY,2) ~= 2
        error('ElevationViewer:PathShape', 'Path must be an N-by-2 XY matrix.');
    end
    valid = all(isfinite(pathXY),2) & ...
        pathXY(:,1) >= min(X) & pathXY(:,1) <= max(X) & ...
        pathXY(:,2) >= min(Y) & pathXY(:,2) <= max(Y);
    pathXY = pathXY(valid,:);
    if size(pathXY,1) < 2
        error('ElevationViewer:PathRange', ...
            'The available path has fewer than two points inside the terrain bounds.');
    end
    keep = [true; hypot(diff(pathXY(:,1)), diff(pathXY(:,2))) > 1e-7];
    poseXY = pathXY(keep,:);
    if size(poseXY,1) < 2
        error('ElevationViewer:PathDuplicates', 'Path contains fewer than two unique points.');
    end
    dx = gradient(poseXY(:,1));
    dy = gradient(poseXY(:,2));
    yaw = atan2(dy,dx);
    frameTime = (1:size(poseXY,1))';
    playbackDescription = 'Planned path preview (not simulated motion)';
end

% Keep only points for which terrain height can be interpolated.
groundPathZ = interp2(X, Y, Z, poseXY(:,1), poseXY(:,2), 'linear');
validPose = isfinite(groundPathZ);
poseXY = poseXY(validPose,:);
yaw = yaw(validPose);
frameTime = frameTime(validPose);
if size(poseXY,1) < 2
    error('ElevationViewer:NoInterpolatedPose', ...
        'Could not interpolate terrain height for at least two rover poses.');
end

fig = figure('Name','Elevation-Colored Terrain + Rover Viewer', ...
    'NumberTitle','off','Color','w','Position',[80 80 1280 820]);
ax = axes('Parent',fig,'Position',[0.07 0.16 0.78 0.78]);
surf(ax, X, Y, Z, Z, 'EdgeColor','none', 'FaceColor','interp');
shading(ax,'interp');
colormap(ax,turbo(256));
clim(ax,[min(Z(:)), max(Z(:))]);
cb = colorbar(ax);
cb.Label.String = 'Elevation Z (m)';
hold(ax,'on');

% Draw obstacles from the canonical scenarioConfig when available. These are
% visualization-only; physical collision geometry is still controlled by Simulink.
hasScenarioConfig = evalin('base', 'exist(''scenarioConfig'',''var'')') == 1;
if hasScenarioConfig
    scenario = evalin('base','scenarioConfig');
    if ~isfield(scenario,'obstacles') || isempty(scenario.obstacles)
        error('ElevationViewer:ScenarioObstacles', ...
            'scenarioConfig exists but scenarioConfig.obstacles is empty or invalid.');
    end
    allObs = scenario.obstacles(:);
    isDynamicObs = [allObs.isDynamic];
    staticObs = allObs(~isDynamicObs);
    dynamicObs = allObs(isDynamicObs);
    staticX = arrayfun(@(o)o.centerXY(1), staticObs)';
    staticY = arrayfun(@(o)o.centerXY(2), staticObs)';
    staticDims = vertcat(staticObs.dimensions);
    staticNames = {staticObs.name};
    nStatic = numel(staticObs);
    % Multiple dynamic obstacles are supported. Each retains its own geometry
    % and spawn time from scenarioConfig.
else
    % Backwards-compatible fallback for a workspace created before scenarioConfig.
    staticX = [];
    staticY = [];
    if evalin('base', 'exist(''obstacleXs'',''var'')') == 1 && ...
            evalin('base', 'exist(''obstacleYs'',''var'')') == 1
        staticX = double(evalin('base','obstacleXs'));
        staticY = double(evalin('base','obstacleYs'));
        staticX = staticX(:);
        staticY = staticY(:);
        nStatic = min(numel(staticX),numel(staticY));
        staticX = staticX(1:nStatic);
        staticY = staticY(1:nStatic);
    else
        nStatic = 0;
    end
    obstacleLength = 1.0;
    obstacleWidth = 1.0;
    obstacleHeight = 0.5;
    if evalin('base','exist(''obstacleLength'',''var'')') == 1
        obstacleLength = double(evalin('base','obstacleLength'));
    end
    if evalin('base','exist(''obstacleWidth'',''var'')') == 1
        obstacleWidth = double(evalin('base','obstacleWidth'));
    end
    if evalin('base','exist(''obstacleHeight'',''var'')') == 1
        obstacleHeight = double(evalin('base','obstacleHeight'));
    end
    staticDims = repmat([obstacleLength obstacleWidth obstacleHeight],nStatic,1);
    staticNames = arrayfun(@(i)sprintf('Obstacle %d',i),1:nStatic,'UniformOutput',false);
    dynamicObs = struct.empty;
end

boxFaces = [1 2 3 4; 5 8 7 6; 1 5 6 2; ...
            2 6 7 3; 3 7 8 4; 4 8 5 1];
staticPatches = gobjects(nStatic,1);
staticLabels = gobjects(nStatic,1);
for oi = 1:nStatic
    dims = staticDims(oi,:);
    [vObs, zTop] = makeObstacleBox(staticX(oi),staticY(oi), ...
        dims(1),dims(2),dims(3));
    staticPatches(oi) = patch(ax,'Vertices',vObs,'Faces',boxFaces, ...
        'FaceColor',[0.90 0.20 0.10], ...
        'EdgeColor',[0.20 0.035 0.02],'LineWidth',1.1, ...
        'FaceAlpha',0.97);
    % Use compact, staggered labels to avoid overlap when obstacles cluster.
    labelX = staticX(oi) + 0.18*(oi-1);
    labelY = staticY(oi) + dims(2)/2 + 0.32 + 0.22*(oi-1);
    staticLabels(oi) = text(ax,labelX,labelY,zTop+0.08, ...
        sprintf('O%d',oi),'Color',[0.20 0.035 0.02], ...
        'FontWeight','bold','FontSize',8, ...
        'HorizontalAlignment','center','VerticalAlignment','bottom');
end

if hasScenarioConfig
    nDynamic = numel(dynamicObs);
    if nDynamic > 0
        dynamicX = arrayfun(@(o)o.centerXY(1), dynamicObs)';
        dynamicY = arrayfun(@(o)o.centerXY(2), dynamicObs)';
        dynamicDims = vertcat(dynamicObs.dimensions);
        dynamicSpawnTime = reshape([dynamicObs.spawnTime], [], 1);
        dynamicName = reshape({dynamicObs.name}, [], 1);
        if isfield(dynamicObs,'id')
            dynamicIds = reshape([dynamicObs.id], [], 1);
        else
            dynamicIds = (nStatic + (1:nDynamic))';
        end
    else
        dynamicX = zeros(0,1);
        dynamicY = zeros(0,1);
        dynamicDims = zeros(0,3);
        dynamicSpawnTime = zeros(0,1);
        dynamicName = cell(0,1);
        dynamicIds = zeros(0,1);
    end
else
    % Backwards-compatible fallback for a workspace created before scenarioConfig.
    hasDynamicObstacle = evalin('base','exist(''obstacle3X'',''var'')') == 1 && ...
        evalin('base','exist(''obstacle3Y'',''var'')') == 1;
    if hasDynamicObstacle
        dynamicX = double(evalin('base','obstacle3X'));
        dynamicY = double(evalin('base','obstacle3Y'));
        dynamicX = dynamicX(:);
        dynamicY = dynamicY(:);
        nDynamic = min(numel(dynamicX), numel(dynamicY));
        dynamicX = dynamicX(1:nDynamic);
        dynamicY = dynamicY(1:nDynamic);
        dynamicDims = repmat([obstacleLength obstacleWidth obstacleHeight], nDynamic, 1);
        dynamicName = repmat({'Dynamic obstacle'}, nDynamic, 1);
        dynamicIds = (nStatic + (1:nDynamic))';
        if evalin('base','exist(''obstacleSpawnTime'',''var'')') == 1
            spawnValue = double(evalin('base','obstacleSpawnTime'));
            dynamicSpawnTime = repmat(spawnValue, nDynamic, 1);
        else
            dynamicSpawnTime = repmat(8.0, nDynamic, 1);
            warning('ElevationViewer:MissingSpawnTime', ...
                'obstacleSpawnTime is missing; using the known configured value 8 s.');
        end
    else
        dynamicX = zeros(0,1);
        dynamicY = zeros(0,1);
        dynamicDims = zeros(0,3);
        dynamicSpawnTime = zeros(0,1);
        dynamicName = cell(0,1);
        dynamicIds = zeros(0,1);
    end
    nDynamic = numel(dynamicX);
end

% Create a separate patch and label for every dynamic obstacle. Visibility is
% controlled independently from the recorded simulation time in updateRover.
dynamicPatches = gobjects(nDynamic,1);
dynamicLabels = gobjects(nDynamic,1);
for di = 1:nDynamic
    dims = dynamicDims(di,:);
    [vDyn,zDynTop] = makeObstacleBox(dynamicX(di), dynamicY(di), ...
        dims(1), dims(2), dims(3));
    dynamicPatches(di) = patch(ax,'Vertices',vDyn,'Faces',boxFaces, ...
        'FaceColor',[0.05 0.55 0.95], ...
        'EdgeColor',[0.02 0.10 0.28],'LineWidth',1.3, ...
        'FaceAlpha',0.98,'Visible','off');

    if isfinite(dynamicSpawnTime(di))
        timingText = sprintf('t=%.1f s', dynamicSpawnTime(di));
    else
        timingText = 'spawn time undefined';
    end
    dynamicLabels(di) = text(ax, ...
        dynamicX(di)+dims(1)/2+0.35, ...
        dynamicY(di)-dims(2)/2-0.22, zDynTop+0.08, ...
        sprintf('O%d: %s (%s)', dynamicIds(di), dynamicName{di}, timingText), ...
        'Color',[0.02 0.10 0.28],'FontWeight','bold','FontSize',8, ...
        'HorizontalAlignment','left','VerticalAlignment','bottom', ...
        'Visible','off');
end

% Planned or recorded rover ground track, draped onto the terrain.
plot3(ax, poseXY(:,1), poseXY(:,2), groundPathZ + 0.035, ...
    '-', 'Color',[0.05 0.05 0.05], 'LineWidth',1.5);
plot3(ax, poseXY(1,1), poseXY(1,2), groundPathZ(1)+0.05, ...
    'o','MarkerFaceColor',[0.1 0.8 0.2],'MarkerEdgeColor','k','MarkerSize',7);
plot3(ax, poseXY(end,1), poseXY(end,2), groundPathZ(end)+0.05, ...
    's','MarkerFaceColor',[0.95 0.2 0.1],'MarkerEdgeColor','k','MarkerSize',7);

% Rover is a lightweight visualization proxy, not a replacement for CAD.
% Approximate dimensions can be adjusted below to better match the rover.
roverLength = 1.00;   % m
roverWidth  = 0.66;   % chassis width, m
wheelRadius = 0.14;   % m
wheelWidth  = 0.10;   % m
chassisBottom = wheelRadius + 0.055;
chassisTop = chassisBottom + 0.19;
wheelX = roverLength/2 - wheelRadius*1.15;
wheelY = roverWidth/2 + wheelWidth*0.42;

roverRoot = hgtransform('Parent',ax);
V = [-roverLength/2 -roverWidth/2 chassisBottom; ...
      roverLength/2 -roverWidth/2 chassisBottom; ...
      roverLength/2  roverWidth/2 chassisBottom; ...
     -roverLength/2  roverWidth/2 chassisBottom; ...
     -roverLength/2 -roverWidth/2 chassisTop; ...
      roverLength/2 -roverWidth/2 chassisTop; ...
      roverLength/2  roverWidth/2 chassisTop; ...
     -roverLength/2  roverWidth/2 chassisTop];
F = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
patch('Parent',roverRoot,'Vertices',V,'Faces',F, ...
    'FaceColor',[0.78 0.12 0.06],'EdgeColor',[0.2 0.05 0.03], ...
    'LineWidth',0.5);

% Raised top panel and forward marker (+X is the rover's forward direction).
Vtop = [-0.26 -0.22 chassisTop+0.005; 0.30 -0.22 chassisTop+0.005; ...
         0.30  0.22 chassisTop+0.005; -0.26 0.22 chassisTop+0.005];
patch('Parent',roverRoot,'Vertices',Vtop,'Faces',[1 2 3 4], ...
    'FaceColor',[0.14 0.22 0.28],'EdgeColor','none');
line('Parent',roverRoot,'XData',[0.20 0.43],'YData',[0 0], ...
    'ZData',[chassisTop+0.02 chassisTop+0.02], ...
    'Color',[1 0.9 0.2],'LineWidth',3);

[ux,uy,uz] = sphere(16);
for sx = [-1 1]
    for sy = [-1 1]
        cx = sx*wheelX;
        cy = sy*wheelY;
        surf('Parent',roverRoot, ...
            'XData',cx + wheelRadius*ux, ...
            'YData',cy + (wheelWidth/2)*uy, ...
            'ZData',wheelRadius + wheelRadius*uz, ...
            'FaceColor',[0.10 0.10 0.11], ...
            'EdgeColor','none');
    end
end

xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Elevation Z (m)');
title(ax,playbackDescription,'Interpreter','none');
view(ax,45,34);
axis(ax,'tight');
grid(ax,'on');
box(ax,'on');
% Slight vertical exaggeration helps reveal terrain features in the viewer.
 daspect(ax,[1 1 0.7]);
camlight(ax,'headlight');
lighting(ax,'gouraud');

% Simple route scrubbing/playback controls.
slider = uicontrol(fig,'Style','slider','Units','normalized', ...
    'Position',[0.12 0.075 0.57 0.035], ...
    'Min',1,'Max',size(poseXY,1),'Value',1, ...
    'SliderStep',[1/(size(poseXY,1)-1), min(10/(size(poseXY,1)-1),1)], ...
    'Callback',@sliderMoved);
playButton = uicontrol(fig,'Style','togglebutton','Units','normalized', ...
    'Position',[0.72 0.064 0.13 0.055], ...
    'String','Play path','Callback',@playClicked);
status = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.12 0.025 0.73 0.03], ...
    'BackgroundColor','w','HorizontalAlignment','left', ...
    'String','');
updateRover(1);

    function sliderMoved(src,~)
        idx = round(get(src,'Value'));
        set(src,'Value',idx);
        updateRover(idx);
    end

    function updateRover(idx)
        idx = max(1,min(size(poseXY,1),idx));
        x = poseXY(idx,1);
        y = poseXY(idx,2);
        zGround = interp2(X,Y,Z,x,y,'linear');
        if ~isfinite(zGround)
            return;
        end
        roverRoot.Matrix = makehgtform('translate',[x y zGround]) * ...
            makehgtform('zrotate',yaw(idx));
        activeDynamicCount = 0;
        for di = 1:nDynamic
            if isTimeBasedPlayback
                showDynamic = isfinite(dynamicSpawnTime(di)) && ...
                    frameTime(idx) >= dynamicSpawnTime(di);
            else
                % A path-only preview has no physical clock; show dynamic
                % obstacles as ghosts instead of implying timed playback.
                showDynamic = true;
            end

            if showDynamic
                set(dynamicPatches(di),'Visible','on');
                set(dynamicLabels(di),'Visible','on');
                activeDynamicCount = activeDynamicCount + 1;
            else
                set(dynamicPatches(di),'Visible','off');
                set(dynamicLabels(di),'Visible','off');
            end
        end

        if nDynamic == 0
            obstacleSummary = sprintf(' | obstacles: %d static; no dynamic obstacles',nStatic);
        else
            obstacleSummary = sprintf(' | obstacles: %d static + %d/%d dynamic active', ...
                nStatic,activeDynamicCount,nDynamic);
        end
        set(status,'String',sprintf('%s | frame %d/%d | t %.3f s | XY = (%.2f, %.2f) m | ground Z = %.3f m%s', ...
            playbackDescription,idx,size(poseXY,1),frameTime(idx),x,y,zGround,obstacleSummary));
        drawnow limitrate;
    end

    function [vertices,zTop] = makeObstacleBox(cx,cy,len,wid,hgt)
        % Place the base just above the highest terrain sample under the box
        % footprint to avoid rendering it through the terrain surface.
        ix = find(X >= cx-len/2 & X <= cx+len/2);
        iy = find(Y >= cy-wid/2 & Y <= cy+wid/2);
        if isempty(ix)
            [~,ix] = min(abs(X-cx));
        end
        if isempty(iy)
            [~,iy] = min(abs(Y-cy));
        end
        baseZ = max(Z(iy,ix),[],'all') + 0.008;
        x0 = cx-len/2; x1 = cx+len/2;
        y0 = cy-wid/2; y1 = cy+wid/2;
        z0 = baseZ; z1 = baseZ+hgt;
        vertices = [x0 y0 z0; x1 y0 z0; x1 y1 z0; x0 y1 z0; ...
                    x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1];
        zTop = z1;
    end

    function playClicked(src,~)
        if get(src,'Value') == 1
            set(src,'String','Pause');
            for n = round(get(slider,'Value')):size(poseXY,1)
                if ~ishandle(fig) || ~ishandle(src) || get(src,'Value') ~= 1
                    break;
                end
                set(slider,'Value',n);
                updateRover(n);
                if startsWith(playbackDescription,'Actual simulated XY motion') && ...
                        n < numel(frameTime)
                    pause(max(0,frameTime(n+1)-frameTime(n)));
                else
                    pause(0.04);
                end
            end
            if ishandle(src)
                set(src,'Value',0,'String','Play path');
            end
        else
            set(src,'String','Play path');
        end
    end
end
