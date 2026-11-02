function previewGeometryAwareObstaclePlanner
%% previewGeometryAwareObstaclePlanner
% READ-ONLY PREVIEW of geometry-aware A* obstacle avoidance.
% Does not modify Simulink models, scripts, or scenario variables.
% It copies plannerFootprint.Graph, blocks links crossing conservative
% obstacle rectangles inflated by the rover footprint radius + a small margin,
% and plots an initial route (static obstacles) plus a post-spawn route (all
% configured obstacles) on the current terrain.
%
% Requirements in the base workspace:
%   scenarioConfig, plannerFootprint, vehicleClearanceRadius, X, Y, Z, simOut
%
% Current scope: axis-aligned box obstacles from scenarioConfig. The expanded
% rectangles are conservative square-cornered envelopes, not an exact rounded
% Minkowski sum. Use this only to compare routes before modifying live logic.

requiredVars = {'scenarioConfig','plannerFootprint', ...
    'vehicleClearanceRadius','X','Y','Z','simOut'};
for k = 1:numel(requiredVars)
    assert(evalin('base', sprintf('exist(''%s'',''var'')', requiredVars{k})) == 1, ...
        'Missing workspace variable "%s". Run startRoughTerrainProject and a simulation first.', ...
        requiredVars{k});
end

cfg = evalin('base','scenarioConfig');
plannerBase = evalin('base','plannerFootprint');
vehicleClearanceRadius = evalin('base','vehicleClearanceRadius');
X = evalin('base','X');
Y = evalin('base','Y');
Z = evalin('base','Z');
simOut = evalin('base','simOut');

obs = cfg.obstacles(:);
startXY = double(cfg.startPose(1:2));
goalXY  = double(cfg.goalPose(1:2));
footprintRadius = double(vehicleClearanceRadius);
extraMargin = 0.10; % m, additional clearance beyond rover footprint

assert(isfinite(footprintRadius) && footprintRadius > 0, ...
    'vehicleClearanceRadius must be a positive finite scalar.');
assert(all(arrayfun(@(o) strcmpi(o.shape,'box'),obs)), ...
    'This preview currently supports box obstacles only.');

staticMask = ~[obs.isDynamic];
staticObs = obs(staticMask);
allObs = obs;

% Route 1: initial planning before the dynamic obstacle spawns.
[routeInitial, nBlockedInitial] = planGeometryRoute( ...
    plannerBase, startXY, goalXY, staticObs, ...
    footprintRadius, extraMargin);

% Route 2: replan from the first logged replan event at/after the dynamic
% obstacle's spawn time. This uses a recorded event pose, not a guessed pose.
dynamicObs = obs([obs.isDynamic]);
assert(~isempty(dynamicObs), 'No dynamic obstacle is configured.');
spawnTime = double(dynamicObs(1).spawnTime);

eventLog = simOut.get('onlineReplanEventLog');
replanXLog = simOut.get('replanStartXLog');
replanYLog = simOut.get('replanStartYLog');
eventMask = eventLog.Data(:) >= 0.5 & eventLog.Time(:) >= spawnTime;
postSpawnEventIdx = find(eventMask, 1, 'first');
assert(~isempty(postSpawnEventIdx), ...
    'No logged replanning event at/after %.2f s. Run the current 45 s simulation first.', spawnTime);
postSpawnTime = double(eventLog.Time(postSpawnEventIdx));
postSpawnStart = [double(replanXLog.Data(postSpawnEventIdx)), ...
                  double(replanYLog.Data(postSpawnEventIdx))];
assert(all(isfinite(postSpawnStart)), ...
    'Logged post-spawn replan start pose is not finite.');

[routeAfterSpawn, nBlockedAfterSpawn] = planGeometryRoute( ...
    plannerBase, postSpawnStart, goalXY, allObs, ...
    footprintRadius, extraMargin);

routeLenInitial = sum(vecnorm(diff(routeInitial,1,1),2,2));
routeLenAfter = sum(vecnorm(diff(routeAfterSpawn,1,1),2,2));

% Draw elevation background (standard MATLAB X/Y/Z grid convention).
fig = figure('Name','Geometry-Aware Obstacle Planner Preview', ...
    'Color','w','Position',[100 80 1150 820]);
ax = axes('Parent',fig);
contourf(ax, X, Y, Z, 30, 'LineColor','none');
colormap(ax,turbo(256));
cb = colorbar(ax);
cb.Label.String = 'Terrain elevation Z (m)';
hold(ax,'on');

% Draw actual box footprints and conservative inflated envelopes.
for k = 1:numel(allObs)
    o = allObs(k);
    cx = o.centerXY(1); cy = o.centerXY(2);
    len = o.dimensions(1); wid = o.dimensions(2);
    x0 = cx-len/2; y0 = cy-wid/2;
    isDyn = logical(o.isDynamic);
    if isDyn
        boxColor = [0.08 0.42 0.90];
        boxStyle = '--';
        label = sprintf('%s (spawn %.1f s)', o.name, o.spawnTime);
    else
        boxColor = [0.85 0.15 0.08];
        boxStyle = '-';
        label = o.name;
    end
    rectangle(ax,'Position',[x0 y0 len wid], ...
        'EdgeColor',boxColor,'LineWidth',1.8,'LineStyle',boxStyle, ...
        'FaceColor','none');
    text(ax,cx,cy,label,'Color',boxColor,'FontWeight','bold', ...
        'FontSize',8,'HorizontalAlignment','center', ...
        'VerticalAlignment','bottom');

    infl = footprintRadius + extraMargin;
    rectangle(ax,'Position',[x0-infl y0-infl len+2*infl wid+2*infl], ...
        'EdgeColor',boxColor,'LineWidth',1.0,'LineStyle',':', ...
        'FaceColor','none');
end

hInitial = plot(ax,routeInitial(:,1),routeInitial(:,2),'-', ...
    'Color',[0.05 0.25 0.90],'LineWidth',2.0);
hAfterSpawn = plot(ax,routeAfterSpawn(:,1),routeAfterSpawn(:,2),'-', ...
    'Color',[0.65 0.05 0.70],'LineWidth',2.0);
hStart = plot(ax,startXY(1),startXY(2),'o','MarkerSize',8, ...
    'MarkerFaceColor',[0.15 0.8 0.25],'MarkerEdgeColor','k');
hGoal = plot(ax,goalXY(1),goalXY(2),'s','MarkerSize',8, ...
    'MarkerFaceColor',[1.0 0.85 0.10],'MarkerEdgeColor','k');
hPostSpawnStart = plot(ax,postSpawnStart(1),postSpawnStart(2),'d','MarkerSize',7, ...
    'MarkerFaceColor',[0.95 0.45 0.95],'MarkerEdgeColor','k');

axis(ax,'equal'); axis(ax,'tight'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'Geometry-aware A* route preview (existing model not modified)');
legend(ax,[hInitial hAfterSpawn hStart hGoal hPostSpawnStart], ...
    {'Initial route (static obstacles)', ...
     'Post-spawn route (all configured obstacles)', ...
     'Configured start','Goal','Logged post-spawn replan start'}, ...
    'Location','bestoutside');

fprintf('\n=== GEOMETRY-AWARE PLANNER PREVIEW ===\n');
fprintf('Rover footprint radius: %.3f m\n',footprintRadius);
fprintf('Extra margin: %.3f m\n',extraMargin);
fprintf('Initial route: %.3f m, %d waypoints, %d directed links blocked\n', ...
    routeLenInitial,size(routeInitial,1),nBlockedInitial);
fprintf('Post-spawn event time: %.3f s\n',postSpawnTime);
fprintf('Post-spawn replan start: (%.3f, %.3f) m\n', ...
    postSpawnStart(1),postSpawnStart(2));
fprintf('Post-spawn route: %.3f m, %d waypoints, %d directed links blocked\n', ...
    routeLenAfter,size(routeAfterSpawn,1),nBlockedAfterSpawn);
fprintf(['The plotted dotted envelopes are conservative rectangles expanded ', ...
    'by rover footprint radius + %.2f m. No Simulink model was changed.\n'],extraMargin);

end % main function

%% Local helper functions
function [path, nBlocked] = planGeometryRoute(plannerBase, startXY, goalXY, obstacles, roverRadius, margin)
    graphBase = plannerBase.Graph;
    graphCandidate = copy(graphBase);
    S = graphCandidate.States.StateVector;
    linkPairs = graphCandidate.Links.EndStates;
    A = S(linkPairs(:,1),:);
    B = S(linkPairs(:,2),:);
    blockedMask = false(size(linkPairs,1),1);

    for j = 1:numel(obstacles)
        o = obstacles(j);
        center = double(o.centerXY(:)');
        dims = double(o.dimensions(:)');
        assert(numel(center)==2 && numel(dims)>=2 && ...
            all(isfinite(center)) && all(isfinite(dims(1:2))) && all(dims(1:2)>0), ...
            'Invalid obstacle center/dimensions for %s.', o.name);
        if isfield(o,'yaw') && isfinite(o.yaw) && abs(o.yaw)>1e-9
            error('Rotated boxes are not yet supported by this preview (%s).',o.name);
        end
        % Conservative box inflation: axis-aligned half dimensions plus the
        % rover circumscribed radius and a small extra clearance margin.
        hx = dims(1)/2 + roverRadius + margin;
        hy = dims(2)/2 + roverRadius + margin;
        blockedMask = blockedMask | segmentIntersectsAABB( ...
            A,B,center(1)-hx,center(1)+hx,center(2)-hy,center(2)+hy);
    end

    blockedPairs = unique(linkPairs(blockedMask,:),'rows');
    nBlocked = size(blockedPairs,1);
    if ~isempty(blockedPairs)
        rmlink(graphCandidate,blockedPairs);
    end

    [~,startIdx] = min(sum((S-double(startXY(:)')).^2,2));
    [~,goalIdx] = min(sum((S-double(goalXY(:)')).^2,2));
    startNode = S(startIdx,:);
    goalNode = S(goalIdx,:);

    % Fail rather than silently plan from inside an inflated obstacle.
    for j = 1:numel(obstacles)
        o = obstacles(j);
        c = double(o.centerXY(:)');
        d = double(o.dimensions(:)');
        hx = d(1)/2 + roverRadius + margin;
        hy = d(2)/2 + roverRadius + margin;
        assert(~pointInAABB(startNode,c(1)-hx,c(1)+hx,c(2)-hy,c(2)+hy), ...
            'Start graph state lies inside inflated obstacle %s.',o.name);
        assert(~pointInAABB(goalNode,c(1)-hx,c(1)+hx,c(2)-hy,c(2)+hy), ...
            'Goal graph state lies inside inflated obstacle %s.',o.name);
    end

    planner = plannerAStar(graphCandidate, ...
        'HeuristicCostFcn',plannerBase.HeuristicCostFcn, ...
        'TieBreaker',plannerBase.TieBreaker);
    path = plan(planner,startNode,goalNode);
    assert(~isempty(path) && size(path,1)>=2, ...
        'A* found no route with the current geometry-aware obstacle envelopes.');

    % Verify every route segment against all the same inflated rectangles.
    PA = path(1:end-1,:); PB = path(2:end,:);
    for j = 1:numel(obstacles)
        o = obstacles(j); c = double(o.centerXY(:)'); d = double(o.dimensions(:)');
        hx = d(1)/2 + roverRadius + margin;
        hy = d(2)/2 + roverRadius + margin;
        hits = segmentIntersectsAABB(PA,PB,c(1)-hx,c(1)+hx,c(2)-hy,c(2)+hy);
        assert(~any(hits), 'Route validation failed near %s.',o.name);
    end
end

function hit = segmentIntersectsAABB(A,B,xmin,xmax,ymin,ymax)
    % Vectorized Liang-Barsky segment-vs-axis-aligned-rectangle test.
    n = size(A,1);
    dx = B(:,1)-A(:,1);
    dy = B(:,2)-A(:,2);
    % Clip each segment against the rectangle's four bounds.
    tEnter = zeros(n,1); tExit = ones(n,1); hit = true(n,1);
    [hit,tEnter,tExit] = clipOne(hit,tEnter,tExit,-dx,A(:,1)-xmin);
    [hit,tEnter,tExit] = clipOne(hit,tEnter,tExit, dx,xmax-A(:,1));
    [hit,tEnter,tExit] = clipOne(hit,tEnter,tExit,-dy,A(:,2)-ymin);
    [hit,tEnter,tExit] = clipOne(hit,tEnter,tExit, dy,ymax-A(:,2));
    hit = hit & (tEnter <= tExit + 1e-12);
end

function [hit,tEnter,tExit] = clipOne(hit,tEnter,tExit,p,q)
    parallel = abs(p) < 1e-12;
    hit(parallel & q < 0) = false;
    idx = ~parallel & p < 0;
    r = zeros(size(p));
    r(~parallel) = q(~parallel)./p(~parallel);
    tEnter(idx) = max(tEnter(idx),r(idx));
    idx = ~parallel & p > 0;
    tExit(idx) = min(tExit(idx),r(idx));
    hit = hit & (tEnter <= tExit + 1e-12);
end

function yes = pointInAABB(p,xmin,xmax,ymin,ymax)
    yes = p(1)>=xmin && p(1)<=xmax && p(2)>=ymin && p(2)<=ymax;
end
