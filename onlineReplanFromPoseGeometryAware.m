function [routePadded, waypointCount, clearances, routeLength] = ...
    onlineReplanFromPoseGeometryAware(currentXY, goalXY, ...
    obstacleXs, obstacleYs, inflatedRadius, maxWaypoints)
% onlineReplanFromPoseGeometryAware
% TEST-ONLY geometry-aware replanner. Does NOT replace onlineReplanFromPose.
% Uses scenarioConfig box dimensions when a detected obstacle center matches
% a configured obstacle, otherwise falls back to the legacy circular radius.
%
% For configured axis-aligned boxes, each rectangle is conservatively
% expanded by the rover circumscribed footprint radius + geometry margin.
% The expansion is an axis-aligned conservative envelope, not an exact
% rounded Minkowski sum. Rotated boxes are rejected rather than mis-planned.
%
% Inputs/outputs intentionally match onlineReplanFromPose for easy comparison.

%% Validate inputs
validateattributes(currentXY, {'numeric'}, ...
    {'real','finite','vector','numel',2}, mfilename,'currentXY');
validateattributes(goalXY, {'numeric'}, ...
    {'real','finite','vector','numel',2}, mfilename,'goalXY');
validateattributes(obstacleXs, {'numeric'}, ...
    {'real','vector'}, mfilename,'obstacleXs');
validateattributes(obstacleYs, {'numeric'}, ...
    {'real','vector'}, mfilename,'obstacleYs');
validateattributes(inflatedRadius, {'numeric'}, ...
    {'real','finite','scalar','positive'}, mfilename,'inflatedRadius');
validateattributes(maxWaypoints, {'numeric'}, ...
    {'real','finite','scalar','integer','>=',2}, mfilename,'maxWaypoints');

currentXY = double(currentXY(:)');
goalXY = double(goalXY(:)');
obstacleXs = double(obstacleXs(:));
obstacleYs = double(obstacleYs(:));
assert(numel(obstacleXs) == numel(obstacleYs), ...
    'Obstacle X and Y arrays must have equal lengths.');
valid = isfinite(obstacleXs) & isfinite(obstacleYs);
obstacleXs = obstacleXs(valid);
obstacleYs = obstacleYs(valid);
assert(~isempty(obstacleXs), 'At least one finite obstacle is required.');
centers = [obstacleXs obstacleYs];
nObs = size(centers,1);

%% Read shared project configuration and graph
plannerBase = evalin('base','plannerFootprint');
configAvailable = evalin('base', ...
    'exist(''scenarioConfig'',''var'') == 1');
if configAvailable
    cfg = evalin('base','scenarioConfig');
else
    cfg = struct();
end

footprintRadius = NaN;
if evalin('base','exist(''vehicleClearanceRadius'',''var'') == 1')
    footprintRadius = evalin('base','vehicleClearanceRadius');
end
extraMargin = 0.10;
if isfield(cfg,'planning') && isfield(cfg.planning,'geometryMargin')
    extraMargin = double(cfg.planning.geometryMargin);
end
assert(isfinite(extraMargin) && extraMargin >= 0, ...
    'scenarioConfig.planning.geometryMargin must be nonnegative.');

G = plannerBase.Graph;
S = G.States.StateVector;
assert(size(S,2)==2, 'Expected a 2-D graph with X/Y states.');
[~,startIdx] = min(sum((S-currentXY).^2,2));
[~,goalIdx] = min(sum((S-goalXY).^2,2));
startNode = S(startIdx,:);
goalNode = S(goalIdx,:);

%% Match detected centers to configured obstacle geometry
isBox = false(nObs,1);
boxHalfX = nan(nObs,1);
boxHalfY = nan(nObs,1);
boxName = cell(nObs,1);
requiredClearance = inflatedRadius * ones(nObs,1);

cfgObs = [];
if isfield(cfg,'obstacles')
    cfgObs = cfg.obstacles(:);
end

for j = 1:nObs
    boxName{j} = sprintf('Obstacle %d',j);
    if isempty(cfgObs) || ~isfinite(footprintRadius) || footprintRadius <= 0
        continue
    end

    cfgCenters = vertcat(cfgObs.centerXY);
    distances = hypot(cfgCenters(:,1)-centers(j,1), ...
                      cfgCenters(:,2)-centers(j,2));
    [matchDist, matchIdx] = min(distances);

    if matchDist > 1e-3
        % Unknown obstacle: retain legacy circular treatment.
        continue
    end

    o = cfgObs(matchIdx);
    boxName{j} = o.name;
    if ~strcmpi(o.shape,'box')
        % Non-box shapes stay on the established circular fallback until
        % their exact collision/planning representation is implemented.
        continue
    end

    yaw = 0;
    if isfield(o,'yaw') && ~isempty(o.yaw)
        yaw = double(o.yaw);
    end
    if abs(yaw) > 1e-9
        error('Rotated box "%s" is not yet supported.', o.name);
    end

    dims = double(o.dimensions(:)');
    assert(numel(dims)>=2 && all(isfinite(dims(1:2))) && ...
        all(dims(1:2)>0), 'Invalid XY dimensions for %s.',o.name);
    isBox(j) = true;
    boxHalfX(j) = dims(1)/2 + footprintRadius + extraMargin;
    boxHalfY(j) = dims(2)/2 + footprintRadius + extraMargin;
end

%% Preserve the existing hard start-safety guard
safetyRadius = 2.25;
startDistances = hypot(startNode(1)-centers(:,1), ...
                        startNode(2)-centers(:,2));
bad = find(startDistances <= safetyRadius);
if ~isempty(bad)
    fprintf(2,'\n===== GEOMETRY-AWARE REPLANNING SAFETY GUARD =====\n');
    fprintf(2,'Measured rover position: (%.3f, %.3f) m\n', ...
        currentXY(1),currentXY(2));
    fprintf(2,'Snapped graph state: (%.3f, %.3f) m\n', ...
        startNode(1),startNode(2));
    for k = bad(:)'
        fprintf(2,'Obstacle %d centre (%.3f, %.3f), distance %.3f m\n', ...
            k,centers(k,1),centers(k,2),startDistances(k));
    end
    error(['Replanning start is inside the existing 2.25 m hard safety ', ...
        'radius. Inspect detection timing and obstacle coordinates.']);
end

% Legacy circular fallback can reduce its requested radius when the rover
% is already within the nominal radius but outside the hard safety guard.
for j = 1:nObs
    if ~isBox(j) && startDistances(j) < inflatedRadius
        requiredClearance(j) = safetyRadius;
    end
end

%% Copy graph and block links that intersect inflated boxes or circles
Gdyn = copy(G);
linkPairs = Gdyn.Links.EndStates;
A = S(linkPairs(:,1),:);
B = S(linkPairs(:,2),:);
V = B-A;
len2 = sum(V.^2,2);
blocked = false(size(linkPairs,1),1);

for j = 1:nObs
    if isBox(j)
        cx = centers(j,1); cy = centers(j,2);
        hx = boxHalfX(j); hy = boxHalfY(j);
        blocked = blocked | segmentIntersectsAABB( ...
            A,B,cx-hx,cx+hx,cy-hy,cy+hy);
    else
        denom = max(len2,eps);
        tauJ = sum((centers(j,:)-A).*V,2)./denom;
        tauJ = max(0,min(1,tauJ));
        closest = A + tauJ.*V;
        d = sqrt(sum((closest-centers(j,:)).^2,2));
        blocked = blocked | d <= requiredClearance(j);
    end
end

blockedPairs = unique(linkPairs(blocked,:),'rows');
fprintf('\n===== GEOMETRY-AWARE ONLINE A* REPLANNING (TEST VARIANT) =====\n');
fprintf('Current rover pose: (%.3f, %.3f) m\n',currentXY(1),currentXY(2));
fprintf('Snapped start state: (%.3f, %.3f) m\n',startNode(1),startNode(2));
fprintf('Destination: (%.3f, %.3f) m\n',goalNode(1),goalNode(2));
fprintf('Detected obstacles: %d\n',nObs);
fprintf('Directed graph links removed: %d\n',size(blockedPairs,1));
for j = 1:nObs
    if isBox(j)
        fprintf('%s: box envelope half-extents [%.3f, %.3f] m\n', ...
            boxName{j},boxHalfX(j),boxHalfY(j));
    else
        fprintf('%s: circular fallback clearance %.3f m\n', ...
            boxName{j},requiredClearance(j));
    end
end
if ~isempty(blockedPairs)
    rmlink(Gdyn,blockedPairs);
end

%% Replan on the copied graph
plannerDyn = plannerAStar(Gdyn, ...
    'HeuristicCostFcn',plannerBase.HeuristicCostFcn, ...
    'TieBreaker',plannerBase.TieBreaker);
path = plan(plannerDyn,startNode,goalNode);
assert(~isempty(path) && size(path,1)>=2, ...
    'A* found no route with the geometry-aware obstacle envelopes.');
% Compress only intermediate points on the same forward straight segment.
% This preserves the XY path geometry; the clearance validation below reruns on it.
if size(path,1) > 2
    compressedPath = path(1,:);
    for pathIdx = 2:size(path,1)
        compressedPath(end+1,:) = path(pathIdx,:); %#ok<AGROW>
        while size(compressedPath,1) >= 3
            pointA = compressedPath(end-2,:);
            pointB = compressedPath(end-1,:);
            pointC = compressedPath(end,:);
            vec1 = pointB - pointA;
            vec2 = pointC - pointB;
            cross2D = vec1(1)*vec2(2) - vec1(2)*vec2(1);
            collinearTol = 1e-10 * max(1, norm(vec1)*norm(vec2));
            if abs(cross2D) <= collinearTol && dot(vec1,vec2) >= -collinearTol
                compressedPath(end-1,:) = [];
            else
                break;
            end
        end
    end
    pointsBeforeCompression = size(path,1);
    path = compressedPath;
    fprintf('Collinear waypoint compression: %d -> %d waypoints.\n', ...
        pointsBeforeCompression, size(path,1));
end
waypointCount = size(path,1);
assert(waypointCount <= maxWaypoints, ...
    'Geometry-aware route still has %d waypoints after straight-segment compression; output capacity is %d.', ...
    waypointCount,maxWaypoints);
routeLength = sum(vecnorm(diff(path,1,1),2,2));

%% Validate every route segment against the same obstacles
pathA = path(1:end-1,:);
pathB = path(2:end,:);
Vpath = pathB-pathA;
pathLen2 = sum(Vpath.^2,2);
clearances = inf(nObs,1); % Legacy output: distance to obstacle centres.
for j = 1:nObs
    denom = max(pathLen2,eps);
    tauJ = sum((centers(j,:)-pathA).*Vpath,2)./denom;
    tauJ = max(0,min(1,tauJ));
    closest = pathA + tauJ.*Vpath;
    clearances(j) = min(sqrt(sum((closest-centers(j,:)).^2,2)));

    if isBox(j)
        cx = centers(j,1); cy = centers(j,2);
        hits = segmentIntersectsAABB(pathA,pathB, ...
            cx-boxHalfX(j),cx+boxHalfX(j), ...
            cy-boxHalfY(j),cy+boxHalfY(j));
        assert(~any(hits), ...
            'Geometry-aware route intersects inflated envelope for %s.',boxName{j});
    else
        assert(clearances(j) >= requiredClearance(j)-1e-6, ...
            'Route failed the circular fallback clearance for %s.',boxName{j});
    end
end

fprintf('Replacement waypoints: %d\n',waypointCount);
fprintf('Replacement route length: %.3f m\n',routeLength);
for j = 1:nObs
    fprintf('%s minimum route distance to centre: %.3f m\n', ...
        boxName{j},clearances(j));
end
fprintf('Route validation against inflated envelopes passed.\n');

routePadded = repmat(path(end,:),maxWaypoints,1);
routePadded(1:waypointCount,:) = path;
fprintf('Output matrix size: %d x %d\n',size(routePadded,1),size(routePadded,2));
end

function hit = segmentIntersectsAABB(A,B,xmin,xmax,ymin,ymax)
% Vectorized Liang-Barsky segment/axis-aligned rectangle intersection.
    n = size(A,1);
    dx = B(:,1)-A(:,1);
    dy = B(:,2)-A(:,2);
    tEnter = zeros(n,1);
    tExit = ones(n,1);
    hit = true(n,1);
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
