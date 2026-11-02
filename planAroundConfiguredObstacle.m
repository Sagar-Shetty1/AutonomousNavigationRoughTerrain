
%% PLAN AROUND MULTIPLE CONFIGURABLE OBSTACLES

mdl = 'OffRoadMultibodyStage1_RandomNavigation';

assert(bdIsLoaded(mdl), ...
    'Open the RandomNavigation model first.');

assert(exist('newPath','var') == 1 && size(newPath,1) >= 2, ...
    'The original newPath must exist.');

assert(exist('obstacleXs','var') == 1 && ...
       exist('obstacleYs','var') == 1, ...
    'Define obstacleXs and obstacleYs first.');

assert(numel(obstacleXs) == numel(obstacleYs), ...
    'Obstacle X and Y arrays must have the same length.');

assert(exist('plannerFootprint','var') == 1, ...
    'Run the project setup first.');

%% 1. Configuration

originalPath = newPath;
obstacleXs = obstacleXs(:);
obstacleYs = obstacleYs(:);

obstacleCenters = [obstacleXs obstacleYs];
numObstacles = size(obstacleCenters,1);

goalX = originalPath(end,1);
goalY = originalPath(end,2);
goalPoint = [goalX goalY];

sensorRange = 7.0;
halfFOV = deg2rad(30);
inflatedRadius = 3.5;

fprintf('\n===== MULTI-OBSTACLE PLANNING =====\n');
fprintf('Number of obstacles: %d\n', numObstacles);
fprintf('Destination: (%.2f, %.2f) m\n', goalX, goalY);
fprintf('Planning radius per obstacle: %.2f m\n', inflatedRadius);

disp('Configured obstacle centres:');
disp(obstacleCenters);

%% 2. Predict the first sensor detection on the original route

detectionIndex = [];
detectedObstacle = NaN;

for k = 1:size(originalPath,1)-1

    roverPos = originalPath(k,:);
    direction = originalPath(k+1,:) - roverPos;
    yaw = atan2(direction(2), direction(1));

    for j = 1:numObstacles

        dx = obstacleCenters(j,1) - roverPos(1);
        dy = obstacleCenters(j,2) - roverPos(2);

        range = hypot(dx,dy);
        bearing = atan2(dy,dx) - yaw;
        bearing = atan2(sin(bearing),cos(bearing));

        if range <= sensorRange && abs(bearing) <= halfFOV
            detectionIndex = k;
            detectedObstacle = j;
            break;
        end
    end

    if ~isempty(detectionIndex)
        break;
    end
end

if isempty(detectionIndex)
    error('No configured obstacle is detectable along the original route.');
end

detectionPoint = originalPath(detectionIndex,:);

fprintf('Predicted detection point: (%.2f, %.2f) m\n', ...
    detectionPoint(1), detectionPoint(2));
fprintf('First detected obstacle: %d\n', detectedObstacle);

%% 3. Snap detection and goal to graph states

graphOriginal = plannerFootprint.Graph;
S = graphOriginal.States.StateVector;

[~, startIdx] = min(sum((S-detectionPoint).^2,2));
startNode = S(startIdx,:);

[~, goalIdx] = min(sum((S-goalPoint).^2,2));
goalNode = S(goalIdx,:);

% Do not start the replacement path inside any exclusion region
startDistances = hypot( ...
    startNode(1)-obstacleXs, startNode(2)-obstacleYs);

if any(startDistances <= inflatedRadius)
    error(['The predicted detection point is too close to an ', ...
        'obstacle for this planning radius. Increase sensor ', ...
        'range or select another obstacle placement.']);
end

%% 4. Copy graph and remove links intersecting any obstacle region

graphDynamic = copy(graphOriginal);
linkPairs = graphDynamic.Links.EndStates;

A = S(linkPairs(:,1),:);
B = S(linkPairs(:,2),:);
V = B-A;

len2 = sum(V.^2,2);

blockedMask = false(size(linkPairs,1),1);

for j = 1:numObstacles

    center = obstacleCenters(j,:);

    tau = sum((center-A).*V,2)./len2;
    tau = max(0,min(1,tau));

    closest = A + tau.*V;

    d = sqrt(sum((closest-center).^2,2));

    blockedMask = blockedMask | (d <= inflatedRadius);
end

blockedPairs = unique(linkPairs(blockedMask,:),'rows');

fprintf('Directed graph links to remove: %d\n', ...
    size(blockedPairs,1));

if ~isempty(blockedPairs)
    rmlink(graphDynamic,blockedPairs);
end

%% 5. Create planner for modified graph

plannerDynamic = plannerAStar( ...
    graphDynamic, ...
    'HeuristicCostFcn',plannerFootprint.HeuristicCostFcn, ...
    'TieBreaker',plannerFootprint.TieBreaker);

%% 6. Find one route around all obstacles

[pathFromDetection,solutionInfo] = ...
    plan(plannerDynamic,startNode,goalNode);

if isempty(pathFromDetection) || size(pathFromDetection,1) < 2
    error('No route exists around all configured obstacles.');
end

routeLength = sum(vecnorm(diff(pathFromDetection,1,1),2,2));

%% 7. Validate clearance from every obstacle

pathClearance = inf(numObstacles,1);

A2 = pathFromDetection(1:end-1,:);
B2 = pathFromDetection(2:end,:);
V2 = B2-A2;

len2Path = sum(V2.^2,2);

for j = 1:numObstacles

    center = obstacleCenters(j,:);

    tau = sum((center-A2).*V2,2)./len2Path;
    tau = max(0,min(1,tau));

    closest = A2 + tau.*V2;

    pathClearance(j) = min( ...
        sqrt(sum((closest-center).^2,2)));
end

disp('Planned clearance from each obstacle centre:');
disp(pathClearance);

if any(pathClearance < inflatedRadius-1e-6)
    error('The replacement path failed obstacle-clearance validation.');
end

fprintf('Replacement route points: %d\n', ...
    size(pathFromDetection,1));
fprintf('Replacement route length: %.2f m\n',routeLength);

%% 8. Pad both route matrices to the same dimensions

n = max(size(originalPath,1),size(pathFromDetection,1));

originalPathPadded = [
    originalPath
    repmat(originalPath(end,:),n-size(originalPath,1),1)
];

replannedPathPadded = [
    pathFromDetection
    repmat(pathFromDetection(end,:), ...
        n-size(pathFromDetection,1),1)
];

%% 9. Update Simulink Constant blocks

set_param([mdl '/PathWaypoints'], ...
    'Value','originalPathPadded');

set_param([mdl '/ReplannedPath'], ...
    'Value','replannedPathPadded');

set_param([mdl '/ObstacleX'], ...
    'Value','obstacleXs');

set_param([mdl '/ObstacleY'], ...
    'Value','obstacleYs');

set_param(mdl,'SimulationCommand','update');

%% 10. Plot the routes and obstacle regions

theta = linspace(0,2*pi,200);

figure;
plot(originalPath(:,1),originalPath(:,2), ...
    'b--','LineWidth',1.3);
hold on;

plot(pathFromDetection(:,1),pathFromDetection(:,2), ...
    'm-','LineWidth',1.8);

for j = 1:numObstacles
    plot(obstacleXs(j)+inflatedRadius*cos(theta), ...
         obstacleYs(j)+inflatedRadius*sin(theta), ...
         'r-','LineWidth',1.5);
end

plot(detectionPoint(1),detectionPoint(2), ...
    'ko','MarkerFaceColor','c');

plot(goalX,goalY,'ks','MarkerFaceColor','y');

axis equal;
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('A* Replanning Around Multiple Obstacles');
legend('Original route','Replacement route', ...
    'Obstacle exclusion regions','Detection point', ...
    'Destination','Location','best');

hold off;

fprintf('\nMulti-obstacle route generated and model updated.\n');
