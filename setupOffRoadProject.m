%% setupOffRoadProject
% Rebuilds terrain, terrain costs, planning graph, and baseline waypoints.
% Workspace only: it deliberately does not load older Simulink models.
% Use startRoughTerrainProject.m for the active online-replanning model.

%% 1. Core simulation / vehicle parameters
wheelBase = 1.0;
maxSteeringAngle = pi/4;
vehicleSpeedRange = [-Inf Inf];

desiredVelocity = 0.3;
lookaheadTime = 2.0;                  % 0.3 * 2.0 = 0.6 m lookahead
lookaheadDistance = desiredVelocity * lookaheadTime;
maxCurvature = tan(maxSteeringAngle) / wheelBase;

startPose = [2 2 pi/4];               % [x y theta]
startState = [2; 2; pi/4; 0];         % Ackermann [x y theta psi]
goalPose = [48 48 0];
start = startPose(1:2);
goal = goalPose(1:2);

%% 2. Terrain: 50 m x 50 m
[X,Y] = meshgrid(0:0.1:50, 0:0.1:50);
Z = zeros(size(X));

% Hill A
qA1 = ((X-10)/7).^2 + ((Y-12)/5.5).^2;
qA2 = ((X-7.5)/4.5).^2 + ((Y-14)/3.5).^2;
hillA = max(0,1-qA1).^2 + 0.35*max(0,1-qA2).^2;
hillA = hillA / max(hillA(:));
Z = Z + 1.2*hillA;

% Hill B
qB1 = ((X-36)/6.5).^2 + ((Y-14)/5).^2;
qB2 = ((X-39)/4).^2 + ((Y-12)/3).^2;
hillB = max(0,1-qB1).^2 + 0.30*max(0,1-qB2).^2;
hillB = hillB / max(hillB(:));
Z = Z + 0.9*hillB;

% Central ridge / hill
qR1 = ((X-25)/8).^2 + ((Y-34)/5).^2;
qR2 = ((X-30)/5).^2 + ((Y-35)/3.5).^2;
ridge = max(0,1-qR1).^2 + 0.25*max(0,1-qR2).^2;
ridge = ridge / max(ridge(:));
Z = Z + 1.0*ridge;

% Dips
qD1 = ((X-12)/5).^2 + ((Y-39)/5.5).^2;
dip1 = max(0,1-qD1).^2;
Z = Z - 0.6*dip1;

qD2 = ((X-43)/2.5).^2 + ((Y-36)/2.8).^2;
dip2 = max(0,1-qD2).^2;
Z = Z - 1.2*dip2;

qD3 = ((X-20)/6.5).^2 + ((Y-25)/5.5).^2;
dip3 = max(0,1-qD3).^2;
Z = Z - 0.15*dip3;

%% 3. Terrain slope and terrain cost
terrainStep = 0.1;
[dZdy,dZdx] = gradient(Z,terrainStep,terrainStep);
slopeMagnitude = sqrt(dZdx.^2 + dZdy.^2);
slope = atan(slopeMagnitude);
slopeDeg = rad2deg(slope);

terrainCost = ones(size(slopeDeg));
terrainCost(slopeDeg > 5  & slopeDeg <= 10) = 2;
terrainCost(slopeDeg > 10 & slopeDeg <= 15) = 4;
terrainCost(slopeDeg > 15 & slopeDeg <= 20) = 8;
terrainCost(slopeDeg > 20 & slopeDeg <= 25) = 15;
terrainCost(slopeDeg > 25) = 30;

%% 4. Artificial / man-made obstacles
obstacleMask = false(size(X));

obstacleMask = obstacleMask | ...
    (X >= 18 & X <= 21 & Y >= 12 & Y <= 20);

obstacleMask = obstacleMask | ...
    (X >= 30 & X <= 34 & Y >= 24 & Y <= 30);

obstacleMask = obstacleMask | ...
    (X >= 39 & X <= 44 & Y >= 29 & Y <= 32);

%% 5. Vehicle footprint approximation and obstacle inflation
vehicleLength = 0.91;
vehicleWidth = 0.91;
vehicleClearanceRadius = 0.5 * sqrt(vehicleLength^2 + vehicleWidth^2);

distanceFromObstacle = bwdist(obstacleMask) * terrainStep;
obstacleMaskInflated = distanceFromObstacle <= vehicleClearanceRadius;

%% 6. Planning grid (0.5 m)
planningStep = 5;
Xg = X(1:planningStep:end,1:planningStep:end);
Yg = Y(1:planningStep:end,1:planningStep:end);
Cg = terrainCost(1:planningStep:end,1:planningStep:end);

[nRows,nCols] = size(Xg);
states = [Xg(:) Yg(:)];

% 8-connected graph links
links = [];
for r = 1:nRows
    for c = 1:nCols
        current = sub2ind([nRows,nCols],r,c);
        if c < nCols
            links(end+1,:) = [current sub2ind([nRows,nCols],r,c+1)]; %#ok<AGROW>
        end
        if r < nRows
            links(end+1,:) = [current sub2ind([nRows,nCols],r+1,c)]; %#ok<AGROW>
        end
        if r < nRows && c < nCols
            links(end+1,:) = [current sub2ind([nRows,nCols],r+1,c+1)]; %#ok<AGROW>
        end
        if r < nRows && c > 1
            links(end+1,:) = [current sub2ind([nRows,nCols],r+1,c-1)]; %#ok<AGROW>
        end
    end
end
links = [links; links(:,[2 1])];
linkTable = table(links,'VariableNames',{'EndStates'});

%% 7. Reusable terrain-aware link-cost function
terrainLinkWeight = @(id1,id2,g) ...
    sqrt(sum((g.States.StateVector(id2,:) - g.States.StateVector(id1,:)).^2,2)) .* ...
    ((g.States.TerrainCost(id1) + g.States.TerrainCost(id2))/2);

%% 8. Terrain-only A* planner
stateTable = table(states,Cg(:), ...
    'VariableNames',{'StateVector','TerrainCost'});

graphObj = navGraph(stateTable,linkTable);
graphObj.LinkWeightFcn = terrainLinkWeight;

planner = plannerAStar(graphObj);
planner.HeuristicCostFcn = @nav.algs.distanceEuclidean;

[path,solutionInfo] = plan(planner,start,goal);
simplifiedPath = reducepoly(path,0.01);
waypointsTerrainOnly = simplifiedPath;

%% 9. Original-obstacle-aware A* planner
obstacleMask_g = obstacleMask(1:planningStep:end,1:planningStep:end);
planningCost = Cg;
planningCost(obstacleMask_g) = planningCost(obstacleMask_g) + 1000;

stateTableObstacle = table(states,planningCost(:), ...
    'VariableNames',{'StateVector','TerrainCost'});

graphObjObstacle = navGraph(stateTableObstacle,linkTable);
graphObjObstacle.LinkWeightFcn = terrainLinkWeight;
plannerObstacle = plannerAStar(graphObjObstacle);
plannerObstacle.HeuristicCostFcn = @nav.algs.distanceEuclidean;

[pathObstacle,solutionInfoObstacle] = plan(plannerObstacle,start,goal);
simplifiedObstaclePath = reducepoly(pathObstacle,0.005);
waypointsObstacle = simplifiedObstaclePath;

%% 10. Vehicle-footprint-aware A* planner
inflatedObstacleMask_g = obstacleMaskInflated(1:planningStep:end,1:planningStep:end);
footprintPlanningCost = Cg;
footprintPlanningCost(inflatedObstacleMask_g) = ...
    footprintPlanningCost(inflatedObstacleMask_g) + 1000;

stateTableFootprint = table(states,footprintPlanningCost(:), ...
    'VariableNames',{'StateVector','TerrainCost'});

graphObjFootprint = navGraph(stateTableFootprint,linkTable);
graphObjFootprint.LinkWeightFcn = terrainLinkWeight;
plannerFootprint = plannerAStar(graphObjFootprint);
plannerFootprint.HeuristicCostFcn = @nav.algs.distanceEuclidean;

[pathFootprint,solutionInfoFootprint] = ...
    plan(plannerFootprint,start,goal);

simplifiedFootprintPath = reducepoly(pathFootprint,0.005);
waypointsFootprint = simplifiedFootprintPath;

%% 11. Active vehicle waypoints: validated footprint-aware route
waypoints = waypointsFootprint;


%% 13. Restore previously saved experiment results, if available
% Planning variables above are always rebuilt from source definitions.
% Saved results are restored separately and are not used to override the
% freshly reconstructed terrain, planner, or active waypoint set.
resultsFile = fullfile(fileparts(mfilename('fullpath')), 'offRoadResults.mat');

if isfile(resultsFile)
    savedResults = load(resultsFile);

    if isfield(savedResults,'footprintResults')
        footprintResults = savedResults.footprintResults;
    end

    if isfield(savedResults,'obstacleResults')
        obstacleResults = savedResults.obstacleResults;
    end

    fprintf('Saved experiment results  : loaded from offRoadResults.mat\n');
else
    fprintf('Saved experiment results  : none found\n');
end

%% 12. Helpful diagnostics
startID = find(states(:,1)==start(1) & states(:,2)==start(2),1);
goalID = find(states(:,1)==goal(1) & states(:,2)==goal(2),1);

fprintf('\n============================================\n');
fprintf(' OFF-ROAD PROJECT WORKSPACE READY\n');
fprintf('============================================\n');
fprintf('Terrain grid             : %d x %d\n',size(X,1),size(X,2));
fprintf('Planning grid            : %d x %d\n',nRows,nCols);
fprintf('Terrain-only path length : %.3f m\n', ...
    sum(vecnorm(diff(waypointsTerrainOnly),2,2)));
fprintf('Obstacle path length     : %.3f m\n', ...
    sum(vecnorm(diff(waypointsObstacle),2,2)));
fprintf('Footprint path length    : %.3f m\n', ...
    sum(vecnorm(diff(waypointsFootprint),2,2)));
fprintf('Active waypoints         : %d\n',size(waypoints,1));
fprintf('Footprint radius         : %.3f m\n',vehicleClearanceRadius);
fprintf('Start cost               : %.2f\n',Cg(startID));
fprintf('Goal cost                : %.2f\n',Cg(goalID));
fprintf('Lookahead distance       : %.3f m\n',lookaheadDistance);
fprintf('============================================\n');


%% ============================================================
% Multibody rough-terrain workspace
% =============================================================

roughAmplitude = 0.035;        % 3 cm surface roughness
terrainVerticalScale = 1.6;   % exaggerate terrain for visualization

roughness = ...
    roughAmplitude * ...
    (0.45*sin(2*pi*X/1.8) .* sin(2*pi*Y/2.2) + ...
    0.30*sin(2*pi*X/0.9 + 1.2) .* ...
    sin(2*pi*Y/1.4));

Z_multibody = terrainVerticalScale * Z + roughness;

% Reduce terrain grid for Multibody performance
mbStep = 2;

X_mb = X(1,1:mbStep:end);
Y_mb = Y(1:mbStep:end,1);

% Grid Surface convention used by the model
Z_mb = Z_multibody(1:mbStep:end,1:mbStep:end)';

fprintf('\nMultibody terrain restored\n');
fprintf('  Roughness amplitude : %.3f m\n', roughAmplitude);
fprintf('  Vertical scale      : %.2f\n', terrainVerticalScale);
fprintf('  Grid size            : %d x %d\n', ...
    size(Z_mb,1), size(Z_mb,2));

%% Workspace-only setup complete
% This script intentionally does not load or update any Simulink model.
% Use startRoughTerrainProject.m to initialize the active online-replanning
% scenario and load OffRoadMultibodyStage1_OnlineReplanCheckpoint.

fprintf('\nWorkspace setup complete (no Simulink models loaded by this script).\n');
