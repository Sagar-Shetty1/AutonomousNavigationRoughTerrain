
%% READ-ONLY DIAGNOSTICS AT THE 40s SIMULATION FAILURE LOCATION

modelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';

assert(exist('simOutCustomFailureDiag40s','var') == 1, ...
    'The captured 40-second simulation output is unavailable.');

assert(exist('X','var') == 1 && ...
       exist('Y','var') == 1 && ...
       exist('Z_multibody','var') == 1, ...
    'Terrain workspace variables are unavailable. Run setupOffRoadProject.m.');

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

%% 1. Recover rover trajectory
pxTS = simOutCustomFailureDiag40s.get('mb_px');
pyTS = simOutCustomFailureDiag40s.get('mb_py');

tX = double(pxTS.Time(:));
x = double(pxTS.Data(:));

tY = double(pyTS.Time(:));
y = double(pyTS.Data(:));

n = min([numel(tX),numel(tY),numel(x),numel(y)]);

t = tX(1:n);
x = x(1:n);
y = y(1:n);

failureTime = t(end);
failureX = x(end);
failureY = y(end);

fprintf('\n============================================\n');
fprintf(' ROVER FAILURE LOCATION\n');
fprintf('============================================\n');

fprintf('Last logged time: %.6f s\n',failureTime);
fprintf('Last rover position: [%.6f %.6f] m\n', ...
    failureX,failureY);

%% 2. Estimate rover speed near failure
recent = find(t >= failureTime-1);

if numel(recent) >= 2

    dt = diff(t(recent));
    dx = diff(x(recent));
    dy = diff(y(recent));

    speed = hypot(dx,dy)./max(dt,eps);

    fprintf('Mean speed in final 1 second: %.4f m/s\n',mean(speed));
    fprintf('Maximum speed in final 1 second: %.4f m/s\n',max(speed));
end

fprintf('\nLast 12 rover samples:\n');
fprintf('Time (s)       X (m)       Y (m)\n');

for k = max(1,n-11):n
    fprintf('%.6f    %.6f    %.6f\n',t(k),x(k),y(k));
end

%% 3. Calculate terrain elevation and slope
xVec = double(X(1,:));
yVec = double(Y(:,1));
Z = double(Z_multibody);

dxGrid = mean(diff(xVec));
dyGrid = mean(diff(yVec));

% Gradient along X columns and Y rows
[dZdx,dZdy] = gradient(Z,dxGrid,dyGrid);

terrainZ = interp2(xVec,yVec,Z, ...
    failureX,failureY,'linear');

slopeX = interp2(xVec,yVec,dZdx, ...
    failureX,failureY,'linear');

slopeY = interp2(xVec,yVec,dZdy, ...
    failureX,failureY,'linear');

slopeMagnitude = hypot(slopeX,slopeY);
slopeAngle = atan(slopeMagnitude)*180/pi;

localMask = abs(X-failureX)<=1.5 & ...
            abs(Y-failureY)<=1.5;

localTerrain = Z(localMask);
localTerrain = localTerrain(isfinite(localTerrain));

fprintf('\n============================================\n');
fprintf(' TERRAIN AT FAILURE LOCATION\n');
fprintf('============================================\n');

fprintf('Terrain elevation: %.5f m\n',terrainZ);
fprintf('X gradient: %.5f\n',slopeX);
fprintf('Y gradient: %.5f\n',slopeY);
fprintf('Terrain slope angle: %.3f degrees\n',slopeAngle);

if ~isempty(localTerrain)
    fprintf('Local terrain minimum: %.5f m\n',min(localTerrain));
    fprintf('Local terrain maximum: %.5f m\n',max(localTerrain));
    fprintf('Local terrain variation: %.5f m\n', ...
        max(localTerrain)-min(localTerrain));
end

%% 4. Distances to configured obstacle centres
centres = [
     5.772   17.315
    27.191   18.858
    27.994   27.006
    16.636   13.673
    33.179    7.006
    19.784   40.463
];

fprintf('\n============================================\n');
fprintf(' DISTANCE TO OBSTACLE CENTRES\n');
fprintf('============================================\n');

for k = 1:6
    d = hypot(failureX-centres(k,1), ...
              failureY-centres(k,2));

    fprintf('Obstacle %d: %.4f m\n',k,d);
end

%% 5. Inspect actual Simscape logging branches
if exist('simlog_CustomFailureDiag40s','var')
    simlog = simlog_CustomFailureDiag40s;
else
    simlog = simOutCustomFailureDiag40s.get( ...
        'simlog_CustomFailureDiag40s');
end

blocksToInspect = {
    'RoverTireContact_FL'
    'RoverTireContact_FR'
    'RoverTireContact_RL'
    'RoverTireContact_RR'
    'RoverTireContact_FL/Custom Tire Force and Torque'
    'RoverTireContact_FR/Custom Tire Force and Torque'
    'RoverTireContact_RL/Custom Tire Force and Torque'
    'RoverTireContact_RR/Custom Tire Force and Torque'
    'Grid Surface'
    'PhysicalObstacle1'
    'PhysicalObstacle2'
    'PhysicalObstacle3'
    'Spatial Contact Force Obstacle2'
    'Spatial Contact Force Obstacle3'
};

fprintf('\n============================================\n');
fprintf(' SIMSCAPE LOGGING BRANCHES\n');
fprintf('============================================\n');

for k = 1:numel(blocksToInspect)

    blockPath = [modelName '/' blocksToInspect{k}];

    fprintf('\nBlock: %s\n',blocksToInspect{k});

    try
        node = simscape.logging.findNode(simlog,blockPath);

        if isempty(node)
            fprintf('  No logged node found.\n');
        else
            simscape.logging.print(node);
        end

    catch ME
        fprintf('  Inspection unavailable: %s\n',ME.message);
    end
end

fprintf('\n============================================\n');
fprintf(' FAILURE LOCATION INSPECTION COMPLETE\n');
fprintf(' No model changes made.\n');
fprintf('============================================\n');
