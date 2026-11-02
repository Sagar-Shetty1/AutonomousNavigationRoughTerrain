
%% AUTOMATED MULTI-OBSTACLE VALIDATION

mdl = 'OffRoadMultibodyStage1_RandomNavigation';

if ~bdIsLoaded(mdl)
    load_system(mdl);
end

assert(exist('plannerFootprint','var') == 1, ...
    'Run the project setup before starting validation.');

assert(exist('newPath','var') == 1, ...
    'The original planned route newPath is required.');

% Two previously tested layouts
layouts(1).xs = [10.0 12.0];
layouts(1).ys = [6.5 8.0];

layouts(2).xs = [9.0 11.5];
layouts(2).ys = [6.5 8.0];

numTrials = numel(layouts);
results = struct([]);

set_param(mdl,'StopTime','45');

for trial = 1:numTrials

    fprintf('\n====================================\n');
    fprintf('MULTI-OBSTACLE TRIAL %d OF %d\n', ...
        trial,numTrials);
    fprintf('====================================\n');

    obstacleXs = layouts(trial).xs;
    obstacleYs = layouts(trial).ys;

    % Generate a new path and update the model
    planAroundConfiguredObstacle

    % Run the Simulink rover
    fprintf('Starting simulation...\n');
    out = sim(mdl);

    % Position signals
    px = squeeze(out.mb_px.Data);
    py = squeeze(out.mb_py.Data);
    t  = out.mb_px.Time;

    px = px(:);
    py = py(:);
    t = t(:);

    N = min([numel(px),numel(py),numel(t)]);
    px = px(1:N);
    py = py(1:N);
    t = t(1:N);

    if N < 2
        error('Insufficient position data in trial %d.',trial);
    end

    % Actual distance from the rover to each obstacle
    dObs = hypot(px-obstacleXs(:)',py-obstacleYs(:)');

    % Destination error
    dGoal = hypot(px-goalX,py-goalY);

    % Final two-second movement
    idxFinal = t >= t(end)-2;

    lastMovement = sum(hypot( ...
        diff(px(idxFinal)),diff(py(idxFinal))));

    lastDuration = t(find(idxFinal,1,'last')) - ...
                   t(find(idxFinal,1,'first'));

    % Sensor activation
    sensorTS = out.sensorDetectedLog;
    sensorTime = sensorTS.Time(:);
    sensorValue = squeeze(sensorTS.Data);
    iDetect = find(sensorValue(:) >= 0.5,1,'first');

    if isempty(iDetect)
        detectionTime = NaN;
    else
        detectionTime = sensorTime(iDetect);
    end

    % Store measurements
    results(trial).ObstacleCenters = ...
        [obstacleXs(:),obstacleYs(:)];

    results(trial).SimulationTime = t(end);
    results(trial).FinalPosition = [px(end),py(end)];
    results(trial).FinalDistance = dGoal(end);

    results(trial).MinimumClearance = min(dObs,[],1);
    results(trial).SamplesInsideSafety = ...
        sum(any(dObs < 2.25,2));

    results(trial).DetectionTime = detectionTime;
    results(trial).Last2sMovement = lastMovement;
    results(trial).Last2sAverageSpeed = ...
        lastMovement/lastDuration;

    fprintf('Simulation time: %.2f s\n',t(end));
    fprintf('Final position: (%.2f, %.2f) m\n', ...
        px(end),py(end));
    fprintf('Final distance to goal: %.3f m\n',dGoal(end));
    fprintf('Obstacle 1 minimum clearance: %.3f m\n', ...
        min(dObs(:,1)));
    fprintf('Obstacle 2 minimum clearance: %.3f m\n', ...
        min(dObs(:,2)));
    fprintf('Samples inside 2.25 m safety radius: %d\n', ...
        results(trial).SamplesInsideSafety);
    fprintf('Sensor detection time: %.3f s\n',detectionTime);
    fprintf('Movement during final 2 s: %.3f m\n',lastMovement);
end

%% Summary table
Trial = (1:numTrials)';
FinalDistance_m = [results.FinalDistance]';
MinClearanceObs1_m = arrayfun( ...
    @(r) r.MinimumClearance(1),results)';
MinClearanceObs2_m = arrayfun( ...
    @(r) r.MinimumClearance(2),results)';
SamplesInsideSafety = [results.SamplesInsideSafety]';
DetectionTime_s = [results.DetectionTime]';
Final2sMovement_m = [results.Last2sMovement]';

summaryTable = table(Trial,FinalDistance_m, ...
    MinClearanceObs1_m,MinClearanceObs2_m, ...
    SamplesInsideSafety,DetectionTime_s,Final2sMovement_m);

disp(' ');
disp('===== MULTI-OBSTACLE VALIDATION SUMMARY =====');
disp(summaryTable);

save('multiObstacleValidationResults.mat', ...
    'results','summaryTable');

fprintf('Results saved to multiObstacleValidationResults.mat\n');
