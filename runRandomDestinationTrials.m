
%% AUTOMATED RANDOM DESTINATION TRIALS
% Each trial starts from the rover's initial position.
% Requires plannerFootprint and graphObjFootprint in the workspace.

mdl = 'OffRoadMultibodyStage1_SpeedTest_15ms';

if ~bdIsLoaded(mdl)
    load_system(mdl);
end

assert(exist('plannerFootprint','var') == 1, ...
    'plannerFootprint is missing. Run the project setup first.');
assert(exist('graphObjFootprint','var') == 1, ...
    'graphObjFootprint is missing. Run the project setup first.');

startRequested = [2 2];
S = graphObjFootprint.States.StateVector;

[~, startIdx] = min(sum((S - startRequested).^2, 2));
startNode = S(startIdx,:);

distanceFromStart = hypot(S(:,1)-startNode(1), ...
                          S(:,2)-startNode(2));

eligible = find(distanceFromStart >= 12);
usedGoalIndices = [];

numTrials = 3;
minPathLength = 12;
maxPathLength = 40;

set_param(mdl, 'StopTime', '45');
rng('shuffle');

results = struct([]);

for trip = 1:numTrials

    fprintf('\n====================================\n');
    fprintf('TRIAL %d OF %d\n', trip, numTrials);
    fprintf('====================================\n');

    available = setdiff(eligible, usedGoalIndices);
    candidateOrder = available(randperm(numel(available)));

    pathFound = false;
    maxAttempts = min(200, numel(candidateOrder));

    for k = 1:maxAttempts

        goalIndexCandidate = candidateOrder(k);
        candidateGoal = S(goalIndexCandidate,:);

        try
            [candidatePath, ~] = plan( ...
                plannerFootprint, startNode, candidateGoal);
        catch
            continue;
        end

        if isempty(candidatePath) || ...
                size(candidatePath,1) < 2 || ...
                any(~isfinite(candidatePath(:)))
            continue;
        end

        candidateLength = sum(vecnorm(diff(candidatePath,1,1),2,2));

        if candidateLength < minPathLength || ...
                candidateLength > maxPathLength
            continue;
        end

        newPath = candidatePath;
        goalX = newPath(end,1);
        goalY = newPath(end,2);

        usedGoalIndices(end+1) = goalIndexCandidate;
        pathFound = true;
        break;
    end

    if ~pathFound
        error(['No suitable route found for trial %d. ', ...
               'Check graph connectivity or relax the path-length limits.'], ...
               trip);
    end

    routeLength = sum(vecnorm(diff(newPath,1,1),2,2));

    fprintf('Destination: (%.2f, %.2f) m\n', goalX, goalY);
    fprintf('Route length: %.2f m\n', routeLength);
    fprintf('Running simulation...\n');

    % Update model dimensions/parameters for the new route
    set_param(mdl, 'SimulationCommand', 'update');

    % Run this trip
    simOut = sim(mdl);

    % Extract logged position
    px = squeeze(simOut.mb_px.Data);
    py = squeeze(simOut.mb_py.Data);
    t  = simOut.mb_px.Time;

    px = px(:);
    py = py(:);
    t  = t(:);

    N = min([numel(px), numel(py), numel(t)]);
    px = px(1:N);
    py = py(1:N);
    t = t(1:N);

    if N < 2
        error('Insufficient position samples in trial %d.', trip);
    end

    % Speed and final goal distance
    speed = hypot(gradient(px,t), gradient(py,t));
    idx = t >= min(6,t(end));

    finalDistance = hypot(px(end)-goalX, py(end)-goalY);

    % Movement during the final 2 seconds
    lastIdx = t >= t(end)-2;
    tx = t(lastIdx);
    lx = px(lastIdx);
    ly = py(lastIdx);

    if numel(tx) >= 2
        lastMovement = sum(hypot(diff(lx),diff(ly)));
        lastDuration = tx(end)-tx(1);
        lastAverageSpeed = lastMovement / lastDuration;
    else
        lastMovement = NaN;
        lastAverageSpeed = NaN;
    end

    % Distance from actual trajectory to planned path
    cte = inf(N,1);

    for seg = 1:size(newPath,1)-1
        a = newPath(seg,:);
        b = newPath(seg+1,:);
        v = b-a;
        vv = dot(v,v);

        if vv < eps
            continue;
        end

        tau = ((px-a(1))*v(1) + (py-a(2))*v(2))/vv;
        tau = max(0,min(1,tau));

        projX = a(1)+tau*v(1);
        projY = a(2)+tau*v(2);

        cte = min(cte, hypot(px-projX,py-projY));
    end

    results(trip).goal = [goalX goalY];
    results(trip).routeLength = routeLength;
    results(trip).simulationTime = t(end);
    results(trip).finalPosition = [px(end) py(end)];
    results(trip).finalDistance = finalDistance;
    results(trip).meanSpeed = mean(speed(idx));
    results(trip).rmsPathDeviation = sqrt(mean(cte(idx).^2));
    results(trip).maxPathDeviation = max(cte(idx));
    results(trip).last2sMovement = lastMovement;
    results(trip).last2sAverageSpeed = lastAverageSpeed;

    fprintf('\n--- TRIAL %d RESULTS ---\n', trip);
    fprintf('Simulation time: %.2f s\n', t(end));
    fprintf('Final position: (%.2f, %.2f) m\n', px(end), py(end));
    fprintf('Final distance to goal: %.3f m\n', finalDistance);
    fprintf('Mean ground speed after 6 s: %.3f m/s\n', ...
        mean(speed(idx)));
    fprintf('RMS path deviation: %.3f m\n', ...
        results(trip).rmsPathDeviation);
    fprintf('Maximum path deviation: %.3f m\n', ...
        results(trip).maxPathDeviation);
    fprintf('Final 2 s movement: %.3f m\n', lastMovement);
    fprintf('Final 2 s average speed: %.3f m/s\n', ...
        lastAverageSpeed);
end

% Summary table
Trial = (1:numTrials)';
GoalX = arrayfun(@(r) r.goal(1),results)';
GoalY = arrayfun(@(r) r.goal(2),results)';
RouteLength_m = [results.routeLength]';
FinalDistance_m = [results.finalDistance]';
MeanSpeed_mps = [results.meanSpeed]';
RMSDeviation_m = [results.rmsPathDeviation]';
Final2sMovement_m = [results.last2sMovement]';

summaryTable = table(Trial,GoalX,GoalY,RouteLength_m, ...
    FinalDistance_m,MeanSpeed_mps,RMSDeviation_m, ...
    Final2sMovement_m);

disp(summaryTable);

save('randomNavigationTrialResults.mat', ...
    'results','summaryTable');

fprintf('\nAll three trials completed.\n');
