%% saveOffRoadResults
% Save the currently available project experiment results without failing
% when one result structure is not present in the current MATLAB session.

if ~exist('footprintResults','var')
    error('footprintResults is not available. Run the footprint-aware simulation and metrics first.');
end

if exist('obstacleResults','var')
    save('offRoadResults.mat','footprintResults','obstacleResults','waypoints');
    fprintf('Saved footprintResults, obstacleResults, and waypoints.\n');
else
    save('offRoadResults.mat','footprintResults','waypoints');
    fprintf('Saved footprintResults and waypoints.\n');
    fprintf('obstacleResults was not present, so it was not saved.\n');
end

fprintf('File: offRoadResults.mat\n');
