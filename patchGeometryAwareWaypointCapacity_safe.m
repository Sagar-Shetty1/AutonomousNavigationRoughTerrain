%% patchGeometryAwareWaypointCapacity_safe
% Safely compresses only forward-collinear route points before the fixed-size
% output-capacity check. It does not add local functions or modify Simulink models.

scriptPath = mfilename('fullpath');
projectRoot = fileparts(scriptPath);
targetFile = fullfile(projectRoot, 'onlineReplanFromPoseGeometryAware.m');

assert(isfile(targetFile), 'Planner file not found: %s', targetFile);
sourceText = fileread(targetFile);
% Normalize line endings so the exact block replacement is consistent.
sourceText = strrep(sourceText, sprintf('\r\n'), sprintf('\n'));

oldLines = {
    'waypointCount = size(path,1);'
    'assert(waypointCount <= maxWaypoints, ...'
    '    ''Geometry-aware route has %d waypoints; output capacity is %d.'', ...'
    '    waypointCount,maxWaypoints);'
    'routeLength = sum(vecnorm(diff(path,1,1),2,2));'
};
oldBlock = strjoin(oldLines, sprintf('\n'));
occurrences = numel(strfind(sourceText, oldBlock)); %#ok<STREMP>
assert(occurrences == 1, ...
    ['Expected route-capacity block exactly once, found %d. ' ...
     'No file changes made.'], occurrences);

newLines = {
    '% Compress only intermediate points on the same forward straight segment.'
    '% This preserves the XY path geometry; the clearance validation below reruns on it.'
    'if size(path,1) > 2'
    '    compressedPath = path(1,:);'
    '    for pathIdx = 2:size(path,1)'
    '        compressedPath(end+1,:) = path(pathIdx,:); %#ok<AGROW>'
    '        while size(compressedPath,1) >= 3'
    '            pointA = compressedPath(end-2,:);'
    '            pointB = compressedPath(end-1,:);'
    '            pointC = compressedPath(end,:);'
    '            vec1 = pointB - pointA;'
    '            vec2 = pointC - pointB;'
    '            cross2D = vec1(1)*vec2(2) - vec1(2)*vec2(1);'
    '            collinearTol = 1e-10 * max(1, norm(vec1)*norm(vec2));'
    '            if abs(cross2D) <= collinearTol && dot(vec1,vec2) >= -collinearTol'
    '                compressedPath(end-1,:) = [];'
    '            else'
    '                break;'
    '            end'
    '        end'
    '    end'
    '    pointsBeforeCompression = size(path,1);'
    '    path = compressedPath;'
    '    fprintf(''Collinear waypoint compression: %d -> %d waypoints.\n'', ...'
    '        pointsBeforeCompression, size(path,1));'
    'end'
    'waypointCount = size(path,1);'
    'assert(waypointCount <= maxWaypoints, ...'
    '    ''Geometry-aware route still has %d waypoints after straight-segment compression; output capacity is %d.'', ...'
    '    waypointCount,maxWaypoints);'
    'routeLength = sum(vecnorm(diff(path,1,1),2,2));'
};
newBlock = strjoin(newLines, sprintf('\n'));
patchedText = strrep(sourceText, oldBlock, newBlock);

% Create a timestamped byte-for-byte backup before writing the source.
backupFile = fullfile(projectRoot, ...
    ['onlineReplanFromPoseGeometryAware_preSafeWaypointCompression_' ...
     datestr(now, 'yyyymmdd_HHMMSS') '.m']);
copyfile(targetFile, backupFile);
fprintf('Planner backup created: %s\n', backupFile);

% Write the patched source, then ask MATLAB to parse it via nargin.
fid = fopen(targetFile, 'w');
assert(fid >= 0, 'Could not open planner file for writing: %s', targetFile);
cleanupObj = onCleanup(@() fclose(fid));
fwrite(fid, patchedText, 'char');
clear cleanupObj;

try
    clear onlineReplanFromPoseGeometryAware
    rehash
    actualNargin = nargin('onlineReplanFromPoseGeometryAware');
    assert(actualNargin == 6, ...
        'Planner parsed but has unexpected input count %d (expected 6).', actualNargin);
catch ME
    copyfile(backupFile, targetFile, 'f');
    clear onlineReplanFromPoseGeometryAware
    rehash
    fprintf(2, 'Patch validation failed; original planner restored from backup.\n');
    rethrow(ME);
end

fprintf('\n=== SAFE WAYPOINT-CAPACITY PATCH APPLIED ===\n');
fprintf('Planner: %s\n', targetFile);
fprintf('MATLAB parser check passed; nargin = %d.\n', actualNargin);
fprintf(['Only consecutive forward-collinear intermediate points are removed. ' ...
    'The existing route-clearance validation runs after compression.\n']);
fprintf(['The planner and Simulink models were not simulated or saved by this script. ' ...
    'Next, preview the route/workflow; the capacity assertion remains active.\n']);
