
%% VISUALIZE TERRAIN UNDER THE THREE PHYSICAL OBSTACLES
% Diagnostic only. Does not modify any Simulink model.

projectRoot = ...
'C:\Users\sagar\OneDrive\Documents\AutonomousNavigationRoughTerrain';

scenarioFile = fullfile(projectRoot, ...
    'scenarioConfig_customization_demo.mat');

assert(exist('X','var') && exist('Y','var') && ...
       exist('Z_multibody','var'), ...
       'Run setupOffRoadProject.m first.');

S = load(scenarioFile,'scenarioConfig');
cfg = S.scenarioConfig;

obs = cfg.obstacles(:);
[~,order] = sort([obs.id]);
obs = obs(order);

figure('Name','Custom Physical Obstacle Terrain Inspection', ...
       'Color','w','Position',[100 100 1550 600]);

tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

xVec = X(1,:);
yVec = Y(:,1);

for k = 1:3

    o = obs(k);

    cx = o.centerXY(1);
    cy = o.centerXY(2);
    dx = o.dimensions(1);
    dy = o.dimensions(2);

    ix = find(xVec >= cx-dx/2 & xVec <= cx+dx/2);
    iy = find(yVec >= cy-dy/2 & yVec <= cy+dy/2);

    assert(numel(ix)>=2 && numel(iy)>=2, ...
        'Insufficient terrain samples for obstacle %d.',o.id);

    Xp = X(iy,ix);
    Yp = Y(iy,ix);
    Zp = Z_multibody(iy,ix);

    validZ = Zp(isfinite(Zp));
    assert(~isempty(validZ),'No valid terrain samples.');

    zCentre = interp2(xVec,yVec,Z_multibody,cx,cy,'linear');
    z95 = prctile(validZ,95);
    zMax = max(validZ);

    ax = nexttile;

    surf(ax,Xp,Yp,Zp,'EdgeColor','none');
    hold(ax,'on');

    % Centre-based bottom elevation
    zCentreBottom = zCentre + 0.005;

    % 95th-percentile bottom elevation
    z95Bottom = z95 + 0.005;

    % Obstacle footprint boundary
    xb = [cx-dx/2 cx+dx/2 cx+dx/2 cx-dx/2 cx-dx/2];
    yb = [cy-dy/2 cy-dy/2 cy+dy/2 cy+dy/2 cy-dy/2];

    plot3(ax,xb,yb, ...
        zCentreBottom*ones(size(xb)), ...
        'r-','LineWidth',2);

    plot3(ax,xb,yb, ...
        z95Bottom*ones(size(xb)), ...
        'm--','LineWidth',2);

    % Terrain centre and maximum point
    plot3(ax,cx,cy,zCentre,'ko', ...
        'MarkerFaceColor','w','MarkerSize',7);

    [~,maxIdx] = max(validZ);
    validIndices = find(isfinite(Zp));
    [rMax,cMax] = ind2sub(size(Zp),validIndices(maxIdx));

    plot3(ax,Xp(rMax,cMax),Yp(rMax,cMax),zMax, ...
        'kp','MarkerFaceColor','y','MarkerSize',10);

    title(ax,sprintf('Obstacle %d | %g x %g x %g m', ...
        o.id,dx,dy,o.dimensions(3)));

    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');
    zlabel(ax,'Elevation (m)');

    grid(ax,'on');
    view(ax,42,30);
    axis(ax,'tight');
    colorbar(ax);

    fprintf('\nObstacle %d\n',o.id);
    fprintf('Centre-based bottom: %.4f m\n',zCentreBottom);
    fprintf('95th percentile bottom: %.4f m\n',z95Bottom);
    fprintf('Maximum terrain: %.4f m\n',zMax);
end

sgtitle('Terrain Variation Beneath Physical Obstacles');

legend(nexttile(1), ...
    {'Terrain','Centre-based level','95th-percentile level', ...
     'Centre','Maximum terrain point'}, ...
    'Location','best');
