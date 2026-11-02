function testGeometryAwareReplanning
% Read-only test of the geometry-aware planner variant. This script/function
% does not edit or save any Simulink model and does not replace the live planner.

required = {'scenarioConfig','plannerFootprint','vehicleClearanceRadius','simOut'};
for k = 1:numel(required)
    assert(evalin('base',sprintf('exist(''%s'',''var'') == 1',required{k})), ...
        'Required base-workspace variable "%s" is missing. Run startup and simulation first.',required{k});
end
cfg = evalin('base','scenarioConfig');
simOut = evalin('base','simOut');

obs = cfg.obstacles(:);
static = obs(~[obs.isDynamic]);
allObs = obs;
xsStatic = arrayfun(@(o)o.centerXY(1),static)';
ysStatic = arrayfun(@(o)o.centerXY(2),static)';
xsAll = arrayfun(@(o)o.centerXY(1),allObs)';
ysAll = arrayfun(@(o)o.centerXY(2),allObs)';

fprintf('\n=== GEOMETRY-AWARE PLANNER NON-DESTRUCTIVE TEST ===\n');
[route0,n0,c0,len0] = onlineReplanFromPoseGeometryAware( ...
    cfg.startPose(1:2),cfg.goalPose(1:2),xsStatic,ysStatic, ...
    cfg.planning.obstacleClearance,cfg.planning.maxWaypoints); %#ok<ASGLU>

el = simOut.get('onlineReplanEventLog');
rx = simOut.get('replanStartXLog');
ry = simOut.get('replanStartYLog');
dynamicObs = obs([obs.isDynamic]);
spawnTime = dynamicObs(1).spawnTime;
eventIdx = find(el.Data(:)>=0.5 & el.Time(:)>=spawnTime,1,'first');
assert(~isempty(eventIdx), ...
    'No logged replanning event at or after spawn time %.3f s.',spawnTime);
postTime = el.Time(eventIdx);
postStart = [rx.Data(eventIdx),ry.Data(eventIdx)];

[route1,n1,c1,len1] = onlineReplanFromPoseGeometryAware( ...
    postStart,cfg.goalPose(1:2),xsAll,ysAll, ...
    cfg.planning.obstacleClearance,cfg.planning.maxWaypoints); %#ok<ASGLU>

fprintf('\n=== TEST SUMMARY ===\n');
fprintf('Initial start: [%.2f %.2f], goal: [%.2f %.2f]\n', ...
    cfg.startPose(1),cfg.startPose(2),cfg.goalPose(1),cfg.goalPose(2));
fprintf('Initial box-aware route: %.3f m, %d waypoints\n',len0,n0);
fprintf('Post-spawn event time: %.3f s; recorded start: [%.2f %.2f]\n', ...
    postTime,postStart(1),postStart(2));
fprintf('Post-spawn box-aware route: %.3f m, %d waypoints\n',len1,n1);
fprintf('The route lengths start from different positions and are not directly comparable.\n');
fprintf('No Simulink model or live onlineReplanFromPose.m was changed.\n');

X = evalin('base','X');
Y = evalin('base','Y');
Z = evalin('base','Z');
foot = evalin('base','vehicleClearanceRadius');
margin = 0.10;
if isfield(cfg,'planning') && isfield(cfg.planning,'geometryMargin')
    margin = cfg.planning.geometryMargin;
end

fig = figure('Name','Geometry-aware planner integration test','Color','w');
ax = axes('Parent',fig);
contourf(ax,X,Y,Z,30,'LineColor','none'); hold(ax,'on');
colormap(ax,turbo(256)); cb=colorbar(ax); cb.Label.String='Terrain elevation Z (m)';
for k=1:numel(allObs)
    o=allObs(k); c=o.centerXY; d=o.dimensions;
    if o.isDynamic
        col=[0.08 0.42 0.90]; ls='--';
    else
        col=[0.85 0.15 0.08]; ls='-';
    end
    rectangle(ax,'Position',[c(1)-d(1)/2,c(2)-d(2)/2,d(1),d(2)], ...
        'EdgeColor',col,'LineWidth',1.5,'LineStyle',ls);
    inflX=d(1)/2+foot+margin; inflY=d(2)/2+foot+margin;
    rectangle(ax,'Position',[c(1)-inflX,c(2)-inflY,2*inflX,2*inflY], ...
        'EdgeColor',col,'LineWidth',1,'LineStyle',':');
    text(ax,c(1),c(2)+d(2)/2+0.15,o.name,'Color',col, ...
        'HorizontalAlignment','center','FontSize',8);
end
h0=plot(ax,route0(1:n0,1),route0(1:n0,2),'b-','LineWidth',1.8);
h1=plot(ax,route1(1:n1,1),route1(1:n1,2),'m-','LineWidth',1.8);
hs=plot(ax,cfg.startPose(1),cfg.startPose(2),'go','MarkerFaceColor','g');
hg=plot(ax,cfg.goalPose(1),cfg.goalPose(2),'ys','MarkerFaceColor','y');
hp=plot(ax,postStart(1),postStart(2),'md','MarkerFaceColor','m');
axis(ax,'equal'); axis(ax,'tight'); grid(ax,'on');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'Geometry-aware route test (live model unchanged)');
legend(ax,[h0,h1,hs,hg,hp], ...
    {'Initial route','Post-spawn route','Configured start','Goal','Logged replan start'}, ...
    'Location','bestoutside');
end
