function inspectSteeringJointSignalPath
% INSPECTSTEERINGJOINTSIGNALPATH
% Read-only inspection of the two front steering joint actuation paths.
% Does not change, simulate, or save any model.

clc;
modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
modelFile = fullfile(pwd, [modelName '.slx']);

fprintf('\n============================================================\n');
fprintf(' FRONT STEERING JOINT / ACTUATION PATH INSPECTION (READ ONLY)\n');
fprintf('============================================================\n');

if ~bdIsLoaded(modelName)
    assert(exist(modelFile,'file') == 2, ...
        'Diagnostic model file not found in the current directory: %s', modelFile);
    load_system(modelFile);
end

dirtyState = get_param(modelName,'Dirty');
fprintf('Diagnostic model: %s\n', modelName);
fprintf('Diagnostic model dirty flag: %s\n', dirtyState);
if strcmpi(dirtyState,'on')
    error(['The diagnostic model has unsaved in-memory changes. ' ...
           'Reload the clean diagnostic copy (close_system(modelName,0); load_system(modelFile)) ' ...
           'before running this read-only inspection.']);
end

targetNames = {'Steering Revolute FL','Steering Revolute FR'};
for k = 1:numel(targetNames)
    fprintf('\n------------------------------------------------------------\n');
    fprintf('JOINT: %s\n', targetNames{k});
    fprintf('------------------------------------------------------------\n');

    found = find_system(modelName, ...
        'LookUnderMasks','all', 'FollowLinks','on', ...
        'SearchDepth', Inf, 'Name', targetNames{k});

    if isempty(found)
        fprintf('No block found with this exact name.\n');
        continue;
    end
    for b = 1:numel(found)
        blk = found{b};
        fprintf('\nBlock path : %s\n', blk);
        safePrintParam(blk,'BlockType');
        safePrintParam(blk,'MaskType');
        safePrintParam(blk,'ReferenceBlock');
        safePrintParam(blk,'LinkStatus');

        % Print relevant dialog and mask parameters without assuming
        % a particular Simscape release's complete parameter schema.
        fprintf('\nRelevant joint parameters:\n');
        names = {};
        try
            dp = get_param(blk,'DialogParameters');
            if isstruct(dp), names = [names, fieldnames(dp)']; end %#ok<AGROW>
        catch
        end
        try
            mn = get_param(blk,'MaskNames');
            if iscell(mn), names = [names, mn(:)']; end %#ok<AGROW>
        catch
        end
        names = unique(names,'stable');
        keys = {'actuation','motion','torque','position','angle','specify','unit','input'};
        printed = false;
        for n = 1:numel(names)
            pn = names{n};
            if any(contains(lower(pn),keys))
                try
                    pv = get_param(blk,pn);
                    fprintf('  %-38s = %s\n', pn, valueToText(pv));
                    printed = true;
                catch
                end
            end
        end
        if ~printed
            fprintf('  (No matching dialog/mask parameter names were exposed.)\n');
        end

        fprintf('\nPort handles and directly connected blocks:\n');
        try
            ph = get_param(blk,'PortHandles');
            fns = fieldnames(ph);
            for fi = 1:numel(fns)
                field = fns{fi};
                handles = ph.(field);
                if isempty(handles), continue; end
                handles = handles(:);
                for pi = 1:numel(handles)
                    h = handles(pi);
                    fprintf('  %s(%d), handle=%g', field, pi, h);
                    try
                        lineHandle = get_param(h,'Line');
                        if isnumeric(lineHandle) && isscalar(lineHandle) && lineHandle ~= -1
                            fprintf(', line=%g', lineHandle);
                            try
                                srcPort = get_param(lineHandle,'SrcPortHandle');
                                dstPorts = get_param(lineHandle,'DstPortHandle');
                                if ~isempty(srcPort) && srcPort ~= -1
                                    fprintf(', source=%s', portDescription(srcPort));
                                end
                                if ~isempty(dstPorts)
                                    for di = 1:numel(dstPorts)
                                        if dstPorts(di) ~= -1
                                            fprintf(', destination=%s', portDescription(dstPorts(di)));
                                        end
                                    end
                                end
                            catch MEline
                                fprintf(', line details unavailable (%s)', MEline.message);
                            end
                        else
                            fprintf(', unconnected/no Simulink line');
                        end
                    catch MEport
                        fprintf(', line query unavailable (%s)', MEport.message);
                    end
                    fprintf('\n');
                end
            end
        catch MEph
            fprintf('  Could not inspect PortHandles: %s\n', MEph.message);
        end

        fprintf('\nPortConnectivity detail:\n');
        try
            pc = get_param(blk,'PortConnectivity');
            for pi = 1:numel(pc)
                fprintf('  Port %d: Type=%s, Position=%s\n', ...
                    pi, safeFieldText(pc(pi),'Type'), safeFieldText(pc(pi),'Position'));
                printBlockHandleList('    SrcBlock', safeField(pc(pi),'SrcBlock'));
                printBlockHandleList('    DstBlock', safeField(pc(pi),'DstBlock'));
            end
        catch MEpc
            fprintf('  Could not inspect PortConnectivity: %s\n', MEpc.message);
        end
    end
end

fprintf('\n============================================================\n');
fprintf('INSPECTION COMPLETE — NO MODEL PARAMETERS OR CONNECTIONS CHANGED\n');
fprintf('Original Custom model was not accessed or modified by this script.\n');
fprintf('============================================================\n');

end

function safePrintParam(blk,paramName)
try
    val = get_param(blk,paramName);
    fprintf('%-13s: %s\n', [paramName ' '], valueToText(val));
catch
end
end

function s = valueToText(v)
if ischar(v)
    s = v;
elseif isstring(v)
    s = char(join(v(:),', '));
elseif isnumeric(v) || islogical(v)
    s = mat2str(v);
elseif iscell(v)
    try
        s = strjoin(cellfun(@valueToText,v(:),'UniformOutput',false),', ');
    catch
        s = '<cell value>';
    end
else
    try
        s = char(string(v));
    catch
        s = ['<' class(v) '>'];
    end
end
end

function s = portDescription(portHandle)
try
    parent = get_param(portHandle,'Parent');
    portNum = get_param(portHandle,'PortNumber');
    portType = get_param(portHandle,'PortType');
    s = sprintf('%s [%s %s]', parent, portType, valueToText(portNum));
catch
    s = '<unknown port>';
end
end

function v = safeField(st,field)
v = [];
try
    v = st.(field);
catch
end
end

function s = safeFieldText(st,field)
v = safeField(st,field);
s = valueToText(v);
end

function printBlockHandleList(label,handles)
if isempty(handles), return; end
handles = handles(:);
for i = 1:numel(handles)
    h = handles(i);
    if ~isnumeric(h) || h < 0, continue; end
    try
        fprintf('%s: %s\n',label,getfullname(h));
    catch
        fprintf('%s: handle %g\n',label,h);
    end
end
end
