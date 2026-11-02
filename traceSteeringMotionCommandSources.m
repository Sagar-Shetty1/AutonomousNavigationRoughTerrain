%% TRACE STEERING MOTION COMMAND SOURCES (READ ONLY)
% Traces the Simulink signal paths feeding the Simulink-PS Converters
% connected to the front-left and front-right steering Revolute Joints.
% Does not alter block parameters, wiring, model state, or files.

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
converterNames = {'Simulink-PS Converter5','Simulink-PS Converter4'};

fprintf('\n============================================================\n');
fprintf(' FRONT STEERING MOTION COMMAND SOURCE TRACE (READ ONLY)\n');
fprintf('============================================================\n');

if ~bdIsLoaded(modelName)
    error('Diagnostic model is not loaded: %s', modelName);
end
fprintf('Model: %s\n', modelName);
fprintf('Dirty flag at start: %s\n', get_param(modelName,'Dirty'));

for k = 1:numel(converterNames)
    converterPath = [modelName '/' converterNames{k}];
    if getSimulinkBlockHandle(converterPath) == -1
        fprintf('\n[SKIP] Converter not found: %s\n', converterPath);
        continue;
    end

    fprintf('\n------------------------------------------------------------\n');
    fprintf('CONVERTER: %s\n', converterNames{k});
    fprintf('------------------------------------------------------------\n');
    printBlockInfo(converterPath);

    try
        dp = get_param(converterPath,'DialogParameters');
        keys = fieldnames(dp);
        fprintf('Relevant converter dialog parameters:\n');
        matched = false;
        for j = 1:numel(keys)
            key = keys{j};
            if ~isempty(regexpi(key,'unit|filter|derivative|input|extrap|interpol|sample|time'))
                try
                    val = get_param(converterPath,key);
                    fprintf('  %-38s = %s\n', key, valueToText(val));
                    matched = true;
                catch
                end
            end
        end
        if ~matched
            fprintf('  (No matching dialog parameters exposed.)\n');
        end
    catch ME
        fprintf('Could not inspect converter dialog parameters: %s\n', ME.message);
    end

    ph = get_param(converterPath,'PortHandles');
    if isempty(ph.Inport)
        fprintf('Converter has no standard Simulink Inport; cannot trace signal input.\n');
        continue;
    end
    for p = 1:numel(ph.Inport)
        fprintf('\nConverter Simulink input %d:\n', p);
        lineH = get_param(ph.Inport(p),'Line');
        if isempty(lineH) || lineH == -1
            fprintf('  UNCONNECTED\n');
            continue;
        end
        srcBlockH = get_param(lineH,'SrcBlockHandle');
        srcPortH = get_param(lineH,'SrcPortHandle');
        if isempty(srcBlockH) || srcBlockH == -1
            fprintf('  No source block found on input line.\n');
            continue;
        end
        visited = {};
        traceSource(srcBlockH, srcPortH, 0, 9, visited);
    end
end

fprintf('\n============================================================\n');
fprintf('TRACE COMPLETE — NO MODEL PARAMETERS OR CONNECTIONS CHANGED\n');
fprintf('Dirty flag at end: %s\n', get_param(modelName,'Dirty'));
fprintf('============================================================\n\n');

%% Local functions
function traceSource(blockH, outPortH, depth, maxDepth, visited)
    indent = repmat('  ',1,depth+1);
    if depth > maxDepth
        fprintf('%s[Maximum trace depth reached]\n', indent);
        return;
    end
    if isempty(blockH) || blockH == -1
        fprintf('%s[No source block]\n',indent);
        return;
    end
    try
        bpath = getfullname(blockH);
    catch
        bpath = '<unresolved block>';
    end
    key = bpath;
    if ~isempty(visited) && any(strcmp(visited,key))
        fprintf('%s%s [already visited]\n',indent,bpath);
        return;
    end
    visited{end+1} = key; %#ok<AGROW>

    try
        btype = get_param(blockH,'BlockType');
    catch
        btype = '<unknown>';
    end
    fprintf('%s-> %s  [BlockType=%s]\n',indent,bpath,btype);
    printBlockInfoIndented(blockH, indent);

    % Determine which output port of this block feeds the downstream line.
    try
        ph = get_param(blockH,'PortHandles');
        outIdx = find(ph.Outport == outPortH,1,'first');
    catch
        ph = struct(); outIdx = [];
    end

    % Cross a subsystem output boundary and inspect its matching Outport.
    if strcmp(btype,'SubSystem')
        if isempty(outIdx)
            fprintf('%s  Could not identify the subsystem output port index.\n',indent);
            return;
        end
        try
            childOuts = find_system(bpath,'SearchDepth',1,'BlockType','Outport', ...
                'LookUnderMasks','all','FollowLinks','on');
            target = '';
            for q = 1:numel(childOuts)
                childPort = str2double(get_param(childOuts{q},'Port'));
                if childPort == outIdx
                    target = childOuts{q};
                    break;
                end
            end
            if isempty(target)
                fprintf('%s  No internal Outport mapped to output %d.\n',indent,outIdx);
                return;
            end
            fprintf('%s  Subsystem output %d maps to: %s\n',indent,outIdx,target);
            tph = get_param(target,'PortHandles');
            if isempty(tph.Inport)
                fprintf('%s  Internal Outport has no input port.\n',indent);
                return;
            end
            followInput(tph.Inport(1),depth+1,maxDepth,visited,indent);
        catch ME
            fprintf('%s  Failed crossing subsystem boundary: %s\n',indent,ME.message);
        end
        return;
    end

    % Cross an Inport block boundary to the parent subsystem's corresponding input.
    if strcmp(btype,'Inport')
        try
            portNo = str2double(get_param(blockH,'Port'));
            parentPath = get_param(blockH,'Parent');
            if strcmp(parentPath,bdroot(blockH))
                fprintf('%s  Root-level Inport %d (external model input).\n',indent,portNo);
                return;
            end
            parentPH = get_param(parentPath,'PortHandles');
            if portNo <= numel(parentPH.Inport)
                fprintf('%s  Crossing to parent subsystem input %d: %s\n',indent,portNo,parentPath);
                followInput(parentPH.Inport(portNo),depth+1,maxDepth,visited,indent);
            else
                fprintf('%s  Parent subsystem input %d not found.\n',indent,portNo);
            end
        catch ME
            fprintf('%s  Failed crossing Inport boundary: %s\n',indent,ME.message);
        end
        return;
    end

    % Trace all Simulink inputs of ordinary signal-processing / routing blocks.
    try
        inHandles = ph.Inport;
    catch
        inHandles = [];
    end
    if isempty(inHandles)
        fprintf('%s  [Source / block has no standard Simulink inputs]\n',indent);
        return;
    end
    for q = 1:numel(inHandles)
        fprintf('%s  Input %d source:\n',indent,q);
        followInput(inHandles(q),depth+1,maxDepth,visited,indent);
    end
end

function followInput(inPortH,depth,maxDepth,visited,parentIndent)
    try
        lineH = get_param(inPortH,'Line');
    catch
        lineH = -1;
    end
    if isempty(lineH) || lineH == -1
        fprintf('%s    [Unconnected input]\n',parentIndent);
        return;
    end
    try
        srcBlockH = get_param(lineH,'SrcBlockHandle');
        srcPortH = get_param(lineH,'SrcPortHandle');
        traceSource(srcBlockH,srcPortH,depth,maxDepth,visited);
    catch ME
        fprintf('%s    [Could not follow input: %s]\n',parentIndent,ME.message);
    end
end

function printBlockInfo(blockPath)
    try
        h = getSimulinkBlockHandle(blockPath);
        printBlockInfoIndented(h,'');
    catch
    end
end

function printBlockInfoIndented(blockH,indent)
    fields = {'MaskType','ReferenceBlock','Operator','Gain','Value','OutDataTypeStr', ...
        'SaturationUpperLimit','SaturationLowerLimit','UpperLimit','LowerLimit', ...
        'GotoTag','Tag','FunctionName','VariableName','SampleTime'};
    for z = 1:numel(fields)
        try
            v = get_param(blockH,fields{z});
            if ~isempty(v)
                fprintf('%s     %-22s = %s\n',indent,fields{z},valueToText(v));
            end
        catch
        end
    end
end

function s = valueToText(v)
    if ischar(v)
        s = v;
    elseif isstring(v)
        s = char(join(v,', '));
    elseif isnumeric(v) || islogical(v)
        s = mat2str(v);
    else
        try
            s = evalc('disp(v)');
            s = strtrim(strrep(s,sprintf('\n'),' '));
        catch
            s = '<value not printable>';
        end
    end
end
