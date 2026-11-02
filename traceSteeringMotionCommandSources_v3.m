%% TRACE STEERING MOTION COMMAND SOURCES — V3 (READ ONLY)
% Corrects find_system criteria ordering; safely searches masked subsystem contents.
% No model parameters, connections, or files are modified.

modelName = 'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom_UpperStopProbe_20261031_160339';
converterNames = {'Simulink-PS Converter5','Simulink-PS Converter4'};

fprintf('\n============================================================\n');
fprintf(' FRONT STEERING MOTION COMMAND SOURCE TRACE V3 (READ ONLY)\n');
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
    printSelectedParams(converterPath, {'Unit','FilteringAndDerivatives','SimscapeFilterOrder','InputFilterTimeConstant'});

    cph = get_param(converterPath,'PortHandles');
    for p = 1:numel(cph.Inport)
        fprintf('\nConverter Simulink input %d:\n',p);
        lineH = get_param(cph.Inport(p),'Line');
        if isempty(lineH) || lineH == -1
            fprintf('  UNCONNECTED\n');
            continue;
        end
        srcBlockH = get_param(lineH,'SrcBlockHandle');
        srcPortH = get_param(lineH,'SrcPortHandle');
        if isempty(srcBlockH) || srcBlockH == -1
            fprintf('  No source block found.\n');
            continue;
        end
        visited = {};
        traceSource(srcBlockH,srcPortH,0,12,visited);
    end
end

fprintf('\n============================================================\n');
fprintf('TRACE COMPLETE — NO MODEL PARAMETERS OR CONNECTIONS CHANGED\n');
fprintf('Dirty flag at end: %s\n',get_param(modelName,'Dirty'));
fprintf('============================================================\n\n');

%% Local functions
function traceSource(blockH,outPortH,depth,maxDepth,visited)
    indent = repmat('  ',1,depth+1);
    if depth > maxDepth
        fprintf('%s[MAXIMUM TRACE DEPTH REACHED]\n',indent); return;
    end
    if isempty(blockH) || blockH == -1
        fprintf('%s[NO SOURCE BLOCK]\n',indent); return;
    end
    try, bpath = getfullname(blockH); catch, bpath = '<unresolved block>'; end
    try, btype = get_param(blockH,'BlockType'); catch, btype = '<unknown>'; end
    try
        ph = get_param(blockH,'PortHandles');
        outIdx = find(ph.Outport == outPortH,1,'first');
    catch
        ph = struct(); outIdx = [];
    end
    visitKey = sprintf('%s|out=%s',bpath,mat2str(outIdx));
    if any(strcmp(visited,visitKey))
        fprintf('%s-> %s [already visited]\n',indent,bpath); return;
    end
    visited{end+1} = visitKey; %#ok<AGROW>

    fprintf('%s-> %s  [BlockType=%s',indent,bpath,btype);
    if ~isempty(outIdx), fprintf(', output port=%d',outIdx); end
    fprintf(']\n');
    printSelectedParams(blockH, {'MaskType','ReferenceBlock','Operator','Gain','Value', ...
        'SaturationUpperLimit','SaturationLowerLimit','UpperLimit','LowerLimit', ...
        'GotoTag','Tag','FunctionName','VariableName','SampleTime','Inputs','DataPortOrder'});

    % Traverse a subsystem output to the corresponding internal Outport.
    if strcmp(btype,'SubSystem')
        if isempty(outIdx)
            fprintf('%s  Could not resolve which subsystem output feeds the converter.\n',indent);
            return;
        end
        % Report subsystem masking/library metadata before looking inside it.
        printSelectedParams(blockH, {'Mask','MaskType','LinkStatus','ReferenceBlock'});
        try
            % In find_system, search constraints must come BEFORE block parameter
            % filters. Search all descendants under masks, then retain only Outport
            % blocks whose direct Parent is this AckermannSteering subsystem.
            % Deliberately do not search inside library links: that can refresh a
            % linked library implementation during inspection.
            allOuts = find_system(bpath,'SearchUnderMasks','all','BlockType','Outport');
            childOuts = {};
            for q = 1:numel(allOuts)
                try
                    if strcmp(get_param(allOuts{q},'Parent'),bpath)
                        childOuts{end+1} = allOuts{q}; %#ok<AGROW>
                    end
                catch
                end
            end
        catch ME
            fprintf('%s  find_system failed: %s\n',indent,ME.message); return;
        end
        fprintf('%s  Direct internal Outport blocks found: %d\n',indent,numel(childOuts));
        target = '';
        for q = 1:numel(childOuts)
            try
                portText = get_param(childOuts{q},'Port');
                portNo = str2double(portText);
                fprintf('%s    Outport block: %s | Port=%s\n',indent,childOuts{q},portText);
                if portNo == outIdx, target = childOuts{q}; end
            catch
                fprintf('%s    Outport block: %s | could not read Port\n',indent,childOuts{q});
            end
        end
        if isempty(target)
            fprintf('%s  NO INTERNAL OUTPORT MATCHED SUBSYSTEM OUTPUT %d.\n',indent,outIdx);
            fprintf('%s  Direct children (diagnostic listing):\n',indent);
            try
                allKids = find_system(bpath,'SearchUnderMasks','all');
                directKids = {};
                for q = 1:numel(allKids)
                    try
                        if ~strcmp(allKids{q},bpath) && strcmp(get_param(allKids{q},'Parent'),bpath)
                            directKids{end+1} = allKids{q}; %#ok<AGROW>
                        end
                    catch
                    end
                end
                for q = 1:numel(directKids)
                    try, bt = get_param(directKids{q},'BlockType'); catch, bt='?'; end
                    try, mt = get_param(directKids{q},'MaskType'); catch, mt=''; end
                    fprintf('%s    %s [%s] %s\n',indent,directKids{q},bt,mt);
                end
                fprintf('%s  Direct child blocks listed: %d\n',indent,numel(directKids));
            catch ME
                fprintf('%s  Child listing failed: %s\n',indent,ME.message);
            end
            try
                if strcmp(get_param(blockH,'LinkStatus'),'resolved')
                    fprintf('%s  NOTE: subsystem is a resolved library link; internal library blocks were not traversed.\n',indent);
                end
            catch
            end
            return;
        end
        fprintf('%s  Subsystem output %d maps to internal block: %s\n',indent,outIdx,target);
        tph = get_param(target,'PortHandles');
        if isempty(tph.Inport)
            fprintf('%s  Internal Outport has no input port.\n',indent); return;
        end
        followInput(tph.Inport(1),depth+1,maxDepth,visited,indent);
        return;
    end

    % Traverse an internal Inport across its containing subsystem boundary.
    if strcmp(btype,'Inport')
        try
            portNo = str2double(get_param(blockH,'Port'));
            parentPath = get_param(blockH,'Parent');
            rootName = bdroot(blockH);
            if strcmp(parentPath,rootName)
                fprintf('%s  Root-level Inport %d (external model input).\n',indent,portNo); return;
            end
            parentPH = get_param(parentPath,'PortHandles');
            if portNo <= numel(parentPH.Inport)
                fprintf('%s  Crossing subsystem boundary: %s input %d\n',indent,parentPath,portNo);
                followInput(parentPH.Inport(portNo),depth+1,maxDepth,visited,indent);
            else
                fprintf('%s  Parent subsystem input %d not found.\n',indent,portNo);
            end
        catch ME
            fprintf('%s  Failed crossing Inport boundary: %s\n',indent,ME.message);
        end
        return;
    end

    % For ordinary blocks, trace each connected Simulink input.
    try, inHandles = ph.Inport; catch, inHandles = []; end
    if isempty(inHandles)
        fprintf('%s  [No standard Simulink input ports]\n',indent); return;
    end
    for q = 1:numel(inHandles)
        fprintf('%s  Input %d source:\n',indent,q);
        followInput(inHandles(q),depth+1,maxDepth,visited,indent);
    end
end

function followInput(inPortH,depth,maxDepth,visited,parentIndent)
    try, lineH = get_param(inPortH,'Line'); catch, lineH = -1; end
    if isempty(lineH) || lineH == -1
        fprintf('%s    [UNCONNECTED INPUT]\n',parentIndent); return;
    end
    try
        srcBlockH = get_param(lineH,'SrcBlockHandle');
        srcPortH = get_param(lineH,'SrcPortHandle');
        traceSource(srcBlockH,srcPortH,depth,maxDepth,visited);
    catch ME
        fprintf('%s    [Could not follow input: %s]\n',parentIndent,ME.message);
    end
end

function printSelectedParams(blockRef,fields)
    for z = 1:numel(fields)
        try
            v = get_param(blockRef,fields{z});
            if isempty(v), continue; end
            fprintf('     %-26s = %s\n',fields{z},valueToText(v));
        catch
        end
    end
end

function s = valueToText(v)
    if ischar(v), s=v;
    elseif isstring(v), s=char(join(v,', '));
    elseif isnumeric(v) || islogical(v), s=mat2str(v);
    else
        try, s=strtrim(strrep(evalc('disp(v)'),sprintf('\n'),' '));
        catch, s='<unprintable>'; end
    end
end
