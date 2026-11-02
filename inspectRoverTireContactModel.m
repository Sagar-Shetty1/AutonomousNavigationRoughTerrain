
%% READ-ONLY INSPECTION OF ROVER TIRE CONTACT MODEL

modelName = ...
    'OffRoadMultibodyStage1_PhysicalScenarioSpawnTest_Custom';

fprintf('\n============================================\n');
fprintf(' ROVER TIRE CONTACT DIAGNOSTICS\n');
fprintf('============================================\n');

%% 1. Inspect Simscape logging configuration

fprintf('\n--- SIMSCAPE LOGGING CONFIGURATION ---\n');

parameters = {
    'SimscapeLogType'
    'SimscapeLogName'
    'SimscapeLogLimitData'
    'SimscapeLogDataHistory'
};

for k = 1:numel(parameters)
    try
        fprintf('%s = %s\n',parameters{k}, ...
            get_param(modelName,parameters{k}));
    catch
        fprintf('%s = unavailable\n',parameters{k});
    end
end

%% 2. Inspect saved failure log tree

fprintf('\n--- FAILURE LOG TREE ---\n');

assert(exist('simOutCustomFailureDiag40s','var') == 1, ...
    'Failure simulation output not found in workspace.');

failureLog = ...
    simOutCustomFailureDiag40s.get( ...
    'simlog_CustomFailureDiag40s');

treeText = evalc('print(failureLog)');

logFile = fullfile(pwd,'customFailureSimscapeLogTree.txt');

fid = fopen(logFile,'w');

assert(fid ~= -1,'Could not create logging tree report.');

fprintf(fid,'%s',treeText);
fclose(fid);

fprintf('Complete logging tree saved to:\n%s\n',logFile);

%% 3. Check whether tire subsystem nodes exist

fprintf('\n--- TIRE SUBSYSTEM LOGGING STATUS ---\n');

tireNames = {
    'RoverTireContact_FL'
    'RoverTireContact_FR'
    'RoverTireContact_RL'
    'RoverTireContact_RR'
};

for k = 1:numel(tireNames)

    blockPath = [modelName '/' tireNames{k}];

    try
        [isValid,nodePath] = ...
            simscape.logging.findPath(failureLog,blockPath);

        fprintf('%s: ',tireNames{k});

        if isValid
            fprintf('LOGGED - %s\n',nodePath);
        else
            fprintf('NOT LOGGED\n');
        end
    catch ME
        fprintf('%s: inspection error - %s\n', ...
            tireNames{k},ME.message);
    end
end

%% 4. Inspect MATLAB Function code in the tire subsystems

fprintf('\n--- TIRE MATLAB FUNCTION SCRIPTS ---\n');

root = sfroot;
charts = find(root,'-isa','Stateflow.EMChart');

scriptFile = fullfile(pwd,'customTireContactSource.txt');

fid = fopen(scriptFile,'w');

assert(fid ~= -1,'Could not create tire source report.');

for k = 1:numel(charts)

    chartPath = char(charts(k).Path);

    if contains(chartPath,'RoverTireContact_')

        fprintf(fid,'\n============================================\n');
        fprintf(fid,'CHART: %s\n',chartPath);
        fprintf(fid,'============================================\n');

        fprintf(fid,'%s\n',char(charts(k).Script));
    end
end

fclose(fid);

fprintf('Tire function source saved to:\n%s\n',scriptFile);

%% 5. Inspect physical tire-contact block parameters

fprintf('\n--- TIRE CONTACT BLOCK PARAMETERS ---\n');

for k = 1:numel(tireNames)

    blockPath = [modelName '/' tireNames{k} ...
        '/Custom Tire Force and Torque'];

    fprintf('\n%s\n',tireNames{k});

    try
        params = get_param(blockPath,'DialogParameters');
        fields = fieldnames(params);

        for j = 1:numel(fields)

            p = fields{j};

            if ~isempty(regexp(p, ...
                'Tire|Radius|Stiff|Damp|Fric|Force|Contact|Deflection|Velocity|Load', ...
                'once','ignorecase'))

                try
                    fprintf('%s = %s\n',p, ...
                        strtrim(evalc( ...
                        'disp(get_param(blockPath,p))')));
                catch
                end
            end
        end
    catch ME
        fprintf('Inspection unavailable: %s\n',ME.message);
    end
end

fprintf('\n============================================\n');
fprintf(' TIRE CONTACT INSPECTION COMPLETE\n');
fprintf(' No model changes made.\n');
fprintf(' No simulation started.\n');
fprintf('============================================\n');
