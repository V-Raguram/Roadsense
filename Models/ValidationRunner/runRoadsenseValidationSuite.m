function report=runRoadsenseValidationSuite(options)
%RUNROADSENSEVALIDATIONSUITE Execute and report closed-loop scenario tests.
arguments
    options.ScenarioIDs (1,:) double = 1:5
    options.StopTimes (1,:) double = []
    options.OutputDirectory (1,1) string = ""
    options.UseFastRestart (1,1) logical = false
    options.GenerateFigures (1,1) logical = true
    options.SaveTimelines (1,1) logical = true
    options.ContinueOnError (1,1) logical = true
    options.ShowProgress (1,1) logical = true
    options.RefreshDataDictionary (1,1) logical = true
    options.InferenceMode (1,1) string {mustBeMember(options.InferenceMode, ...
        ["network","syntheticColor"])} = "syntheticColor"
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
configuration=RoadsenseValidationConfiguration(string(root));
scenarioIDs=round(options.ScenarioIDs);
if any(scenarioIDs<1 | scenarioIDs>5 | scenarioIDs~=options.ScenarioIDs)
    error("Roadsense:Validation:ScenarioID","ScenarioIDs must contain integers from 1 through 5.");
end
if strlength(options.OutputDirectory)>0
    configuration.OutputDirectory=char(options.OutputDirectory);
end
configuration.ScenarioIDs=scenarioIDs;
configuration.UseFastRestart=options.UseFastRestart;
configuration.InferenceMode=options.InferenceMode;
configuration.GenerateFigures=options.GenerateFigures;
configuration.SaveTimelines=options.SaveTimelines;
configuration.ContinueOnError=options.ContinueOnError;
if ~isfolder(configuration.OutputDirectory); mkdir(configuration.OutputDirectory); end

if options.RefreshDataDictionary
    createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
end
modelPath=fullfile(root,"Models","ScenarioClosedLoopHarness", ...
    "Roadsense_ScenarioClosedLoopHarness.slx");
if ~isfile(modelPath); createRoadsenseScenarioClosedLoopHarnessModel(); end
modelName="Roadsense_ScenarioClosedLoopHarness"; load_system(modelPath);
modelCleanup=onCleanup(@() closeIfLoaded(modelName));
% The built-in scenarios render a deterministic colour-coded camera stream.
% Select its matching backend only in memory after loading the hierarchy;
% this avoids rewriting the generated perception SLX for each qualification.
semanticWasLoaded=bdIsLoaded("Roadsense_SemanticPerception");
previousInferenceMode=setRoadsenseSemanticInferenceMode( ...
    options.InferenceMode,Persist=false);
inferenceCleanup=onCleanup(@() restoreInferenceMode( ...
    previousInferenceMode,semanticWasLoaded));

stopTimes=zeros(size(scenarioIDs));
for index=1:numel(scenarioIDs)
    [~,~,status]=RoadsenseScenarioCatalog(scenarioIDs(index),0);
    stopTimes(index)=double(status.Duration)+configuration.CompletionBuffer;
end
if ~isempty(options.StopTimes)
    if isscalar(options.StopTimes); stopTimes(:)=options.StopTimes;
    elseif numel(options.StopTimes)==numel(scenarioIDs); stopTimes=options.StopTimes;
    else; error("Roadsense:Validation:StopTimes", ...
            "StopTimes must be scalar or match the number of scenarios.");
    end
end
if any(stopTimes<=0); error("Roadsense:Validation:StopTimes","Every stop time must be positive."); end

simulationInputs=repmat(Simulink.SimulationInput(modelName),1,numel(scenarioIDs));
for index=1:numel(scenarioIDs)
    dataset=createRoadsenseScenarioHarnessInputs(scenarioIDs(index),stopTimes(index));
    input=Simulink.SimulationInput(modelName); input=input.setExternalInput(dataset);
    input=input.setModelParameter("StopTime",string(stopTimes(index)), ...
        "SaveOutput","off","SignalLogging","on","SignalLoggingName","logsout", ...
        "ReturnWorkspaceOutputs","on","CaptureErrors","on");
    simulationInputs(index)=input;
end

fprintf("Roadsense validation: %d scenario(s), contract v%d.\n", ...
    numel(scenarioIDs),configuration.ContractVersion);
suiteTimer=tic;
useFastRestart="off"; if options.UseFastRestart; useFastRestart="on"; end
showProgress="off"; if options.ShowProgress; showProgress="on"; end
simulationOutputs=sim(simulationInputs,"UseFastRestart",useFastRestart, ...
    "ShowProgress",showProgress);
suiteWallTime=toc(suiteTimer);

results=cell(1,numel(scenarioIDs)); rows=cell(1,numel(scenarioIDs));
for index=1:numel(scenarioIDs)
    errorMessage=string(simulationOutputs(index).ErrorMessage);
    if strlength(errorMessage)>0
        results{index}=failedResult(scenarioIDs(index),stopTimes(index),errorMessage);
        rows{index}=results{index}.Metrics;
        if ~options.ContinueOnError; error("Roadsense:Validation:Simulation",errorMessage); end
        continue
    end
    wallTime=simulationWallTime(simulationOutputs(index));
    try
        results{index}=extractRoadsenseValidationResult( ...
            simulationOutputs(index),scenarioIDs(index),wallTime,stopTimes(index));
        rows{index}=results{index}.Metrics;
        if configuration.SaveTimelines
            result=results{index};
            save(fullfile(configuration.OutputDirectory,sprintf( ...
                "scenario_%d_result.mat",scenarioIDs(index))),"result","-v7.3");
        end
    catch exception
        results{index}=failedResult(scenarioIDs(index),stopTimes(index),string(exception.message));
        rows{index}=results{index}.Metrics;
        if ~options.ContinueOnError; rethrow(exception); end
    end
end
summary=vertcat(rows{:});
writetable(summary,fullfile(configuration.OutputDirectory,"validation_summary.csv"));
save(fullfile(configuration.OutputDirectory,"validation_results.mat"), ...
    "summary","results","configuration","suiteWallTime","-v7.3");
if configuration.GenerateFigures
    generateRoadsenseValidationFigures(results,summary,configuration.OutputDirectory);
end
reportPath=writeRoadsenseValidationReport(summary,configuration,suiteWallTime);
report=struct("Summary",summary,"ScenarioResults",{results}, ...
    "Configuration",configuration,"SuiteWallTime",suiteWallTime, ...
    "ReportPath",reportPath,"OutputDirectory",string(configuration.OutputDirectory));
close_system(modelName,0); clear modelCleanup;
clear inferenceCleanup;
fprintf("Validation artifacts: %s\n",configuration.OutputDirectory);
fprintf("Scenario completion rate: %.1f%% | acceptance rate: %.1f%%\n", ...
    100*mean(summary.Completed),100*mean(summary.Pass));
end

function wallSeconds=simulationWallTime(output)
try
    value=output.SimulationMetadata.TimingInfo.TotalElapsedWallTime;
    if isduration(value); wallSeconds=seconds(value); else; wallSeconds=double(value); end
catch
    wallSeconds=NaN;
end
end

function result=failedResult(scenarioID,stopTime,message)
names=RoadsenseValidationConfiguration().ScenarioNames;
metrics=table(uint16(scenarioID),names(scenarioID),stopTime,false,false,false, ...
    false,single(NaN),uint32(0),single(NaN),single(NaN),single(NaN), ...
    single(NaN),uint32(0),single(0),uint8(0),false,false,NaN,false,"simulation failed",string(message), ...
    'VariableNames',metricVariableNames());
result=struct("ScenarioID",uint16(scenarioID),"ScenarioName",names(scenarioID), ...
    "StopTime",stopTime,"WallTime",NaN,"Metrics",metrics,"Timeline",struct, ...
    "Route",zeros(0,2),"ErrorMessage",string(message));
end

function names=metricVariableNames()
names=["ScenarioID","ScenarioName","RequestedStopTime","SimulationSucceeded", ...
    "Completed","CollisionFree","GoalReached","MinimumClearanceM","ReplanCount", ...
    "MaximumReplanLatencyMs","PathSmoothnessInvM","MaximumJerkMps3", ...
    "MaximumCrossTrackErrorM","SafetyInterventions","PipelineReadyPercent", ...
    "DropoutObservedMask","Pass","Valid","WallTimeSeconds", ...
    "FinalPipelineReady","ReadinessBlockers","ErrorMessage"];
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName); close_system(modelName,0); end
end

function restoreInferenceMode(mode,wasLoaded)
setRoadsenseSemanticInferenceMode(mode,Persist=false);
if ~wasLoaded && bdIsLoaded("Roadsense_SemanticPerception")
    close_system("Roadsense_SemanticPerception",0);
end
end
