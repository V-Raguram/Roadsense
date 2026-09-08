function report=runRoadsenseScenarioTuningSuite(options)
%RUNROADSENSESCENARIOTUNINGSUITE Compare profiles on the closed-loop suite.
arguments
    options.Profiles (1,:) string = ["baseline","balanced"]
    options.ScenarioIDs (1,:) double = 1:5
    options.StopTimes (1,:) double = []
    options.OutputDirectory (1,1) string = ""
    options.InferenceMode (1,1) string {mustBeMember(options.InferenceMode, ...
        ["network","syntheticColor"])} = "syntheticColor"
    options.UseFastRestart (1,1) logical = false
    options.GenerateValidationFigures (1,1) logical = false
    options.ShowProgress (1,1) logical = true
end
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
if strlength(options.OutputDirectory)==0
    options.OutputDirectory=fullfile(root,"Results","ScenarioTuning");
end
if ~isfolder(options.OutputDirectory); mkdir(options.OutputDirectory); end
validProfiles=["baseline","balanced","cautious"];
if any(~ismember(options.Profiles,validProfiles))
    error("Roadsense:Tuning:Profile","Profiles must be baseline, balanced, or cautious.");
end
createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
deploymentCleanup=onCleanup(@() activateRoadsenseDeploymentProfile(string(root)));
runs=cell(1,numel(options.Profiles)); scores=cell(1,numel(options.Profiles));
suiteTimer=tic;
for index=1:numel(options.Profiles)
    profile=RoadsenseTuningProfile(options.Profiles(index));
    snapshot=applyRoadsenseTuningProfile(profile);
    rollback=onCleanup(@() restoreRoadsenseTuningSnapshot(snapshot));
    profileDirectory=fullfile(options.OutputDirectory,profile.Name);
    runs{index}=runRoadsenseValidationSuite(ScenarioIDs=options.ScenarioIDs, ...
        StopTimes=options.StopTimes,OutputDirectory=profileDirectory, ...
        UseFastRestart=options.UseFastRestart, ...
        GenerateFigures=options.GenerateValidationFigures,SaveTimelines=true, ...
        ContinueOnError=true,ShowProgress=options.ShowProgress, ...
        RefreshDataDictionary=false,InferenceMode=options.InferenceMode);
    scores{index}=scoreRoadsenseTuningMetrics(runs{index}.Summary,profile.Name);
    clear rollback;
end
comparison=vertcat(scores{:}); [~,winnerIndex]=min(comparison.Objective);
recommendedProfile=comparison.Profile(winnerIndex);
suiteWallTime=toc(suiteTimer);
writetable(comparison,fullfile(options.OutputDirectory,"tuning_comparison.csv"));
figurePath=generateRoadsenseTuningFigure(comparison,options.OutputDirectory);
reportPath=writeRoadsenseTuningReport(comparison,recommendedProfile,options,suiteWallTime);
report=struct("Comparison",comparison,"Runs",{runs}, ...
    "RecommendedProfile",recommendedProfile,"SuiteWallTime",suiteWallTime, ...
    "ReportPath",reportPath,"FigurePath",figurePath, ...
    "OutputDirectory",options.OutputDirectory);
save(fullfile(options.OutputDirectory,"tuning_results.mat"),"report","-v7.3");
clear deploymentCleanup; activateRoadsenseDeploymentProfile(string(root));
fprintf("Recommended Roadsense profile: %s\n",recommendedProfile);
end
