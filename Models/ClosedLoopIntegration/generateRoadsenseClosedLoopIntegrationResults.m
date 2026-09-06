function metrics=generateRoadsenseClosedLoopIntegrationResults()
%GENERATEROADSENSECLOSEDLOOPINTEGRATIONRESULTS Save deterministic evidence.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
resultDir=fullfile(root,"Results","ClosedLoopIntegration");
if ~isfolder(resultDir); mkdir(resultDir); end
result=runRoadsenseClosedLoopIntegrationDemo(true);
exportgraphics(gcf,fullfile(resultDir,"integration_health_timeline.png"),"Resolution",180);
writetable(result,fullfile(resultDir,"integration_health_timeline.csv"));
dt=median(diff(result.Time));
metrics=table(height(result),100*mean(result.PipelineReady), ...
    dt*sum(result.SafetyOverride),dt*sum(result.EmergencyStop), ...
    result.CycleID(end),any(result.GoalReached), ...
    'VariableNames',cellstr(["Samples","ReadyPercent","OverrideDurationSeconds", ...
    "EmergencyDurationSeconds","FinalCycleID","GoalReached"]));
writetable(metrics,fullfile(resultDir,"integration_metrics.csv"));
save(fullfile(resultDir,"integration_results.mat"),"result","metrics");
fprintf("Wrote closed-loop integration evidence to %s\n",resultDir);
end
