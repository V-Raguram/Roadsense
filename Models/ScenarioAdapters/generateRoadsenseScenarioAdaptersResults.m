function metrics=generateRoadsenseScenarioAdaptersResults()
%GENERATEROADSENSESCENARIOADAPTERSRESULTS Save deterministic adapter evidence.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
resultDir=fullfile(root,"Results","ScenarioAdapters");
if ~isfolder(resultDir); mkdir(resultDir); end
result=runRoadsenseScenarioAdaptersDemo(true);
exportgraphics(gcf,fullfile(resultDir,"scenario_adapter_route.png"),"Resolution",180);
writetable(result.Timeline,fullfile(resultDir,"scenario_adapter_timeline.csv"));
writetable(result.Summary,fullfile(resultDir,"scenario_adapter_sensor_summary.csv"));
metrics=table(height(result.Timeline),result.Timeline.DistanceToGoal(1), ...
    result.Timeline.DistanceToGoal(end),sum(result.Timeline.Reset), ...
    any(result.Timeline.GoalReached),all(result.Summary{1,4:6}), ...
    'VariableNames',{'RouteSamples','InitialDistance','FinalDistance', ...
    'ResetPulseCount','GoalReached','AllSensorsValid'});
writetable(metrics,fullfile(resultDir,"scenario_adapter_metrics.csv"));
save(fullfile(resultDir,"scenario_adapter_results.mat"),"result","metrics");
fprintf("Wrote scenario-adapter evidence to %s\n",resultDir);
end
