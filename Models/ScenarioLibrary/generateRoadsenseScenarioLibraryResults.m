function metrics=generateRoadsenseScenarioLibraryResults()
%GENERATEROADSENSESCENARIOLIBRARYRESULTS Save repeatable scenario evidence.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
resultDir=fullfile(root,"Results","ScenarioLibrary");
if ~isfolder(resultDir); mkdir(resultDir); end
drivingManifest=createRoadsenseDrivingScenarios();
roadRunnerManifest=createRoadsenseRoadRunnerHDMaps();
result=runRoadsenseScenarioLibraryDemo(false);
exportgraphics(result.RouteFigure,fullfile(resultDir,"five_scenario_overview.png"),"Resolution",180);
exportgraphics(result.TimelineFigure,fullfile(resultDir,"scenario_event_timeline.png"),"Resolution",180);
close(result.RouteFigure); close(result.TimelineFigure);
modelPath=createRoadsenseScenarioLibraryModel(); [~,modelName]=fileparts(modelPath);
load_system(modelPath); set_param(modelName,"ZoomFactor","FitSystem");
print("-s"+modelName,"-dpng","-r180",fullfile(resultDir,"model_layout.png"));
close_system(modelName,0);
writetable(result.Summary,fullfile(resultDir,"scenario_library_metrics.csv"));
writetable(result.Timeline,fullfile(resultDir,"scenario_library_timeline.csv"));
writetable(drivingManifest,fullfile(resultDir,"driving_scenario_manifest.csv"));
writetable(roadRunnerManifest,fullfile(resultDir,"roadrunner_map_manifest.csv"));
metrics=result.Summary;
result=rmfield(result,{'RouteFigure','TimelineFigure'}); %#ok<NASGU>
save(fullfile(resultDir,"scenario_library_results.mat"),"result","metrics", ...
    "drivingManifest","roadRunnerManifest");
fprintf("Wrote scenario-library evidence to %s\n",resultDir);
end
