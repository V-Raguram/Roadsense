function metrics=generateRoadsenseScenarioSensorSourceResults()
%GENERATEROADSENSESCENARIOSENSORSOURCERESULTS Save sensor-source evidence.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
resultDir=fullfile(root,"Results","ScenarioSensorSource");
if ~isfolder(resultDir); mkdir(resultDir); end
result=runRoadsenseScenarioSensorSourceDemo(false);
exportgraphics(result.CameraFigure,fullfile(resultDir,"camera_scenario_snapshots.png"),"Resolution",180);
exportgraphics(result.GeometryFigure,fullfile(resultDir,"lidar_radar_scenario_snapshots.png"),"Resolution",180);
close(result.CameraFigure); close(result.GeometryFigure);
writetable(result.Summary,fullfile(resultDir,"sensor_source_metrics.csv"));
writetable(result.DropoutSchedule,fullfile(resultDir,"sensor_dropout_schedule.csv"));
metrics=table(height(result.Summary),all(result.Summary.CameraValid), ...
    all(result.Summary.LidarValid),all(result.Summary.RadarValid), ...
    sum(result.Summary.LidarPoints),sum(result.Summary.RadarDetections), ...
    'VariableNames',{'ScenarioCount','AllCameraSnapshotsValid','AllLidarSnapshotsValid', ...
    'AllRadarSnapshotsValid','TotalSnapshotLidarPoints','TotalSnapshotRadarDetections'});
writetable(metrics,fullfile(resultDir,"sensor_source_summary.csv"));
modelPath=createRoadsenseScenarioSensorSourceModel(); [~,modelName]=fileparts(modelPath);
load_system(modelPath); set_param(modelName,"ZoomFactor","FitSystem");
print("-s"+modelName,"-dpng","-r180",fullfile(resultDir,"model_layout.png"));
close_system(modelName,0);
result=rmfield(result,{'CameraFigure','GeometryFigure'}); %#ok<NASGU>
save(fullfile(resultDir,"sensor_source_results.mat"),"result","metrics");
fprintf("Wrote scenario-sensor-source evidence to %s\n",resultDir);
end
