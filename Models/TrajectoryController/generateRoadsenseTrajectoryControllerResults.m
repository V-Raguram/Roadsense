function metrics=generateRoadsenseTrajectoryControllerResults()
%GENERATEROADSENSETRAJECTORYCONTROLLERRESULTS Save closed-loop evidence.
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
resultDir=fullfile(root,"Results","TrajectoryController");
if ~isfolder(resultDir); mkdir(resultDir); end
scenarios=["offset";"curved"]; rmsCrossTrack=zeros(2,1); finalCrossTrack=zeros(2,1);
rmsSpeed=zeros(2,1); maximumSteering=zeros(2,1); maximumAcceleration=zeros(2,1);
for index=1:2
    result=runRoadsenseTrajectoryControllerDemo(scenarios(index),true);
    rmsCrossTrack(index)=result.RmsCrossTrackError;
    finalCrossTrack(index)=result.FinalCrossTrackError;
    rmsSpeed(index)=result.RmsSpeedError;
    maximumSteering(index)=max(abs(rad2deg(result.SteeringAngle)));
    maximumAcceleration(index)=max(abs(result.Acceleration));
    exportgraphics(gcf,fullfile(resultDir,scenarios(index)+"_tracking_demo.png"), ...
        "Resolution",160); close(gcf);
end
metrics=table(scenarios,rmsCrossTrack,finalCrossTrack,rmsSpeed,maximumSteering, ...
    maximumAcceleration,'VariableNames',{'Scenario','RmsCrossTrackMetres', ...
    'FinalCrossTrackMetres','RmsSpeedErrorMetresPerSecond', ...
    'MaximumSteeringDegrees','MaximumAccelerationMetresPerSecondSquared'});
writetable(metrics,fullfile(resultDir,"trajectory_controller_metrics.csv"));
disp(metrics);
end
