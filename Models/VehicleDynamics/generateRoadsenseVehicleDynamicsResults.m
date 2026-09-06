function metrics=generateRoadsenseVehicleDynamicsResults()
%GENERATEROADSENSEVEHICLEDYNAMICSRESULTS Save plant evidence and metrics.
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
resultDir=fullfile(root,"Results","VehicleDynamics");
if ~isfolder(resultDir); mkdir(resultDir); end
scenarios=["straight";"turn";"brake";"low_friction";"uphill"];
finalSpeed=zeros(5,1); distance=zeros(5,1); lateralDisplacement=zeros(5,1);
finalYaw=zeros(5,1); maximumSideslip=zeros(5,1); maximumUtilization=zeros(5,1);
saturatedSamples=zeros(5,1);
for index=1:5
    result=runRoadsenseVehicleDynamicsDemo(scenarios(index),false);
    finalSpeed(index)=result.Speed(end); distance(index)=result.Position(end,1);
    lateralDisplacement(index)=result.Position(end,2); finalYaw(index)=rad2deg(result.Yaw(end));
    maximumSideslip(index)=max(abs(rad2deg(result.Sideslip)));
    maximumUtilization(index)=max(result.FrictionUtilization);
    saturatedSamples(index)=nnz(result.Saturated);
end
metrics=table(scenarios,finalSpeed,distance,lateralDisplacement,finalYaw, ...
    maximumSideslip,maximumUtilization,saturatedSamples, ...
    'VariableNames',{'Scenario','FinalSpeedMetresPerSecond','ForwardMetres', ...
    'LateralMetres','FinalYawDegrees','MaximumSideslipDegrees', ...
    'MaximumFrictionUtilization','SaturatedSamples'});
writetable(metrics,fullfile(resultDir,"vehicle_dynamics_metrics.csv"));
plotScenarios=["turn","low_friction"];
for index=1:2
    runRoadsenseVehicleDynamicsDemo(plotScenarios(index),true);
    exportgraphics(gcf,fullfile(resultDir,plotScenarios(index)+"_dynamics_demo.png"), ...
        "Resolution",160); close(gcf);
end
disp(metrics);
end
