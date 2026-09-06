function result=runRoadsenseScenarioAdaptersDemo(showPlot)
%RUNROADSENSESCENARIOADAPTERSDEMO Demonstrate route conversion and startup.
arguments
    showPlot (1,1) logical=true
end
data=createSyntheticRoadsenseScenarioAdapterInputs();
camera=RoadsenseCameraFrameAdapterSystem;
lidar=RoadsenseLidarFrameAdapterSystem;
radar=RoadsenseRadarObjectAdapterSystem;
route=RoadsenseRouteAdapterSystem;
initialization=RoadsenseScenarioInitializationSystem;
[~,cameraFrameID,~,cameraValid]=camera(data.CurrentTime,data.CameraImage,data.CameraValid);
[~,lidarFrameID,lidarCount,~,~,~,lidarValid]=lidar(data.CurrentTime, ...
    data.LidarPoints,data.LidarIntensity,data.LidarCount,data.LidarOverflow,data.LidarValid);
[~,radarFrameID,radarCount,~,~,~,~,~,~,~,radarValid]=radar(data.CurrentTime, ...
    data.RadarPositions,data.RadarVelocities,data.RadarPositionCovariances, ...
    data.RadarVelocityCovariances,data.RadarScores,data.RadarCount, ...
    data.RadarOverflow,data.RadarValid);

sampleIndices=1:2:double(data.ScenarioDefinition.Count);
numberSamples=numel(sampleIndices); time=(0:numberSamples-1).'*0.1;
distance=zeros(numberSamples,1); referenceCount=zeros(numberSamples,1,"uint16");
goal=false(numberSamples,1); firstForward=zeros(numberSamples,1);
reset=false(numberSamples,1);
for sample=1:numberSamples
    routeIndex=sampleIndices(sample);
    data.EgoState.Timestamp=time(sample);
    data.EgoState.Position=data.ScenarioDefinition.WorldPositions(routeIndex,:).';
    if routeIndex<double(data.ScenarioDefinition.Count)
        difference=data.ScenarioDefinition.WorldPositions(routeIndex+1,1:2)- ...
            data.ScenarioDefinition.WorldPositions(routeIndex,1:2);
        data.EgoState.Yaw=atan2(difference(2),difference(1));
    end
    [~,~,~,distance(sample),goal(sample),~,~,~,~,~,~,referenceCount(sample), ...
        positions,~,~,~,~]=route(time(sample),data.EgoState,data.ScenarioDefinition);
    firstForward(sample)=positions(1,1);
    [initialOutputs{1:17}]=initialization(time(sample),data.ScenarioDefinition,false);
    reset(sample)=initialOutputs{17};
end
release(camera); release(lidar); release(radar); release(route); release(initialization);
timeline=table(time,distance,referenceCount,firstForward,goal,reset, ...
    'VariableNames',{'Time','DistanceToGoal','ReferenceCount', ...
    'FirstForwardPoint','GoalReached','Reset'});
summary=table(cameraFrameID,lidarFrameID,radarFrameID,cameraValid,lidarValid, ...
    radarValid,lidarCount,radarCount,'VariableNames',{'CameraFrameID', ...
    'LidarFrameID','RadarFrameID','CameraValid','LidarValid','RadarValid', ...
    'LidarCount','RadarCount'});
result=struct("Timeline",timeline,"Summary",summary);
if showPlot
    figure("Name","Roadsense scenario adapters","Color","white", ...
        "Position",[100 100 1050 680]); tiledlayout(2,2,"TileSpacing","compact");
    nexttile([1 2]); count=double(data.ScenarioDefinition.Count);
    plot(data.ScenarioDefinition.WorldPositions(1:count,1), ...
        data.ScenarioDefinition.WorldPositions(1:count,2),"k-","LineWidth",1.5); hold on;
    positions=data.ScenarioDefinition.WorldPositions(sampleIndices,1:2);
    scatter(positions(:,1),positions(:,2),28,time,"filled"); axis equal; grid on;
    xlabel("World x (m)"); ylabel("World y (m)"); title("World route and ego samples"); colorbar;
    nexttile; plot(time,distance,"LineWidth",1.5); grid on;
    xlabel("Time (s)"); ylabel("Distance to goal (m)");
    nexttile; stairs(time,[double(referenceCount) goal reset],"LineWidth",1.4); grid on;
    xlabel("Time (s)"); legend("Reference count","Goal reached","Reset pulse");
end
end
