function modelPath=createRoadsenseScenarioAdaptersModel()
%CREATEROADSENSESCENARIOADAPTERSMODEL Generate raw-scenario interface model.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
modelName="Roadsense_ScenarioAdapters"; modelPath=fullfile(componentDir,modelName+".slx");
addpath(fullfile(root,"Models","SharedData")); addpath(componentDir); addpath(fullfile(root,"Data"));
createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
if bdIsLoaded(modelName); close_system(modelName,0); end
if isfile(modelPath); delete(modelPath); end
new_system(modelName,"Model"); cleanup=onCleanup(@() closeIfLoaded(modelName));
set_param(modelName,"DataDictionary","Roadsense_Data.sldd","SolverType","Fixed-step", ...
    "Solver","FixedStepDiscrete","FixedStep","RsTsBase","StopTime","1.0", ...
    "SignalLogging","on");

names=["CurrentTime","CameraImage","CameraValid","LidarPoints", ...
    "LidarIntensity","LidarCount","LidarOverflow","LidarValid", ...
    "RadarPositions","RadarVelocities","RadarPositionCovariances", ...
    "RadarVelocityCovariances","RadarScores","RadarCount","RadarOverflow", ...
    "RadarValid","EgoState","ScenarioDefinition","ResetRequest"];
types=["double","uint8","boolean","single","single","uint32","boolean", ...
    "boolean","single","single","single","single","single","uint16", ...
    "boolean","boolean","Bus: RsEgoStateBus","Bus: RsScenarioDefinitionBus","boolean"];
dimensions={"1","[480 640 3]","1", ...
    "[120000 3]","[120000 1]","1","1","1", ...
    "[256 3]","[256 3]","[3 3 256]","[3 3 256]", ...
    "[256 1]","1","1","1","1","1","1"};
times=["RsTsBase","RsTsSemanticInference","RsTsSemanticInference", ...
    "RsTsLidarPerception","RsTsLidarPerception","RsTsLidarPerception", ...
    "RsTsLidarPerception","RsTsLidarPerception","RsTsFusion","RsTsFusion", ...
    "RsTsFusion","RsTsFusion","RsTsFusion","RsTsFusion","RsTsFusion", ...
    "RsTsFusion","RsTsControl","RsTsPlanning","RsTsControl"];
for index=1:numel(names)
    y=55+(index-1)*58;
    add_block("simulink/Ports & Subsystems/In1",modelName+"/"+names(index), ...
        "Position",[20 y 50 y+20],"Port",string(index),"OutDataTypeStr",types(index), ...
        "PortDimensions",dimensions{index},"SampleTime",times(index));
end

addRate(modelName,"Time 100 to 10 Hz","RsTsPlanning",[105 45 190 75]);
addRate(modelName,"Time 100 to 20 Hz","RsTsFusion",[105 90 190 120]);
addRate(modelName,"Time 100 to 50 Hz","RsTsControl",[105 135 190 165]);
addRate(modelName,"Ego 50 to 10 Hz","RsTsPlanning",[700 800 795 830]);
addRate(modelName,"Scenario 10 to 50 Hz","RsTsControl",[700 865 805 895]);
wire(modelName,"CurrentTime/1","Time 100 to 10 Hz/1","CurrentTime");
wire(modelName,"CurrentTime/1","Time 100 to 20 Hz/1","CurrentTime");
wire(modelName,"CurrentTime/1","Time 100 to 50 Hz/1","CurrentTime");
wire(modelName,"EgoState/1","Ego 50 to 10 Hz/1","EgoState");
wire(modelName,"ScenarioDefinition/1","Scenario 10 to 50 Hz/1","ScenarioDefinition");

camera=modelName+"/Camera Frame Adapter";
add_block("simulink/User-Defined Functions/MATLAB System",camera, ...
    "Position",[260 45 485 145]);
set_param(camera,"System","RoadsenseCameraFrameAdapterSystem");
wire(modelName,"Time 100 to 10 Hz/1","Camera Frame Adapter/1","CameraTime");
wire(modelName,"CameraImage/1","Camera Frame Adapter/2","CameraImage");
wire(modelName,"CameraValid/1","Camera Frame Adapter/3","CameraValid");
addBus(modelName,"Camera Frame Bus","Bus: RsCameraFrameBus",4,[555 35 565 165]);
cameraNames=["Timestamp","FrameID","Image","Valid"];
for port=1:4; wire(modelName,"Camera Frame Adapter/"+port,"Camera Frame Bus/"+port,cameraNames(port)); end

lidar=modelName+"/LiDAR Frame Adapter";
add_block("simulink/User-Defined Functions/MATLAB System",lidar, ...
    "Position",[260 220 485 400]);
set_param(lidar,"System","RoadsenseLidarFrameAdapterSystem", ...
    "MaxPoints","double(RsMaxLidarPoints)");
lidarSources=["Time 100 to 10 Hz/1","LidarPoints/1","LidarIntensity/1", ...
    "LidarCount/1","LidarOverflow/1","LidarValid/1"];
for port=1:6; wire(modelName,lidarSources(port),"LiDAR Frame Adapter/"+port,""); end
addBus(modelName,"LiDAR Frame Bus","Bus: RsLidarFrameBus",7,[555 210 565 420]);
lidarNames=["Timestamp","FrameID","Count","Points","Intensity","Overflow","Valid"];
for port=1:7; wire(modelName,"LiDAR Frame Adapter/"+port,"LiDAR Frame Bus/"+port,lidarNames(port)); end

radar=modelName+"/Radar Object Adapter";
add_block("simulink/User-Defined Functions/MATLAB System",radar, ...
    "Position",[260 475 485 730]);
set_param(radar,"System","RoadsenseRadarObjectAdapterSystem", ...
    "MaxDetections","double(RsMaxDetections)");
radarSources=["Time 100 to 20 Hz/1","RadarPositions/1","RadarVelocities/1", ...
    "RadarPositionCovariances/1","RadarVelocityCovariances/1","RadarScores/1", ...
    "RadarCount/1","RadarOverflow/1","RadarValid/1"];
for port=1:9; wire(modelName,radarSources(port),"Radar Object Adapter/"+port,""); end
addBus(modelName,"Radar Object Bus","Bus: RsRadarObjectListBus",11,[555 455 565 750]);
radarNames=["Timestamp","FrameID","Count","Positions","Velocities", ...
    "PositionCovariances","VelocityCovariances","Scores","ValidMask","Overflow","Valid"];
for port=1:11; wire(modelName,"Radar Object Adapter/"+port,"Radar Object Bus/"+port,radarNames(port)); end

route=modelName+"/World Route Adapter";
add_block("simulink/User-Defined Functions/MATLAB System",route, ...
    "Position",[900 150 1160 500]);
set_param(route,"System","RoadsenseRouteAdapterSystem", ...
    "MaxReferencePoints","double(RsMaxReferencePoints)");
wire(modelName,"Time 100 to 10 Hz/1","World Route Adapter/1","RouteTime");
wire(modelName,"Ego 50 to 10 Hz/1","World Route Adapter/2","PlanningEgo");
wire(modelName,"ScenarioDefinition/1","World Route Adapter/3","ScenarioDefinition");
addBus(modelName,"Route Context Bus","Bus: RsRouteContextBus",10,[1260 110 1270 390]);
routeNames=["Timestamp","DesiredSpeed","SpeedLimit","DistanceToGoal","GoalReached", ...
    "AllowObstacleAvoidance","UncontrolledIntersection","OccludedArea","MergeRequired","Valid"];
for port=1:10; wire(modelName,"World Route Adapter/"+port,"Route Context Bus/"+port,routeNames(port)); end
addBus(modelName,"Reference Path Bus","Bus: RsReferencePathBus",7,[1260 440 1270 650]);
referenceNames=["Timestamp","Count","Positions","Yaws","RecommendedSpeeds","ValidMask","Valid"];
for port=1:7
    wire(modelName,"World Route Adapter/"+(port+10),"Reference Path Bus/"+port,referenceNames(port));
end

initialization=modelName+"/Scenario Initialization";
add_block("simulink/User-Defined Functions/MATLAB System",initialization, ...
    "Position",[900 725 1160 1080]);
set_param(initialization,"System","RoadsenseScenarioInitializationSystem");
wire(modelName,"Time 100 to 50 Hz/1","Scenario Initialization/1","ControlTime");
wire(modelName,"Scenario 10 to 50 Hz/1","Scenario Initialization/2","ControlScenario");
wire(modelName,"ResetRequest/1","Scenario Initialization/3","ResetRequest");
addBus(modelName,"Initial State Bus","Bus: RsEgoStateBus",10,[1260 700 1270 965]);
initialNames=["Timestamp","Position","Velocity","Acceleration","Yaw","Pitch", ...
    "Roll","YawRate","SteeringAngle","Valid"];
for port=1:10; wire(modelName,"Scenario Initialization/"+port,"Initial State Bus/"+port,initialNames(port)); end
addBus(modelName,"Road Condition Bus","Bus: RsRoadConditionBus",6,[1260 1000 1270 1170]);
roadNames=["Timestamp","FrictionCoefficient","Grade","Bank","RollingResistance","Valid"];
for port=1:6
    wire(modelName,"Scenario Initialization/"+(port+10),"Road Condition Bus/"+port,roadNames(port));
end

outputNames=["CameraFrame","LidarFrame","RadarObjects","RouteContext", ...
    "ReferencePath","InitialState","RoadCondition","Reset"];
outputTypes=["Bus: RsCameraFrameBus","Bus: RsLidarFrameBus", ...
    "Bus: RsRadarObjectListBus","Bus: RsRouteContextBus", ...
    "Bus: RsReferencePathBus","Bus: RsEgoStateBus","Bus: RsRoadConditionBus","boolean"];
outputSources=["Camera Frame Bus/1","LiDAR Frame Bus/1","Radar Object Bus/1", ...
    "Route Context Bus/1","Reference Path Bus/1","Initial State Bus/1", ...
    "Road Condition Bus/1","Scenario Initialization/17"];
for index=1:numel(outputNames)
    y=90+(index-1)*125;
    add_block("simulink/Ports & Subsystems/Out1",modelName+"/"+outputNames(index), ...
        "Position",[1470 y 1500 y+20],"Port",string(index), ...
        "OutDataTypeStr",outputTypes(index));
    wire(modelName,outputSources(index),outputNames(index)+"/1",outputNames(index));
end

annotation=Simulink.Annotation(modelName,"Roadsense Scenario Adapters"+newline+ ...
    "raw camera/LiDAR/radar -> typed buses | world route -> ego route"+newline+ ...
    "scenario initialization + road conditions + one-tick reset sequencing");
annotation.Position=[590 1210 1320 1280];
set_param(modelName,"Location",[20 40 1850 1000]);
applyRoadsenseModelStyle(modelName);
save_system(modelName,modelPath); clear cleanup; close_system(modelName,0);
fprintf("Generated %s\n",modelPath);
end

function addRate(modelName,name,sampleTime,position)
add_block("simulink/Signal Attributes/Rate Transition",modelName+"/"+name,"Position",position);
set_param(modelName+"/"+name,"OutPortSampleTime",sampleTime);
end
function addBus(modelName,name,type,count,position)
add_block("simulink/Signal Routing/Bus Creator",modelName+"/"+name, ...
    "Position",position,"Inputs",string(count),"OutDataTypeStr",type);
end
function wire(modelName,source,destination,name)
line=add_line(modelName,source,destination,"autorouting","on");
if strlength(name)>0; set_param(line,"Name",name); end
end
function closeIfLoaded(modelName)
if bdIsLoaded(modelName); close_system(modelName,0); end
end
