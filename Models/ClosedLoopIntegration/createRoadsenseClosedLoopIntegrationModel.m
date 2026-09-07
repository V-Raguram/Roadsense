function modelPath=createRoadsenseClosedLoopIntegrationModel()
%CREATEROADSENSECLOSEDLOOPINTEGRATIONMODEL Connect every Roadsense component.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
modelName="Roadsense_ClosedLoopIntegration";
modelPath=fullfile(componentDir,modelName+".slx");
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
ensureReferencedModels(root);
enforceSimulationModes(root);
if bdIsLoaded(modelName); close_system(modelName,0); end
if isfile(modelPath); delete(modelPath); end
new_system(modelName,"Model"); cleanup=onCleanup(@() closeIfLoaded(modelName));
set_param(modelName,"DataDictionary","Roadsense_Data.sldd", ...
    "SolverType","Fixed-step","Solver","FixedStepDiscrete", ...
    "FixedStep","RsTsBase","StopTime","2.0","SignalLogging","on");

inputNames=["CameraFrame","LidarFrame","RadarObjects","RouteContext", ...
    "ReferencePath","InitialState","RoadCondition","Reset"];
inputTypes=["Bus: RsCameraFrameBus","Bus: RsLidarFrameBus", ...
    "Bus: RsRadarObjectListBus","Bus: RsRouteContextBus", ...
    "Bus: RsReferencePathBus","Bus: RsEgoStateBus", ...
    "Bus: RsRoadConditionBus","boolean"];
% A referenced model must expose its root interface at the parent base rate.
% Explicit ingress transitions below recover each algorithm task rate.
inputTimes=repmat("RsTsBase",1,8);
for index=1:numel(inputNames)
    y=70+(index-1)*105;
    add_block("simulink/Ports & Subsystems/In1",modelName+"/"+inputNames(index), ...
        "Position",[20 y 50 y+20],"Port",string(index), ...
        "OutDataTypeStr",inputTypes(index),"PortDimensions","1", ...
        "SampleTime",inputTimes(index));
end

addModel(modelName,"Semantic Perception","Roadsense_SemanticPerception",[150 40 350 130]);
addModel(modelName,"LiDAR Perception","Roadsense_LidarPerception",[150 180 350 285]);
addModel(modelName,"Sensor Fusion","Roadsense_SensorFusion",[555 105 750 220]);
addModel(modelName,"Motion Prediction","Roadsense_MotionPrediction",[900 55 1100 130]);
addModel(modelName,"Semantic Map Fusion","Roadsense_SemanticMapFusion",[900 200 1100 335]);
addModel(modelName,"Behaviour Planner","Roadsense_BehaviourPlanner",[1240 310 1445 440]);
addModel(modelName,"Local Planner","Roadsense_LocalPlanner",[1585 205 1790 365]);
addModel(modelName,"Trajectory Controller","Roadsense_TrajectoryController",[1940 200 2150 315]);
addModel(modelName,"Safety Supervisor","Roadsense_SafetySupervisor",[2290 285 2505 485]);
addModel(modelName,"Vehicle Dynamics","Roadsense_VehicleDynamics",[500 655 705 790]);
add_block("simulink/Sources/Digital Clock",modelName+"/Fusion Clock", ...
    "Position",[440 55 490 85],"SampleTime","RsTsFusion");

addRate(modelName,"Camera Objects 10 to 20 Hz","RsTsFusion",[405 105 500 135]);
addRate(modelName,"LiDAR Objects 10 to 20 Hz","RsTsFusion",[405 220 500 250]);
addRate(modelName,"Tracks 20 to 10 Hz","RsTsPlanning",[790 125 870 155]);
addRate(modelName,"Ego 50 to 10 Hz","RsTsPlanning",[1040 570 1130 600]);
% LocalPlan and PlannerStatus form one decision transaction. Transfer them
% together so a controller tick cannot combine different planner updates.
planBundle=modelName+"/Atomic Plan and Status Bundle";
add_block("simulink/Signal Routing/Bus Creator",planBundle,"Position",[1810 205 1830 275], ...
    "Inputs","2","BackgroundColor","[0.98 0.93 0.72]");
addRate(modelName,"Atomic Plan and Status 10 to 50 Hz","RsTsControl",[1860 225 1960 255]);
planSelector=modelName+"/Atomic Plan and Status Selector";
add_block("simulink/Signal Routing/Bus Selector",planSelector,"Position",[1990 205 2020 275], ...
    "OutputSignals","LocalPlan,PlannerStatus","BackgroundColor","[0.98 0.93 0.72]");
addRate(modelName,"Map 10 to 50 Hz","RsTsControl",[1885 440 1975 470]);
addRate(modelName,"Tracks 20 to 50 Hz","RsTsControl",[1885 490 1975 520]);
% Fusion (20 Hz) and control (50 Hz) are both derived from the 100 Hz base
% task but are not integer multiples of one another.  A non-deterministic
% transition is therefore required; it still transfers atomically and the
% safety layer treats each fused-track frame as held data until the next hit.
set_param(modelName+"/Tracks 20 to 50 Hz","Deterministic","off");
addRate(modelName,"Behaviour 10 to 50 Hz","RsTsControl",[1885 540 1975 570]);
addRate(modelName,"Image Semantics 10 to 50 Hz","RsTsControl",[1190 25 1290 55]);
addRate(modelName,"LiDAR Grid 10 to 50 Hz","RsTsControl",[1190 85 1290 115]);
addRate(modelName,"Predictions 10 to 50 Hz","RsTsControl",[1190 145 1290 175]);
addRate(modelName,"Route 10 to 50 Hz","RsTsControl",[1190 655 1290 685]);
addRate(modelName,"Camera Ingress to 10 Hz","RsTsSemanticInference",[70 65 145 95]);
addRate(modelName,"LiDAR Ingress to 10 Hz","RsTsLidarPerception",[70 170 145 200]);
addRate(modelName,"Radar Ingress to 20 Hz","RsTsFusion",[70 275 145 305]);
addRate(modelName,"Route Ingress to 10 Hz","RsTsPlanning",[70 380 145 410]);
addRate(modelName,"Reference Ingress to 10 Hz","RsTsPlanning",[70 485 145 515]);
addRate(modelName,"Initial State Ingress to 50 Hz","RsTsControl",[70 590 145 620]);
addRate(modelName,"Road Ingress to 50 Hz","RsTsControl",[70 695 145 725]);
addRate(modelName,"Reset Ingress to 50 Hz","RsTsControl",[70 800 145 830]);

add_block("simulink/Discrete/Unit Delay",modelName+"/One Control Tick Actuator Delay", ...
    "Position",[2300 610 2430 650],"SampleTime","RsTsControl", ...
    "InitialCondition","0");

% Sensor perception, fusion, prediction, and map.
wire(modelName,"CameraFrame/1","Camera Ingress to 10 Hz/1","CameraFrame");
wire(modelName,"Camera Ingress to 10 Hz/1","Semantic Perception/1","");
wire(modelName,"LidarFrame/1","LiDAR Ingress to 10 Hz/1","LidarFrame");
wire(modelName,"LiDAR Ingress to 10 Hz/1","LiDAR Perception/1","");
wire(modelName,"Semantic Perception/2","Camera Objects 10 to 20 Hz/1","CameraObjects");
wire(modelName,"Fusion Clock/1","Sensor Fusion/1","CurrentTime");
wire(modelName,"Camera Objects 10 to 20 Hz/1","Sensor Fusion/2","CameraObjects20Hz");
wire(modelName,"LiDAR Perception/2","LiDAR Objects 10 to 20 Hz/1","LidarObjects");
wire(modelName,"LiDAR Objects 10 to 20 Hz/1","Sensor Fusion/3","LidarObjects20Hz");
wire(modelName,"RadarObjects/1","Radar Ingress to 20 Hz/1","RadarObjects");
wire(modelName,"Radar Ingress to 20 Hz/1","Sensor Fusion/4","");
wire(modelName,"Sensor Fusion/1","Tracks 20 to 10 Hz/1","FusedTracks");
wire(modelName,"Tracks 20 to 10 Hz/1","Motion Prediction/1","PlanningTracks");
wire(modelName,"Semantic Perception/1","Semantic Map Fusion/1","ImageSemantic");
wire(modelName,"LiDAR Perception/1","Semantic Map Fusion/2","LidarGrid");
wire(modelName,"Tracks 20 to 10 Hz/1","Semantic Map Fusion/3","PlanningTracks");
wire(modelName,"Motion Prediction/1","Semantic Map Fusion/4","Predictions");

% Planning and nominal trajectory control.
wire(modelName,"Vehicle Dynamics/1","Ego 50 to 10 Hz/1","EgoState");
wire(modelName,"Ego 50 to 10 Hz/1","Behaviour Planner/1","PlanningEgo");
wire(modelName,"Semantic Map Fusion/1","Behaviour Planner/2","SemanticGrid");
wire(modelName,"Tracks 20 to 10 Hz/1","Behaviour Planner/3","PlanningTracks");
wire(modelName,"RouteContext/1","Route Ingress to 10 Hz/1","RouteContext");
wire(modelName,"Route Ingress to 10 Hz/1","Behaviour Planner/4","");
wire(modelName,"Ego 50 to 10 Hz/1","Local Planner/1","PlanningEgo");
wire(modelName,"Semantic Map Fusion/1","Local Planner/2","SemanticGrid");
wire(modelName,"Tracks 20 to 10 Hz/1","Local Planner/3","PlanningTracks");
wire(modelName,"Motion Prediction/1","Local Planner/4","Predictions");
wire(modelName,"Behaviour Planner/1","Local Planner/5","BehaviourCommand");
wire(modelName,"ReferencePath/1","Reference Ingress to 10 Hz/1","ReferencePath");
wire(modelName,"Reference Ingress to 10 Hz/1","Local Planner/6","");
wire(modelName,"Local Planner/1","Atomic Plan and Status Bundle/1","LocalPlan");
wire(modelName,"Local Planner/2","Atomic Plan and Status Bundle/2","PlannerStatus");
wire(modelName,"Atomic Plan and Status Bundle/1", ...
    "Atomic Plan and Status 10 to 50 Hz/1","AtomicPlanStatus");
wire(modelName,"Atomic Plan and Status 10 to 50 Hz/1", ...
    "Atomic Plan and Status Selector/1","");
wire(modelName,"Vehicle Dynamics/1","Trajectory Controller/1","EgoState");
wire(modelName,"Atomic Plan and Status Selector/1","Trajectory Controller/2","");
wire(modelName,"Atomic Plan and Status Selector/2","Trajectory Controller/3","");

% Safety-only rate conversions and independent command authority.
wire(modelName,"Semantic Map Fusion/1","Map 10 to 50 Hz/1","SemanticGrid");
wire(modelName,"Sensor Fusion/1","Tracks 20 to 50 Hz/1","FusedTracks");
wire(modelName,"Behaviour Planner/1","Behaviour 10 to 50 Hz/1","BehaviourCommand");
wire(modelName,"Vehicle Dynamics/1","Safety Supervisor/1","EgoState");
wire(modelName,"Map 10 to 50 Hz/1","Safety Supervisor/2","SafetyMap");
wire(modelName,"Tracks 20 to 50 Hz/1","Safety Supervisor/3","SafetyTracks");
wire(modelName,"Behaviour 10 to 50 Hz/1","Safety Supervisor/4","SafetyBehaviour");
wire(modelName,"Atomic Plan and Status Selector/2","Safety Supervisor/5","");
wire(modelName,"Trajectory Controller/2","Safety Supervisor/6","TrackingStatus");
wire(modelName,"Vehicle Dynamics/2","Safety Supervisor/7","DynamicsStatus");
wire(modelName,"Trajectory Controller/1","Safety Supervisor/8","NominalControl");

% The delay is the only actuator path and breaks the plant/safety feedback loop.
wire(modelName,"Safety Supervisor/1","One Control Tick Actuator Delay/1","SafeVehicleControl");
wire(modelName,"One Control Tick Actuator Delay/1","Vehicle Dynamics/1","DelayedSafeControl");
wire(modelName,"InitialState/1","Initial State Ingress to 50 Hz/1","InitialState");
wire(modelName,"Initial State Ingress to 50 Hz/1","Vehicle Dynamics/2","");
wire(modelName,"RoadCondition/1","Road Ingress to 50 Hz/1","RoadCondition");
wire(modelName,"Road Ingress to 50 Hz/1","Vehicle Dynamics/3","");
wire(modelName,"Reset/1","Reset Ingress to 50 Hz/1","Reset");
wire(modelName,"Reset Ingress to 50 Hz/1","Vehicle Dynamics/4","");

% Additional 50 Hz monitor conversions.
wire(modelName,"Semantic Perception/1","Image Semantics 10 to 50 Hz/1","ImageSemantic");
wire(modelName,"LiDAR Perception/1","LiDAR Grid 10 to 50 Hz/1","LidarGrid");
wire(modelName,"Motion Prediction/1","Predictions 10 to 50 Hz/1","Predictions");
wire(modelName,"Route Ingress to 10 Hz/1","Route 10 to 50 Hz/1","RouteContext");

monitor=modelName+"/End-to-End Health Monitor";
add_block("simulink/User-Defined Functions/MATLAB System",monitor, ...
    "Position",[1435 720 1725 1110]);
set_param(monitor,"System","RoadsenseIntegrationMonitorSystem");
monitorSources=["Vehicle Dynamics/1","Image Semantics 10 to 50 Hz/1", ...
    "LiDAR Grid 10 to 50 Hz/1","Tracks 20 to 50 Hz/1", ...
    "Predictions 10 to 50 Hz/1","Map 10 to 50 Hz/1", ...
    "Behaviour 10 to 50 Hz/1","Atomic Plan and Status Selector/1", ...
    "Atomic Plan and Status Selector/2","Trajectory Controller/1", ...
    "Trajectory Controller/2","Safety Supervisor/2", ...
    "Safety Supervisor/1","Vehicle Dynamics/2","Route 10 to 50 Hz/1"];
monitorTags=["RsTapEgoState","RsTapImageSemantics","RsTapLidarGrid", ...
    "RsTapFusedTracks","RsTapPredictions","RsTapSemanticGrid", ...
    "RsTapBehaviour","RsTapLocalPlan","RsTapPlannerStatus", ...
    "RsTapNominalControl","RsTapTrackingStatus","RsTapSafetyStatus", ...
    "RsTapSafeControl","RsTapDynamicsStatus","RsTapRouteContext"];
tapPositions=[ ...
    715 680; 1305 20; 1305 80; 760 80; 1110 20; ...
    1110 180; 1450 335; 2035 180; 2035 250; 2160 185; ...
    2160 255; 2515 415; 2515 285; 715 750; 1305 640];
for port=1:numel(monitorSources)
    addSignalTap(modelName,"Signal Tap "+port,monitorSources(port), ...
        monitorTags(port),tapPositions(port,:));
    y=735+(port-1)*24;
    addSignalSource(modelName,"Monitor Source "+port,monitorTags(port), ...
        [1340 y 1410 y+18]);
    wire(modelName,"Monitor Source "+port+"/1", ...
        "End-to-End Health Monitor/"+port,"");
end
add_block("simulink/Signal Routing/Bus Creator",modelName+"/Integration Status Bus", ...
    "Position",[1810 720 1820 1105],"Inputs","16", ...
    "OutDataTypeStr","Bus: RsIntegrationStatusBus");
statusNames=["Timestamp","CycleID","PerceptionValid","FusionValid", ...
    "PredictionValid","MapValid","BehaviourValid","PlanValid", ...
    "TrackingValid","SafetyValid","VehicleValid","PipelineReady", ...
    "SafetyOverride","EmergencyStop","GoalReached","Valid"];
for port=1:16
    wire(modelName,"End-to-End Health Monitor/"+port,"Integration Status Bus/"+port,statusNames(port));
end
addSignalTap(modelName,"Signal Tap Integration Status", ...
    "Integration Status Bus/1","RsTapIntegrationStatus",[1840 895]);

outputNames=["EgoState","SemanticGrid","FusedTracks","LocalPlan", ...
    "SafeVehicleControl","SafetyStatus","IntegrationStatus","PlannerStatus", ...
    "TrackingStatus","DynamicsStatus","BehaviourCommand"];
outputTypes=["Bus: RsEgoStateBus","Bus: RsSemanticGridBus", ...
    "Bus: RsFusedTrackListBus","Bus: RsLocalPlanBus", ...
    "Bus: RsVehicleControlBus","Bus: RsSafetyStatusBus", ...
    "Bus: RsIntegrationStatusBus","Bus: RsPlannerStatusBus", ...
    "Bus: RsTrackingStatusBus","Bus: RsVehicleDynamicsStatusBus", ...
    "Bus: RsBehaviourCommandBus"];
outputTags=["RsTapEgoState","RsTapSemanticGrid","RsTapFusedTracks", ...
    "RsTapLocalPlan","RsTapSafeControl","RsTapSafetyStatus", ...
    "RsTapIntegrationStatus","RsTapPlannerStatus","RsTapTrackingStatus", ...
    "RsTapDynamicsStatus","RsTapBehaviour"];
for index=1:numel(outputNames)
    y=90+(index-1)*105;
    addSignalSource(modelName,"Output Source "+outputNames(index), ...
        outputTags(index),[2550 y 2630 y+20]);
    add_block("simulink/Ports & Subsystems/Out1",modelName+"/"+outputNames(index), ...
        "Position",[2690 y 2720 y+20],"Port",string(index), ...
        "OutDataTypeStr",outputTypes(index));
    wire(modelName,"Output Source "+outputNames(index)+"/1", ...
        outputNames(index)+"/1",outputNames(index));
end

title=Simulink.Annotation(modelName,"ROADSENSE  |  CLOSED-LOOP AUTONOMY STACK");
title.Position=[870 1160 1880 1205]; title.FontSize=16; title.FontWeight="bold";
annotation=Simulink.Annotation(modelName, ...
    "10 Hz perception/map/planning | 20 Hz JPDA fusion | 50 Hz control/safety/plant"+newline+ ...
    "SafeVehicleControl -> one control-tick delay -> vehicle dynamics");
annotation.Position=[815 1210 1940 1275]; annotation.FontSize=10;
zone1=Simulink.Annotation(modelName,"1  PERCEPTION + MULTI-SENSOR FUSION");
zone1.Position=[150 5 750 30]; zone1.FontSize=11; zone1.FontWeight="bold";
zone2=Simulink.Annotation(modelName,"2  PREDICTION + SEMANTIC WORLD MODEL");
zone2.Position=[845 5 1335 30]; zone2.FontSize=11; zone2.FontWeight="bold";
zone3=Simulink.Annotation(modelName,"3  BEHAVIOUR + LOCAL PLANNING");
zone3.Position=[1220 285 1790 310]; zone3.FontSize=11; zone3.FontWeight="bold";
zone4=Simulink.Annotation(modelName,"4  CONTROL + INDEPENDENT SAFETY");
zone4.Position=[1915 145 2520 170]; zone4.FontSize=11; zone4.FontWeight="bold";
zone5=Simulink.Annotation(modelName,"5  CLOSED-LOOP PLANT + HEALTH EVIDENCE");
zone5.Position=[620 1120 1770 1150]; zone5.FontSize=11; zone5.FontWeight="bold";
set_param(modelName,"Location",[15 40 1900 1050]);
applyRoadsenseModelStyle(modelName);
save_system(modelName,modelPath); clear cleanup; close_system(modelName,0);
fprintf("Generated %s\n",modelPath);
end

function addModel(modelName,blockName,referencedModel,position)
path=modelName+"/"+blockName;
add_block("simulink/Ports & Subsystems/Model",path,"Position",position);
set_param(path,"ModelName",referencedModel,"SimulationMode","Normal");
end

function addRate(modelName,blockName,outSampleTime,position)
path=modelName+"/"+blockName;
add_block("simulink/Signal Attributes/Rate Transition",path,"Position",position);
set_param(path,"OutPortSampleTime",outSampleTime);
end

function wire(modelName,source,destination,signalName)
line=add_line(modelName,source,destination,"autorouting","on");
if strlength(signalName)>0; set_param(line,"Name",signalName); end
end

function addSignalTap(modelName,blockName,source,tag,position)
path=modelName+"/"+blockName;
add_block("simulink/Signal Routing/Goto",path, ...
    "Position",[position(1) position(2) position(1)+70 position(2)+18], ...
    "GotoTag",tag,"TagVisibility","local","ShowName","off");
wire(modelName,source,blockName+"/1","");
end

function addSignalSource(modelName,blockName,tag,position)
add_block("simulink/Signal Routing/From",modelName+"/"+blockName, ...
    "Position",position,"GotoTag",tag,"ShowName","off");
end

function ensureReferencedModels(root)
models=["SemanticPerception/Roadsense_SemanticPerception.slx", ...
    "LidarPerception/Roadsense_LidarPerception.slx", ...
    "SensorFusion/Roadsense_SensorFusion.slx", ...
    "MotionPrediction/Roadsense_MotionPrediction.slx", ...
    "SemanticMapFusion/Roadsense_SemanticMapFusion.slx", ...
    "BehaviourPlanner/Roadsense_BehaviourPlanner.slx", ...
    "LocalPlanner/Roadsense_LocalPlanner.slx", ...
    "TrajectoryController/Roadsense_TrajectoryController.slx", ...
    "SafetySupervisor/Roadsense_SafetySupervisor.slx", ...
    "VehicleDynamics/Roadsense_VehicleDynamics.slx"];
generators=["createRoadsenseSemanticPerceptionModel", ...
    "createRoadsenseLidarPerceptionModel","createRoadsenseSensorFusionModel", ...
    "createRoadsenseMotionPredictionModel","createRoadsenseSemanticMapFusionModel", ...
    "createRoadsenseBehaviourPlannerModel","createRoadsenseLocalPlannerModel", ...
    "createRoadsenseTrajectoryControllerModel","createRoadsenseSafetySupervisorModel", ...
    "createRoadsenseVehicleDynamicsModel"];
for index=1:numel(models)
    if ~isfile(fullfile(root,"Models",models(index))); feval(generators(index)); end
end
end

function enforceSimulationModes(root)
% MATLAB-only AI/tracking algorithms use interpreted SIL execution.
relativeModels=["SemanticPerception/Roadsense_SemanticPerception.slx", ...
    "LidarPerception/Roadsense_LidarPerception.slx", ...
    "SensorFusion/Roadsense_SensorFusion.slx", ...
    "MotionPrediction/Roadsense_MotionPrediction.slx", ...
    "SemanticMapFusion/Roadsense_SemanticMapFusion.slx"];
for index=1:numel(relativeModels)
    modelPath=fullfile(root,"Models",relativeModels(index));
    [~,name]=fileparts(modelPath); load_system(modelPath);
    systems=find_system(name,"LookUnderMasks","all","FollowLinks","on", ...
        "BlockType","MATLABSystem");
    changed=false;
    for blockIndex=1:numel(systems)
        if ~strcmp(get_param(systems{blockIndex},"SimulateUsing"),"Interpreted execution")
            set_param(systems{blockIndex},"SimulateUsing","Interpreted execution");
            changed=true;
        end
    end
    if changed; save_system(name,modelPath); end
    close_system(name,0);
end
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName); close_system(modelName,0); end
end
