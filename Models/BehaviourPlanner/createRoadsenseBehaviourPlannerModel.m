function modelPath=createRoadsenseBehaviourPlannerModel()
%CREATEROADSENSEBEHAVIOURPLANNERMODEL Generate assessment + Stateflow planner.

componentDir=fileparts(mfilename("fullpath"));
root=fileparts(fileparts(componentDir));
modelName="Roadsense_BehaviourPlanner";
modelPath=fullfile(componentDir,modelName+".slx");
addpath(fullfile(root,"Models","SharedData")); addpath(componentDir); addpath(fullfile(root,"Data"));
createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
if bdIsLoaded(modelName); close_system(modelName,0); end
if isfile(modelPath); delete(modelPath); end
new_system(modelName,"Model"); cleanup=onCleanup(@() closeIfLoaded(modelName));
set_param(modelName,"DataDictionary","Roadsense_Data.sldd","SolverType","Fixed-step", ...
    "Solver","FixedStepDiscrete","FixedStep","RsTsBase","StopTime","1.0","SignalLogging","on");

inputNames=["EgoState","SemanticGrid","FusedTracks","RouteContext"];
inputTypes=["Bus: RsEgoStateBus","Bus: RsSemanticGridBus", ...
    "Bus: RsFusedTrackListBus","Bus: RsRouteContextBus"];
for index=1:4
    y=100+(index-1)*65;
    add_block("simulink/Ports & Subsystems/In1",modelName+"/"+inputNames(index), ...
        "Position",[25 y 55 y+20],"Port",string(index),"OutDataTypeStr",inputTypes(index), ...
        "PortDimensions","1","SampleTime","RsTsPlanning");
end

assessment=modelName+"/Hazard and Context Assessment";
add_block("simulink/User-Defined Functions/MATLAB System",assessment,"Position",[135 80 390 355]);
set_param(assessment,"System","RoadsenseBehaviourAssessmentSystem");
set_param(assessment,"MaxTracks","double(RsMaxTracks)","GridRows","double(RsGridRows)", ...
    "GridCols","double(RsGridCols)","GridResolution","RsGridResolution", ...
    "GridXLimits","RsGridXLimits","GridYLimits","RsGridYLimits", ...
    "ReactionTime","RsBehaviourReactionTime", ...
    "EmergencyDeceleration","RsBehaviourEmergencyDeceleration", ...
    "ComfortDeceleration","RsBehaviourComfortDeceleration", ...
    "EmergencyTTC","RsBehaviourEmergencyTTC","FollowTimeGap","RsBehaviourFollowTimeGap", ...
    "CreepSpeed","RsBehaviourCreepSpeed","CautiousSpeed","RsBehaviourCautiousSpeed", ...
    "AvoidSpeed","RsBehaviourAvoidSpeed");
for index=1:4
    add_line(modelName,inputNames(index)+"/1","Hazard and Context Assessment/"+index,"autorouting","on");
end

chartPath=modelName+"/Stateflow Supervisory Decision";
add_block("sflib/Chart",chartPath,"Position",[500 75 695 245]);
buildDecisionChart(chartPath);
for index=1:8
    add_line(modelName,"Hazard and Context Assessment/"+(index+1), ...
        "Stateflow Supervisory Decision/"+index,"autorouting","on");
end
add_block("simulink/Sources/Constant",modelName+"/Emergency Hold Ticks", ...
    "Position",[410 370 485 395],"Value","RsBehaviourEmergencyHoldTicks", ...
    "OutDataTypeStr","uint16");
add_block("simulink/Sources/Constant",modelName+"/Yield Clear Ticks", ...
    "Position",[410 410 485 435],"Value","RsBehaviourYieldClearTicks", ...
    "OutDataTypeStr","uint16");
add_line(modelName,"Emergency Hold Ticks/1","Stateflow Supervisory Decision/9","autorouting","on");
add_line(modelName,"Yield Clear Ticks/1","Stateflow Supervisory Decision/10","autorouting","on");

command=modelName+"/Build Behaviour Command";
add_block("simulink/User-Defined Functions/MATLAB System",command,"Position",[790 80 1040 400]);
set_param(command,"System","RoadsenseBehaviourCommandSystem");
add_line(modelName,"Hazard and Context Assessment/1","Build Behaviour Command/1","autorouting","on");
add_line(modelName,"Stateflow Supervisory Decision/1","Build Behaviour Command/2","autorouting","on");
add_line(modelName,"Hazard and Context Assessment/2","Build Behaviour Command/3","autorouting","on");
for assessmentPort=10:22
    commandPort=assessmentPort-6;
    add_line(modelName,"Hazard and Context Assessment/"+assessmentPort, ...
        "Build Behaviour Command/"+commandPort,"autorouting","on");
end

add_block("simulink/Signal Attributes/Data Type Conversion",modelName+"/Mode Enum", ...
    "Position",[1090 115 1170 145],"OutDataTypeStr","Enum: RoadsenseTypes.BehaviourMode");
modeCodeLine=add_line(modelName,"Build Behaviour Command/3","Mode Enum/1","autorouting","on");
set_param(modeCodeLine,"Name","ModeCode");
add_block("simulink/Signal Routing/Bus Creator",modelName+"/Behaviour Command Bus", ...
    "Position",[1260 55 1270 425],"Inputs","16","OutDataTypeStr","Bus: RsBehaviourCommandBus");
line=add_line(modelName,"Build Behaviour Command/1","Behaviour Command Bus/1","autorouting","on");
set_param(line,"Name","Timestamp");
line=add_line(modelName,"Build Behaviour Command/2","Behaviour Command Bus/2","autorouting","on");
set_param(line,"Name","SequenceID");
line=add_line(modelName,"Mode Enum/1","Behaviour Command Bus/3","autorouting","on");
set_param(line,"Name","Mode");
commandNames=["TargetSpeed","MaximumAcceleration","MinimumAcceleration", ...
    "DesiredClearance","StopDistance","LeadTrackID","MinimumTTC","ForwardRisk", ...
    "UnknownFraction","ReasonMask","ReplanRequested","EmergencyRequested","Valid"];
for port=4:16
    line=add_line(modelName,"Build Behaviour Command/"+port, ...
        "Behaviour Command Bus/"+port,"autorouting","on");
    set_param(line,"Name",commandNames(port-3));
end
add_block("simulink/Ports & Subsystems/Out1",modelName+"/BehaviourCommand", ...
    "Position",[1360 225 1390 245],"OutDataTypeStr","Bus: RsBehaviourCommandBus");
add_line(modelName,"Behaviour Command Bus/1","BehaviourCommand/1","autorouting","on");

annotation=Simulink.Annotation(modelName,"Roadsense Behaviour Planner"+newline+ ...
    "Map/track hazard assessment -> Stateflow priority and hysteresis -> bounded command"+newline+ ...
    "invalid | emergency | stop | yield | avoid | follow | creep | cautious | cruise");
annotation.Position=[390 15 1030 62];
save_system(modelName,modelPath); clear cleanup; close_system(modelName,0);
fprintf("Generated %s\n",modelPath);
end

function buildDecisionChart(chartPath)
rootObject=sfroot;
chart=find(rootObject,"-isa","Stateflow.Chart","Path",char(chartPath));
chart.ActionLanguage="MATLAB";
inputNames={'InputsValid','EmergencyHazard','GoalStop','YieldRequired', ...
    'FollowRequired','CreepRecommended','AvoidRecommended','CautiousRecommended', ...
    'EmergencyHoldTicks','YieldClearTicks'};
inputTypes={'boolean','boolean','boolean','boolean','boolean','boolean','boolean','boolean', ...
    'uint16','uint16'};
for index=1:numel(inputNames)
    data=Stateflow.Data(chart); data.Name=inputNames{index}; data.Scope='Input';
    data.Port=index; data.DataType=inputTypes{index};
end
output=Stateflow.Data(chart); output.Name='ModeCode'; output.Scope='Output';
output.Port=1; output.DataType='uint8';
locals={'PreviousMode','EmergencyHoldCounter','YieldClearCounter'};
localTypes={'uint8','uint16','uint16'};
for index=1:numel(locals)
    data=Stateflow.Data(chart); data.Name=locals{index}; data.Scope='Local';
    data.DataType=localTypes{index};
end
state=Stateflow.State(chart); state.Name='SupervisoryDecision'; state.Position=[35 35 360 185];
state.LabelString=sprintf(['SupervisoryDecision\nentry:\n' ...
    ' PreviousMode = uint8(0);\n EmergencyHoldCounter = uint16(0);\n' ...
    ' YieldClearCounter = uint16(0);\n ModeCode = uint8(0);\n' ...
    'during:\n [ModeCode, EmergencyHoldCounter, YieldClearCounter] = ...\n' ...
    '  RoadsenseBehaviourDecisionCore(InputsValid, EmergencyHazard, GoalStop, ...\n' ...
    '  YieldRequired, FollowRequired, CreepRecommended, AvoidRecommended, ...\n' ...
    '  CautiousRecommended, PreviousMode, EmergencyHoldCounter, ...\n' ...
    '  YieldClearCounter, EmergencyHoldTicks, YieldClearTicks);\n' ...
    ' PreviousMode = ModeCode;']);
transition=Stateflow.Transition(chart); transition.Destination=state;
transition.DestinationOClock=0; transition.SourceEndPoint=[210 15];
transition.MidPoint=[210 25];
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName); close_system(modelName,0); end
end
