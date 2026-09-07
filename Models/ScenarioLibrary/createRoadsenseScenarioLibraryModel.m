function modelPath=createRoadsenseScenarioLibraryModel()
%CREATEROADSENSESCENARIOLIBRARYMODEL Generate five-scenario source model.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
modelName="Roadsense_ScenarioLibrary"; modelPath=fullfile(componentDir,modelName+".slx");
addpath(fullfile(root,"Models","SharedData")); addpath(componentDir); addpath(fullfile(root,"Data"));
createRoadsenseDataDictionary(fullfile(root,"Data","Roadsense_Data.sldd"));
if bdIsLoaded(modelName); close_system(modelName,0); end
if isfile(modelPath); delete(modelPath); end
new_system(modelName,"Model"); cleanup=onCleanup(@() closeIfLoaded(modelName));
set_param(modelName,"DataDictionary","Roadsense_Data.sldd","SolverType","Fixed-step", ...
    "Solver","FixedStepDiscrete","FixedStep","RsTsBase","StopTime","25", ...
    "SignalLogging","on","ScreenColor","white","ZoomFactor","FitSystem");

lightBlue="[0.82, 0.91, 1.00]"; orange="[1.00, 0.78, 0.43]";
add_block("simulink/Ports & Subsystems/In1",modelName+"/CurrentTime", ...
    "Position",[55 270 85 290],"OutDataTypeStr","double", ...
    "SampleTime","RsTsScenario","BackgroundColor",lightBlue);
add_block("simulink/Ports & Subsystems/In1",modelName+"/ScenarioSelect", ...
    "Position",[55 390 85 410],"Port","2","OutDataTypeStr","uint8", ...
    "SampleTime","RsTsScenario","BackgroundColor",lightBlue);

engine=modelName+"/Scenario Catalog and Truth Engine";
add_block("simulink/Ports & Subsystems/Subsystem",engine, ...
    "Position",[245 170 665 525],"BackgroundColor",orange, ...
    "ForegroundColor","black","FontWeight","bold","FontSize","12");
set_param(engine,"ContentPreviewEnabled","off");
Simulink.SubSystem.deleteContents(engine);
buildEngine(engine);
wire(modelName,"CurrentTime/1","Scenario Catalog and Truth Engine/1","time @ 10 Hz");
wire(modelName,"ScenarioSelect/1","Scenario Catalog and Truth Engine/2","scenario 1..5");

outputs=["ScenarioDefinition","TruthActors","ScenarioStatus"];
types=["Bus: RsScenarioDefinitionBus","Bus: RsScenarioActorListBus", ...
    "Bus: RsScenarioStatusBus"];
yValues=[240 340 440];
for index=1:3
    add_block("simulink/Ports & Subsystems/Out1",modelName+"/"+outputs(index), ...
        "Position",[875 yValues(index) 905 yValues(index)+20],"Port",string(index), ...
        "OutDataTypeStr",types(index));
    wire(modelName,"Scenario Catalog and Truth Engine/"+index,outputs(index)+"/1",outputs(index));
end

title=Simulink.Annotation(modelName,"ROADSENSE  |  INDIAN ROAD SCENARIO LIBRARY");
title.Position=[225 30 720 63]; title.FontSize=16; title.FontWeight="bold";
subtitle=Simulink.Annotation(modelName,sprintf([ ...
    '1  Village road     2  Uncontrolled intersection     3  Highway merge\n' ...
    '4  Dense market     5  Sudden cattle crossing']));
subtitle.Position=[235 75 760 125]; subtitle.FontSize=11;
legendNote=Simulink.Annotation(modelName,sprintf([ ...
    'BLUE  operator/time inputs\nORANGE  deterministic truth engine\n' ...
    'Outputs  route definition  |  actor truth  |  timed event status']));
legendNote.Position=[730 165 1050 255]; legendNote.FontSize=9;
interfaceNote=Simulink.Annotation(modelName,sprintf([ ...
    'Fixed-size, code-generation-friendly contracts\n' ...
    '64 actors  |  256 route points  |  10 Hz']));
interfaceNote.Position=[315 555 670 615]; interfaceNote.FontSize=10;
set_param(modelName,"Location",[100 80 1370 820]);
applyRoadsenseModelStyle(modelName);
save_system(modelName,modelPath); clear cleanup; close_system(modelName,0);
fprintf("Generated %s\n",modelPath);
end

function buildEngine(engine)
add_block("simulink/Ports & Subsystems/In1",engine+"/CurrentTime", ...
    "Position",[25 180 55 200],"OutDataTypeStr","double");
add_block("simulink/Ports & Subsystems/In1",engine+"/ScenarioSelect", ...
    "Position",[25 275 55 295],"Port","2","OutDataTypeStr","uint8");
library=engine+"/Five Indian Road Scenarios";
add_block("simulink/User-Defined Functions/MATLAB System",library, ...
    "Position",[145 85 415 390],"BackgroundColor","orange");
set_param(library,"System","RoadsenseScenarioLibrarySystem", ...
    "SimulateUsing","Interpreted execution");
wire(engine,"CurrentTime/1","Five Indian Road Scenarios/1","CurrentTime");
wire(engine,"ScenarioSelect/1","Five Indian Road Scenarios/2","ScenarioSelect");

definitionNames=["Timestamp","ScenarioID","Count","WorldPositions", ...
    "RecommendedSpeeds","ValidMask","DesiredSpeed","SpeedLimit","GoalRadius", ...
    "AllowObstacleAvoidance","UncontrolledIntersection","OccludedArea", ...
    "MergeRequired","InitialPosition","InitialYaw","InitialSpeed", ...
    "FrictionCoefficient","Grade","Bank","RollingResistance","Valid"];
truthNames=["Timestamp","Count","ActorIDs","ClassIDs","Positions","Velocities", ...
    "Yaws","Dimensions","ValidMask","Overflow","Valid"];
statusNames=["Timestamp","ScenarioID","ElapsedTime","Duration","EventMask", ...
    "ActiveActorCount","Valid"];
addBus(engine,"Scenario Definition",21,"Bus: RsScenarioDefinitionBus",[555 40 565 455]);
addBus(engine,"Truth Actors",11,"Bus: RsScenarioActorListBus",[555 485 565 755]);
addBus(engine,"Scenario Status",7,"Bus: RsScenarioStatusBus",[555 790 565 975]);
for port=1:21; wire(engine,"Five Indian Road Scenarios/"+port, ...
        "Scenario Definition/"+port,definitionNames(port)); end
for port=1:11; wire(engine,"Five Indian Road Scenarios/"+(port+21), ...
        "Truth Actors/"+port,truthNames(port)); end
for port=1:7; wire(engine,"Five Indian Road Scenarios/"+(port+32), ...
        "Scenario Status/"+port,statusNames(port)); end
outputs=["ScenarioDefinition","TruthActors","ScenarioStatus"];
types=["Bus: RsScenarioDefinitionBus","Bus: RsScenarioActorListBus", ...
    "Bus: RsScenarioStatusBus"];
sources=["Scenario Definition/1","Truth Actors/1","Scenario Status/1"];
yValues=[215 595 865];
for index=1:3
    add_block("simulink/Ports & Subsystems/Out1",engine+"/"+outputs(index), ...
        "Position",[735 yValues(index) 765 yValues(index)+20],"Port",string(index), ...
        "OutDataTypeStr",types(index));
    wire(engine,sources(index),outputs(index)+"/1",outputs(index));
end
set_param(engine,"ZoomFactor","FitSystem");
end

function addBus(system,name,count,type,position)
add_block("simulink/Signal Routing/Bus Creator",system+"/"+name, ...
    "Position",position,"Inputs",string(count),"OutDataTypeStr",type);
end
function wire(system,source,destination,name)
line=add_line(system,source,destination,"autorouting","on");
if strlength(name)>0; set_param(line,"Name",name); end
end
function closeIfLoaded(modelName)
if bdIsLoaded(modelName); close_system(modelName,0); end
end
