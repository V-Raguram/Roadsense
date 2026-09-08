function manifest=createRoadsenseRoadRunnerScenarios(projectFolder)
%CREATEROADSENSEROADRUNNERSCENARIOS Author all five dynamic 3D scenarios.
arguments
    projectFolder (1,1) string = defaultProjectFolder()
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
assetDir=fullfile(root,"Scenarios","RoadRunner");
manifest=importRoadsenseRoadRunnerScenes(projectFolder);
modelPath=createRoadsenseRoadRunnerIntegrationModel();
installationFolder=defaultRoadRunnerInstallation();
rrApp=roadrunner(ProjectFolder=projectFolder, ...
    InstallationFolder=installationFolder,NoDisplay=true);
cleanup=onCleanup(@() close(rrApp));
egoActorIDs=zeros(height(manifest),1,"uint64");
actorCounts=zeros(height(manifest),1,"uint16");

for scenarioID=1:height(manifest)
    openScene(rrApp,manifest.SceneFile(scenarioID));
    newScenario(rrApp);
    rrApi=roadrunnerAPI(rrApp);
    scenario=rrApi.Scenario; project=rrApi.Project;
    sedan=project.getAsset("Vehicles/Sedan.fbx","VehicleAsset");
    pedestrian=project.getAsset("Characters/Citizen_Male.rrchar","CharacterAsset");
    behaviour=ensureSimulinkBehaviour(project,modelPath);
    [definition,~,status,plans]=RoadsenseScenarioCatalog(scenarioID,0);

    ego=scenario.addActor(sedan,double(definition.InitialPosition(:).'));
    ego.Name="Roadsense Ego"; ego.Color="blue";
    ego.BehaviorAsset=behaviour;
    egoActorIDs(scenarioID)=ego.ActorID;

    for actorIndex=1:numel(plans)
        plan=plans(actorIndex);
        if ismember(plan.ClassID,uint8([7 9]))
            asset=pedestrian;
        else
            asset=sedan;
        end
        actor=scenario.addActor(asset,plan.Waypoints(1,:));
        actor.Name=actorName(plan.ClassID,plan.ActorID);
        actor.Color=actorColor(plan.ClassID);
        configureTimedRoute(scenario,actor,plan);
    end
    actorCounts(scenarioID)=uint16(numel(plans)+1);
    endCondition=scenario.PhaseLogic.RootPhase.setEndCondition( ...
        "SimulationTimeCondition");
    endCondition.Time=double(status.Duration);
    saveScenario(rrApp,manifest.ScenarioFile(scenarioID));
    copyfile(fullfile(projectFolder,"Scenarios",manifest.ScenarioFile(scenarioID)), ...
        fullfile(assetDir,manifest.ScenarioFile(scenarioID)));
end
manifest.EgoActorID=egoActorIDs;
manifest.ActorCount=actorCounts;
portable=manifest;
portable.HDMapFile="Scenarios/RoadRunner/"+erase(manifest.SceneFile,".rrscene")+".rrhd";
writetable(portable,fullfile(assetDir,"roadrunner_scene_manifest.csv"));
fprintf("Authored five dynamic RoadRunner scenarios in %s\n",projectFolder);
end

function behaviour=ensureSimulinkBehaviour(project,modelPath)
assetPath="Behaviors/RoadsenseAutonomy.rrbehavior";
try
    behaviour=project.getAsset(assetPath,"BehaviorAsset");
catch
    behaviour=project.createAsset(assetPath,"BehaviorAsset");
end
platform=behaviour.setPlatform("SimulinkPlatform");
platform.FileName=string(modelPath);
end

function configureTimedRoute(scenario,actor,plan)
distance=norm(plan.Waypoints(end,:)-plan.Waypoints(1,:));
duration=max(plan.Times(end)-plan.Times(1),eps);
speed=distance/duration;
initial=actor.InitialPoint;
if distance>1e-3
    initial.HasTime=true; initial.Time=plan.Times(1);
    final=initial.Route.addPoint(plan.Waypoints(end,:));
    final.HasTime=true; final.Time=plan.Times(end);
end
initialPhase=scenario.PhaseLogic.initialPhaseForActor(actor);
speedAction=initialPhase.findActions("ChangeSpeedAction");
speedAction.Speed=speed; speedAction.SpeedReference="absolute";
end

function name=actorName(classID,actorID)
labels=["Unknown","Bus","Truck","AutoRickshaw","Motorcycle", ...
    "Bicycle","Pedestrian","Pushcart","Cattle","Obstacle"];
index=min(max(double(classID),1),numel(labels));
name=labels(index)+" "+string(actorID);
end

function color=actorColor(classID)
palette=["white","#ff8c00","red","yellow","cyan","green", ...
    "white","magenta","#8b4513","red"];
index=min(max(double(classID),1),numel(palette));
color=palette(index);
end

function projectFolder=defaultProjectFolder()
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
projectFolder=string(fullfile(root,"Scenarios","RoadRunnerProject"));
end

function installationFolder=defaultRoadRunnerInstallation()
installationFolder="C:\Program Files\RoadRunner "+string(matlabRelease.Release)+ ...
    "\bin\win64";
if ~isfolder(installationFolder)
    error("Roadsense:RoadRunner:InstallationNotFound", ...
        "RoadRunner installation was not found at %s.",installationFolder);
end
end
