function session=runRoadsenseRoadRunnerScenario(scenarioID,options)
%RUNROADSENSEROADRUNNERSCENARIO Start one complete live 3D co-simulation.
arguments
    scenarioID (1,1) double {mustBeInteger,mustBeMember(scenarioID,1:5)} = 1
    options.Pacing (1,1) double {mustBePositive} = 1
    options.RebuildProject (1,1) logical = false
    options.NoDisplay (1,1) logical = false
    options.IsBlocking (1,1) logical = false
end
root=fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(root,"Models")));
addpath(fullfile(root,"Data"));
dictionaryPath=fullfile(root,"Data","Roadsense_Data.sldd");
if ~isfile(dictionaryPath)
    createRoadsenseDataDictionary(dictionaryPath);
end
projectFolder=fullfile(root,"Scenarios","RoadRunnerProject");
manifestFile=fullfile(root,"Scenarios","RoadRunner", ...
    "roadrunner_scene_manifest.csv");
if options.RebuildProject || ~isfile(manifestFile) || ...
        ~isfolder(fullfile(projectFolder,"Project"))
    createRoadsenseRoadRunnerScenarios(projectFolder);
end
manifest=readtable(manifestFile,TextType="string");
row=manifest(manifest.ScenarioID==scenarioID,:);
if height(row)~=1 || ~isfile(fullfile(projectFolder,"Scenarios",row.ScenarioFile))
    manifest=createRoadsenseRoadRunnerScenarios(projectFolder);
    row=manifest(manifest.ScenarioID==scenarioID,:);
end

modelName="Roadsense_RoadRunnerIntegration";
modelPath=fullfile(root,"Models","RoadRunnerIntegration",modelName+".slx");
if ~isfile(modelPath); createRoadsenseRoadRunnerIntegrationModel(); end
Simulink.ActorSimulation.load('BusActorPose');
if ~bdIsLoaded(modelName); load_system(modelPath); end
modelWorkspace=get_param(modelName,"ModelWorkspace");
assignin(modelWorkspace,"RsRoadRunnerScenarioID",uint8(scenarioID));
% Keep the generated SLX in its normal network backend on disk. R2025b can
% then cold-load the complete hierarchy reliably, while this unsaved model
% selection uses the deterministic colour camera during the live scenario.
setRoadsenseSemanticInferenceMode("syntheticColor",Persist=false);

installationFolder="C:\Program Files\RoadRunner "+ ...
    string(matlabRelease.Release)+"\bin\win64";
rrApp=roadrunner(ProjectFolder=projectFolder, ...
    InstallationFolder=installationFolder,NoDisplay=options.NoDisplay);
openScenario(rrApp,row.ScenarioFile);
simulateScenario(rrApp,Pacing=options.Pacing, ...
    IsBlocking=options.IsBlocking,EnableLogging=true);

session=struct("App",rrApp,"ScenarioID",uint8(scenarioID), ...
    "Name",row.Name,"ScenarioFile",row.ScenarioFile, ...
    "Model",modelName,"ProjectFolder",string(projectFolder));
assignin("base","RoadsenseRoadRunnerSession",session);
fprintf("Roadsense RoadRunner stage %d started: %s\n",scenarioID,row.Name);
fprintf("The 3D window is live; the blue ego vehicle is controlled by %s.\n",modelName);
fprintf("Native sensor blocks active: vision ID 1, radar ID 2, LiDAR ID 3.\n");
end
