function simulationOutput=runRoadsenseScenarioClosedLoopHarnessDemo(scenarioID,stopTime)
%RUNROADSENSESCENARIOCLOSEDLOOPHARNESSDEMO Run one full closed-loop scenario.
arguments
    scenarioID (1,1) double {mustBeInteger,mustBeInRange(scenarioID,1,5)} = 1
    stopTime (1,1) double {mustBePositive} = 2
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
modelPath=fullfile(componentDir,"Roadsense_ScenarioClosedLoopHarness.slx");
if ~isfile(modelPath); createRoadsenseScenarioClosedLoopHarnessModel(); end
dataset=createRoadsenseScenarioHarnessInputs(scenarioID,stopTime);
input=Simulink.SimulationInput("Roadsense_ScenarioClosedLoopHarness");
input=input.setExternalInput(dataset);
input=input.setModelParameter("StopTime",string(stopTime), ...
    "SaveOutput","on","OutputSaveName","yout","SaveFormat","Dataset");
simulationOutput=sim(input);
end
