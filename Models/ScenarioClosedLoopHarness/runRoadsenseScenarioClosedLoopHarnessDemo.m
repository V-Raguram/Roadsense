function simulationOutput=runRoadsenseScenarioClosedLoopHarnessDemo(scenarioID,stopTime,options)
%RUNROADSENSESCENARIOCLOSEDLOOPHARNESSDEMO Run one full closed-loop scenario.
arguments
    scenarioID (1,1) double {mustBeInteger,mustBeInRange(scenarioID,1,6)} = 1
    stopTime (1,1) double {mustBePositive} = 2
    options.ParameterOverrides (1,1) struct = struct
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
overrideNames=fieldnames(options.ParameterOverrides);
for index=1:numel(overrideNames)
    name=overrideNames{index};
    input=input.setVariable(name,options.ParameterOverrides.(name));
end
simulationOutput=sim(input);
end
