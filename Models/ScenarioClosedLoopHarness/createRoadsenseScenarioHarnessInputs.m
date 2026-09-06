function dataset=createRoadsenseScenarioHarnessInputs(scenarioID,stopTime)
%CREATEROADSENSESCENARIOHARNESSINPUTS Create named root inputs for simulation.
arguments
    scenarioID (1,1) double {mustBeInteger,mustBeInRange(scenarioID,1,6)} = 1
    stopTime (1,1) double {mustBePositive} = 2
end
time=[0;stopTime];
selection=timeseries(uint8([scenarioID;scenarioID]),time);
selection.Name="ScenarioSelect";
reset=timeseries(false(2,1),time); reset.Name="ManualReset";
dataset=Simulink.SimulationData.Dataset;
dataset=dataset.addElement(selection,"ScenarioSelect");
dataset=dataset.addElement(reset,"ManualReset");
end
