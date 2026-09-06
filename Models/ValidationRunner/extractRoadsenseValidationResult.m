function result=extractRoadsenseValidationResult(simulationOutput,scenarioID,wallTime,stopTime)
%EXTRACTROADSENSEVALIDATIONRESULT Convert logged buses into compact evidence.
arguments
    simulationOutput (1,1) Simulink.SimulationOutput
    scenarioID (1,1) double
    wallTime (1,1) double = NaN
    stopTime (1,1) double = NaN
end
logs=simulationOutput.logsout;
required=["EgoState","LocalPlan","SafeVehicleControl","SafetyStatus", ...
    "IntegrationStatus","SensorStatus","EvaluationStatus","PlannerStatus", ...
    "TrackingStatus","DynamicsStatus","BehaviourCommand"];
signals=struct;
for index=1:numel(required)
    element=logs.getElement(required(index));
    if isempty(element)
        error("Roadsense:Validation:MissingLog","Required signal %s was not logged.",required(index));
    end
    signals.(required(index))=element.Values;
end
result=extractRoadsenseValidationSignals(signals,scenarioID,wallTime,stopTime);
end
