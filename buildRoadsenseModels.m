function report=buildRoadsenseModels()
%BUILDROADSENSEMODELS Regenerate and compile every Roadsense Simulink model.
%   REPORT=BUILDROADSENSEMODELS rebuilds the component models in dependency
%   order from their MATLAB generator files, applies the shared Roadsense
%   presentation style, and runs a Simulink diagram update on each model.
%   The final model is the RoadRunner actor-behavior co-simulation wrapper.

project=setupRoadsense;
generators=[ ...
    "createRoadsenseSemanticPerceptionModel"; ...
    "createRoadsenseLidarPerceptionModel"; ...
    "createRoadsenseSensorFusionModel"; ...
    "createRoadsenseMotionPredictionModel"; ...
    "createRoadsenseSemanticMapFusionModel"; ...
    "createRoadsenseBehaviourPlannerModel"; ...
    "createRoadsenseLocalPlannerModel"; ...
    "createRoadsenseTrajectoryControllerModel"; ...
    "createRoadsenseSafetySupervisorModel"; ...
    "createRoadsenseVehicleDynamicsModel"; ...
    "createRoadsenseScenarioLibraryModel"; ...
    "createRoadsenseScenarioSensorSourceModel"; ...
    "createRoadsenseScenarioAdaptersModel"; ...
    "createRoadsenseClosedLoopIntegrationModel"; ...
    "createRoadsenseScenarioClosedLoopHarnessModel"; ...
    "createRoadsenseRoadRunnerIntegrationModel"];

modelNames=strings(size(generators));
compileSeconds=zeros(size(generators));
passed=false(size(generators));
messages=strings(size(generators));

for index=1:numel(generators)
    started=tic;
    try
        modelPath=feval(generators(index));
        [~,modelName]=fileparts(modelPath);
        modelNames(index)=modelName;
        load_system(modelPath);
        cleanup=onCleanup(@() closeLoadedModel(modelName));
        set_param(modelName,"SimulationCommand","update");
        passed(index)=true;
        messages(index)="Compiled";
        clear cleanup
        closeLoadedModel(modelName);
    catch exception
        messages(index)=string(exception.message);
        closeLoadedModel(modelNames(index));
    end
    compileSeconds(index)=toc(started);
end

report=table(generators,modelNames,passed,compileSeconds,messages, ...
    VariableNames=["Generator","Model","Passed","Seconds","Message"]);
disp(report(:,["Model","Passed","Seconds","Message"]));

if ~all(passed)
    failed=join(generators(~passed),", ");
    error("Roadsense:ModelBuildFailed", ...
        "The following Roadsense model builds failed: %s",failed);
end

fprintf("All %d Roadsense Simulink models regenerated and compiled from %s.\n", ...
    height(report),project.Root);
end

function closeLoadedModel(modelName)
if strlength(modelName)>0 && bdIsLoaded(modelName)
    close_system(modelName,0);
end
end
