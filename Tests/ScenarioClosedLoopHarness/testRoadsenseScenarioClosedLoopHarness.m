classdef testRoadsenseScenarioClosedLoopHarness < matlab.unittest.TestCase
    %TESTROADSENSESCENARIOCLOSEDLOOPHARNESS End-to-end harness and scoring tests.
    properties
        Root
    end
    methods (TestClassSetup)
        function configure(testCase)
            testFile=mfilename("fullpath");
            testCase.Root=fileparts(fileparts(fileparts(testFile)));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
        end
    end
    methods (Test)
        function successfulFinishedRunPassesAcceptance(testCase)
            data=evaluationInputs(1,28);
            data.IntegrationStatus.GoalReached=true;
            data.TruthActors.ValidMask(:)=false; data.TruthActors.Count=uint16(0);
            evaluator=RoadsenseScenarioEvaluationSystem;
            [out{1:18}]=evaluator(data.EgoState,data.LocalPlan,data.SafeControl, ...
                data.SafetyStatus,data.IntegrationStatus,data.PlannerStatus, ...
                data.TrackingStatus,data.DynamicsStatus,data.TruthActors, ...
                data.ScenarioStatus,data.SensorStatus);
            testCase.verifyTrue(out{6}); testCase.verifyTrue(out{7});
            testCase.verifyEqual(out{15},single(1)); testCase.verifyTrue(out{17});
            testCase.verifyTrue(out{18});
        end

        function truthFootprintCollisionIsLatched(testCase)
            data=evaluationInputs(1,28); data.IntegrationStatus.GoalReached=true;
            data.TruthActors.Count=uint16(1); data.TruthActors.ValidMask(:)=false;
            data.TruthActors.ValidMask(1)=true;
            data.TruthActors.Positions(1,:)=data.EgoState.Position(:).';
            data.TruthActors.Dimensions(1,:)=single([4.4 1.8 1.6]);
            evaluator=RoadsenseScenarioEvaluationSystem;
            [out{1:18}]=evaluator(data.EgoState,data.LocalPlan,data.SafeControl, ...
                data.SafetyStatus,data.IntegrationStatus,data.PlannerStatus, ...
                data.TrackingStatus,data.DynamicsStatus,data.TruthActors, ...
                data.ScenarioStatus,data.SensorStatus);
            testCase.verifyTrue(out{5}); testCase.verifyLessThanOrEqual(out{4},single(0));
            testCase.verifyFalse(out{17});
        end

        function replansLatencyAndInterventionsAreCounted(testCase)
            data=evaluationInputs(3,2); evaluator=RoadsenseScenarioEvaluationSystem;
            data.PlannerStatus.PlanID=uint32(41); data.PlannerStatus.ExecutionTime=single(0.035);
            call(evaluator,data);
            call(evaluator,data);
            data.PlannerStatus.PlanID=uint32(42); data.PlannerStatus.ExecutionTime=single(0.052);
            data.SafetyStatus.OverrideActive=true;
            out=call(evaluator,data);
            testCase.verifyEqual(out{8},uint32(2));
            testCase.verifyEqual(out{9},single(0.052),"AbsTol",single(1e-6));
            testCase.verifyEqual(out{10},single(0.052),"AbsTol",single(1e-6));
            testCase.verifyEqual(out{14},uint32(1));
        end

        function dropoutBitsAndScenarioResetAreDeterministic(testCase)
            first=evaluationInputs(1,1); first.SensorStatus.CameraDropout=true;
            first.SensorStatus.RadarDropout=true;
            evaluator=RoadsenseScenarioEvaluationSystem; out=call(evaluator,first);
            testCase.verifyEqual(out{16},uint8(5));
            first.TruthActors.Count=uint16(1); first.TruthActors.ValidMask(1)=true;
            first.TruthActors.Positions(1,:)=first.EgoState.Position(:).';
            out=call(evaluator,first); testCase.verifyTrue(out{5});
            second=evaluationInputs(2,1); second.TruthActors.ValidMask(:)=false;
            second.TruthActors.Count=uint16(0); out=call(evaluator,second);
            testCase.verifyFalse(out{5}); testCase.verifyEqual(out{8},uint32(1));
        end

        function startupFailSafeDoesNotPolluteComfortMetrics(testCase)
            data=evaluationInputs(1,0.1); evaluator=RoadsenseScenarioEvaluationSystem;
            data.IntegrationStatus.PipelineReady=false;
            data.SafeControl.AccelerationCommand=single(-6);
            data.SafeControl.EmergencyStop=true; data.SafetyStatus.OverrideActive=true;
            call(evaluator,data);
            data.SafeControl.Timestamp=data.SafeControl.Timestamp+0.02;
            data.SafeControl.AccelerationCommand=single(0);
            out=call(evaluator,data);
            testCase.verifyEqual(out{12},single(0));
            testCase.verifyEqual(out{14},uint32(0));
        end

        function safetyOverrideBreaksComfortJerkDerivative(testCase)
            data=evaluationInputs(1,1); evaluator=RoadsenseScenarioEvaluationSystem;
            data.SafeControl.AccelerationCommand=single(0); call(evaluator,data);
            data.SafeControl.Timestamp=1.02; data.SafetyStatus.OverrideActive=true;
            data.SafeControl.EmergencyStop=true;
            data.SafeControl.AccelerationCommand=single(-6); call(evaluator,data);
            data.SafeControl.Timestamp=1.04; data.SafetyStatus.OverrideActive=false;
            data.SafeControl.EmergencyStop=false;
            data.SafeControl.AccelerationCommand=single(0); out=call(evaluator,data);
            testCase.verifyEqual(out{12},single(0));
            testCase.verifyEqual(out{14},uint32(1));
        end

        function generatedModelHasOrganizedTruthIsolatedHierarchy(testCase)
            modelPath=createRoadsenseScenarioClosedLoopHarnessModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            testCase.verifyEqual(get_param(modelName+"/Roadsense Autonomous Driving Stack", ...
                "ModelName"),'Roadsense_ClosedLoopIntegration');
            testCase.verifyEqual(get_param(modelName+"/Simulated Indian Road Environment", ...
                "ContentPreviewEnabled"),'off');
            testCase.verifyEqual(get_param(modelName+"/Truth-Isolated Acceptance Evaluation", ...
                "ContentPreviewEnabled"),'off');
            bridge=modelName+"/Simulated Indian Road Environment/"+ ...
                "One-Tick Sensor Latency and Radar Rate Bridge";
            delays=find_system(bridge,"SearchDepth",1,"BlockType","UnitDelay");
            rates=find_system(bridge,"SearchDepth",1,"BlockType","RateTransition");
            testCase.verifyNumElements(delays,15); testCase.verifyNumElements(rates,8);
            outports=find_system(modelName,"SearchDepth",1,"BlockType","Outport");
            testCase.verifyNumElements(outports,9);
            testCase.verifyEqual(get_param(modelName+"/EvaluationStatus", ...
                "OutDataTypeStr"),'Bus: RsScenarioEvaluationBus');
            set_param(modelName,"SimulationCommand","update");
            clear cleanup; close_system(modelName,0);
        end

        function namedScenarioInputsPreserveFixedTypes(testCase)
            dataset=createRoadsenseScenarioHarnessInputs(5,0.2);
            testCase.verifyEqual(dataset.numElements,2);
            testCase.verifyClass(dataset{1}.Data,"uint8");
            testCase.verifyClass(dataset{2}.Data,"logical");
            testCase.verifyEqual(dataset{1}.Data,uint8([5;5]));
        end
    end
end

function data=evaluationInputs(scenarioID,time)
integration=createSyntheticRoadsenseIntegrationInputs();
data.EgoState=integration{1}; data.LocalPlan=integration{8};
data.PlannerStatus=integration{9}; data.TrackingStatus=integration{11};
data.SafetyStatus=integration{12}; data.SafeControl=integration{13};
data.DynamicsStatus=integration{14};
[~,data.TruthActors,data.ScenarioStatus]=RoadsenseScenarioCatalog(scenarioID,time);
data.EgoState.Timestamp=time; data.SafeControl.Timestamp=time;
data.EgoState.Valid=true; data.DynamicsStatus.Valid=true;
data.IntegrationStatus=struct("Timestamp",time,"CycleID",uint32(1), ...
    "PerceptionValid",true,"FusionValid",true,"PredictionValid",true, ...
    "MapValid",true,"BehaviourValid",true,"PlanValid",true, ...
    "TrackingValid",true,"SafetyValid",true,"VehicleValid",true, ...
    "PipelineReady",true,"SafetyOverride",false,"EmergencyStop",false, ...
    "GoalReached",false,"Valid",true);
data.SensorStatus=struct("Timestamp",time,"ScenarioID",uint16(scenarioID), ...
    "CameraVisibleCount",uint16(1),"LidarVisibleCount",uint16(1), ...
    "RadarVisibleCount",uint16(1),"OccludedCount",uint16(0), ...
    "CameraDropout",false,"LidarDropout",false,"RadarDropout",false,"Valid",true);
end

function output=call(evaluator,data)
output=cell(1,18);
[output{:}]=evaluator(data.EgoState,data.LocalPlan,data.SafeControl, ...
    data.SafetyStatus,data.IntegrationStatus,data.PlannerStatus, ...
    data.TrackingStatus,data.DynamicsStatus,data.TruthActors, ...
    data.ScenarioStatus,data.SensorStatus);
end
