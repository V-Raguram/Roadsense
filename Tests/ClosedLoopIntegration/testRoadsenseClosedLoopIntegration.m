classdef testRoadsenseClosedLoopIntegration < matlab.unittest.TestCase
    %TESTROADSENSECLOSEDLOOPINTEGRATION Top-level scheduling and health tests.
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
        function healthyPipelineReportsReady(testCase)
            inputs=createSyntheticRoadsenseIntegrationInputs();
            monitor=RoadsenseIntegrationMonitorSystem;
            [out{1:16}]=monitor(inputs{:});
            testCase.verifyEqual(out{2},uint32(1));
            testCase.verifyTrue(out{3});
            testCase.verifyTrue(out{12});
            testCase.verifyFalse(out{13});
            testCase.verifyFalse(out{14});
            testCase.verifyFalse(out{15});
            testCase.verifyTrue(out{16});
            [~,cycle]=monitor(inputs{:});
            testCase.verifyEqual(cycle,uint32(2));
        end

        function monitorLocalizesInvalidPrediction(testCase)
            inputs=createSyntheticRoadsenseIntegrationInputs();
            inputs{5}.Valid=false;
            monitor=RoadsenseIntegrationMonitorSystem;
            [out{1:16}]=monitor(inputs{:});
            testCase.verifyTrue(out{3});
            testCase.verifyTrue(out{4});
            testCase.verifyFalse(out{5});
            testCase.verifyFalse(out{12});
            testCase.verifyTrue(out{16});
        end

        function monitorExposesOverrideEmergencyAndGoal(testCase)
            inputs=createSyntheticRoadsenseIntegrationInputs();
            inputs{12}.OverrideActive=true;
            inputs{13}.EmergencyStop=true;
            inputs{15}.GoalReached=true;
            monitor=RoadsenseIntegrationMonitorSystem;
            [out{1:16}]=monitor(inputs{:});
            testCase.verifyTrue(out{13});
            testCase.verifyTrue(out{14});
            testCase.verifyTrue(out{15});
        end

        function generatedModelHasCompleteReferencedHierarchy(testCase)
            modelPath=createRoadsenseClosedLoopIntegrationModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            references=find_system(modelName,"SearchDepth",1,"BlockType","ModelReference");
            referenceNames=sort(string(get_param(references,"ModelName")));
            expected=sort(["Roadsense_SemanticPerception","Roadsense_LidarPerception", ...
                "Roadsense_SensorFusion","Roadsense_MotionPrediction", ...
                "Roadsense_SemanticMapFusion","Roadsense_BehaviourPlanner", ...
                "Roadsense_LocalPlanner","Roadsense_TrajectoryController", ...
                "Roadsense_SafetySupervisor","Roadsense_VehicleDynamics"]);
            testCase.verifyEqual(referenceNames,expected.');
            transitions=find_system(modelName,"SearchDepth",1,"BlockType","RateTransition");
            testCase.verifyNumElements(transitions,20);
            testCase.verifyEqual(get_param(modelName+"/Tracks 20 to 50 Hz", ...
                "Deterministic"),'off');
            testCase.verifyEqual(get_param(modelName+"/One Control Tick Actuator Delay", ...
                "SampleTime"),'RsTsControl');
            testCase.verifyEqual(get_param(modelName+"/IntegrationStatus", ...
                "OutDataTypeStr"),'Bus: RsIntegrationStatusBus');
            gotos=find_system(modelName,"SearchDepth",1,"BlockType","Goto");
            froms=find_system(modelName,"SearchDepth",1,"BlockType","From");
            testCase.verifyNumElements(gotos,16);
            testCase.verifyNumElements(froms,26);
            testCase.verifyEqual(get_param(modelName+"/Output Source EgoState", ...
                "GotoTag"),'RsTapEgoState');
            screenColour=sscanf(get_param(modelName,"ScreenColor"), ...
                '[%f, %f, %f]').';
            testCase.verifyEqual(screenColour,[0.98 0.98 0.98], ...
                "AbsTol",1e-12);
            set_param(modelName,"SimulationCommand","update");
            sim(modelName,"StopTime","0.02");
            clear cleanup; close_system(modelName,0);
        end

        function safetyCommandIsOnlyPlantActuatorSource(testCase)
            modelPath=createRoadsenseClosedLoopIntegrationModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            delay=get_param(modelName+"/One Control Tick Actuator Delay","PortHandles");
            delayInput=get_param(delay.Inport,"Line");
            source=get_param(get_param(delayInput,"SrcBlockHandle"),"Name");
            testCase.verifyEqual(string(source),"Safety Supervisor");
            plant=get_param(modelName+"/Vehicle Dynamics","PortHandles");
            actuatorLine=get_param(plant.Inport(1),"Line");
            actuatorSource=get_param(get_param(actuatorLine,"SrcBlockHandle"),"Name");
            testCase.verifyEqual(string(actuatorSource),"One Control Tick Actuator Delay");
            clear cleanup; close_system(modelName,0);
        end
    end
end
