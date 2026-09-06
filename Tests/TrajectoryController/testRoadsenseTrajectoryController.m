classdef testRoadsenseTrajectoryController < matlab.unittest.TestCase
    %TESTROADSENSETRAJECTORYCONTROLLER Closed-loop and fail-safe tests.
    properties
        Root
    end
    methods (TestClassSetup)
        function configure(testCase)
            testFile=mfilename("fullpath"); testCase.Root=fileparts(fileparts(fileparts(testFile)));
            addpath(genpath(fullfile(testCase.Root,"Models"))); addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
        end
    end
    methods (Test)
        function alignedPlanProducesBoundedCommands(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("aligned");
            controller=RoadsenseTrajectoryControllerSystem;
            [out{1:22}]=controller(ego,plan,status);
            testCase.verifyTrue(out{10}); testCase.verifyTrue(out{22});
            testCase.verifyFalse(out{9});
            testCase.verifyLessThanOrEqual(abs(out{3}),single(0.55));
            testCase.verifyLessThanOrEqual(abs(out{4}),single(0.70+1e-5));
            testCase.verifyGreaterThanOrEqual(out{6},single(0));
            testCase.verifyGreaterThanOrEqual(out{7},single(0));
            testCase.verifyEqual(out{6}*out{7},single(0),"AbsTol",single(1e-7));
        end

        function offsetErrorCommandsRecoveryDirection(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("offset");
            controller=RoadsenseTrajectoryControllerSystem; [out{1:22}]=controller(ego,plan,status);
            testCase.verifyGreaterThan(out{14},single(1));
            testCase.verifyLessThan(out{3},single(0));
            testCase.verifyTrue(out{19});
        end

        function freshCurvedPlanUsesMeaningfulPreview(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("curved");
            controller=RoadsenseTrajectoryControllerSystem;
            [out{1:22}]=controller(ego,plan,status);
            testCase.verifyGreaterThanOrEqual(out{13},uint16(11));
            testCase.verifyGreaterThan(abs(out{15}),single(1e-3));
        end

        function overspeedRequestsBraking(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("overspeed");
            controller=RoadsenseTrajectoryControllerSystem; [out{1:22}]=controller(ego,plan,status);
            testCase.verifyLessThan(out{16},single(0));
            testCase.verifyLessThan(out{5},single(0));
            testCase.verifyGreaterThan(out{7},single(0));
            testCase.verifyEqual(out{6},single(0));
        end

        function emergencyBypassesNormalSlewForFullBrake(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("emergency");
            controller=RoadsenseTrajectoryControllerSystem; [out{1:22}]=controller(ego,plan,status);
            testCase.verifyTrue(out{9}); testCase.verifyTrue(out{10});
            testCase.verifyEqual(out{5},single(-6));
            testCase.verifyEqual(out{7},single(1)); testCase.verifyEqual(out{6},single(0));
        end

        function staleFallbackFlagDoesNotOverrideCurrentSafePlannerStatus(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("staleFallback");
            controller=RoadsenseTrajectoryControllerSystem; [out{1:22}]=controller(ego,plan,status);
            testCase.verifyTrue(plan.EmergencyFallback);
            testCase.verifyFalse(status.EmergencyRequested);
            testCase.verifyFalse(out{9});
            testCase.verifyTrue(out{10});
            testCase.verifyGreaterThanOrEqual(out{5},single(-0.10));
        end

        function expiredOrInvalidPlanFailsSafe(testCase)
            scenarios=["expired","invalid"];
            for index=1:numel(scenarios)
                [ego,plan,status]=createSyntheticRoadsenseControllerInputs(scenarios(index));
                controller=RoadsenseTrajectoryControllerSystem; [out{1:22}]=controller(ego,plan,status);
                testCase.verifyTrue(out{9}); testCase.verifyFalse(out{10});
                testCase.verifyEqual(out{5},single(-6)); testCase.verifyEqual(out{7},single(1));
                if scenarios(index)=="expired"; testCase.verifyTrue(out{21}); end
            end
        end

        function closedLoopReducesCrossTrackError(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("offset");
            controller=RoadsenseTrajectoryControllerSystem; dt=0.02;
            initialError=abs(ego.Position(2)-double(plan.Positions(1,2)));
            for step=1:200
                ego.Timestamp=plan.Timestamp+(step-1)*dt;
                [out{1:22}]=controller(ego,plan,status);
                speed=max(0,norm(ego.Velocity(1:2))+double(out{5})*dt);
                ego.Yaw=wrapLocal(ego.Yaw+speed/2.8*tan(double(out{3}))*dt);
                ego.Position(1:2)=ego.Position(1:2)+speed*[cos(ego.Yaw);sin(ego.Yaw)]*dt;
                ego.Velocity(1:2)=speed*[cos(ego.Yaw);sin(ego.Yaw)];
                ego.Acceleration(1:2)=double(out{5})*[cos(ego.Yaw);sin(ego.Yaw)];
                ego.SteeringAngle=double(out{3});
            end
            finalError=abs(double(out{14}));
            testCase.verifyLessThan(finalError,0.45);
            testCase.verifyLessThan(finalError,initialError/2);
        end

        function generatedModelCompilesAndRuns(testCase)
            modelPath=createRoadsenseTrajectoryControllerModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName+"/VehicleControl","OutDataTypeStr"), ...
                'Bus: RsVehicleControlBus');
            testCase.verifyEqual(get_param(modelName+"/TrackingStatus","OutDataTypeStr"), ...
                'Bus: RsTrackingStatusBus');
            sim(modelName,"StopTime","0.04");
            clear cleanup; close_system(modelName,0);
        end
    end
end

function angle=wrapLocal(angle)
angle=mod(angle+pi,2*pi)-pi;
end
