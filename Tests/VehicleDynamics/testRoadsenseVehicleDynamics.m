classdef testRoadsenseVehicleDynamics < matlab.unittest.TestCase
    %TESTROADSENSEVEHICLEDYNAMICS Physical response and model tests.
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
        function resetLoadsInitialStateExactly(testCase)
            [control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs("straight");
            plant=RoadsenseVehicleDynamicsSystem; [out{1:24}]=plant(control,initial,road,true);
            testCase.verifyEqual(out{2},initial.Position,"AbsTol",1e-12);
            testCase.verifyEqual(out{3},initial.Velocity,"AbsTol",1e-12);
            testCase.verifyTrue(out{22}); testCase.verifyTrue(out{10});
        end

        function accelerationProducesForwardMotion(testCase)
            [control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs("straight");
            [out,~]=runPlant(control,initial,road,250);
            testCase.verifyGreaterThan(out{12},single(10));
            testCase.verifyGreaterThan(out{2}(1),35);
            testCase.verifyLessThan(abs(out{2}(2)),0.05);
            testCase.verifyLessThan(abs(out{13}),single(0.02));
        end

        function steeringProducesStableLeftTurn(testCase)
            [control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs("turn");
            [out,history]=runPlant(control,initial,road,150);
            testCase.verifyGreaterThan(out{5},0.4);
            testCase.verifyGreaterThan(out{2}(2),3);
            testCase.verifyLessThan(max(abs(history.Sideslip)),0.25);
            testCase.verifyLessThanOrEqual(max(history.Utilization),1.01);
        end

        function lowFrictionLimitsEmergencyDeceleration(testCase)
            [control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs("low_friction");
            [out,history]=runPlant(control,initial,road,80);
            testCase.verifyGreaterThanOrEqual(min(history.LongitudinalAcceleration),-2.05);
            testCase.verifyTrue(any(history.Saturated));
            testCase.verifyLessThan(out{12},single(15));
        end

        function uphillRoadReducesAcceleration(testCase)
            [flatControl,flatInitial,flatRoad]=createSyntheticRoadsenseVehicleDynamicsInputs("straight");
            [hillControl,hillInitial,hillRoad]=createSyntheticRoadsenseVehicleDynamicsInputs("uphill");
            [flat,~]=runPlant(flatControl,flatInitial,flatRoad,150);
            [hill,~]=runPlant(hillControl,hillInitial,hillRoad,150);
            testCase.verifyLessThan(hill{12},flat{12}-single(2));
            testCase.verifyGreaterThan(hill{2}(3),0);
        end

        function invalidCommandUsesPhysicalFailSafeBrake(testCase)
            [control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs("invalid");
            [out,history]=runPlant(control,initial,road,30);
            testCase.verifyLessThan(out{12},single(5));
            testCase.verifyLessThan(min(history.LongitudinalAcceleration),-1);
            testCase.verifyTrue(out{10});
        end

        function controllerAndDynamicPlantCloseTheLoop(testCase)
            [ego,plan,status]=createSyntheticRoadsenseControllerInputs("offset");
            initial=ego; [~,~,road]=createSyntheticRoadsenseVehicleDynamicsInputs("straight");
            controller=RoadsenseTrajectoryControllerSystem; plant=RoadsenseVehicleDynamicsSystem;
            initialError=1.2;
            for step=1:201
                ego.Timestamp=plan.Timestamp+(step-1)*0.02; road.Timestamp=ego.Timestamp;
                [command{1:22}]=controller(ego,plan,status);
                control=struct("Timestamp",command{1},"PlanID",command{2}, ...
                    "SteeringAngle",command{3},"SteeringRate",command{4}, ...
                    "AccelerationCommand",command{5},"Throttle",command{6}, ...
                    "Brake",command{7},"TargetSpeed",command{8}, ...
                    "EmergencyStop",command{9},"Valid",command{10});
                [state{1:24}]=plant(control,initial,road,step==1);
                ego=struct("Timestamp",state{1},"Position",state{2}, ...
                    "Velocity",state{3},"Acceleration",state{4},"Yaw",state{5}, ...
                    "Pitch",state{6},"Roll",state{7},"YawRate",state{8}, ...
                    "SteeringAngle",state{9},"Valid",state{10});
            end
            testCase.verifyLessThan(abs(double(command{14})),0.30);
            testCase.verifyLessThan(abs(double(command{14})),initialError/2);
            testCase.verifyTrue(state{24});
        end

        function generatedModelCompilesAndRuns(testCase)
            modelPath=createRoadsenseVehicleDynamicsModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName+"/EgoState","OutDataTypeStr"), ...
                'Bus: RsEgoStateBus');
            testCase.verifyEqual(get_param(modelName+"/DynamicsStatus","OutDataTypeStr"), ...
                'Bus: RsVehicleDynamicsStatusBus');
            sim(modelName,"StopTime","0.04");
            clear cleanup; close_system(modelName,0);
        end
    end
end

function [out,history]=runPlant(control,initial,road,numberSteps)
plant=RoadsenseVehicleDynamicsSystem; history.Sideslip=zeros(numberSteps,1);
history.Utilization=zeros(numberSteps,1); history.LongitudinalAcceleration=zeros(numberSteps,1);
history.Saturated=false(numberSteps,1);
for step=1:numberSteps
    control.Timestamp=(step-1)*0.02; road.Timestamp=control.Timestamp;
    [out{1:24}]=plant(control,initial,road,step==1);
    history.Sideslip(step)=double(out{14}); history.Utilization(step)=double(out{20});
    history.LongitudinalAcceleration(step)=double(out{18}); history.Saturated(step)=out{23};
end
end
