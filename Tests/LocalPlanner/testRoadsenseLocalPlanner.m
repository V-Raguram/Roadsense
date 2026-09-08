classdef testRoadsenseLocalPlanner < matlab.unittest.TestCase
    %TESTROADSENSELOCALPLANNER Sampling, safety, feasibility, and model tests.
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
        function cruiseProducesSmoothWorldFramePlan(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            testCase.verifyEqual(out{3},uint16(61));
            testCase.verifyEqual(out{13},uint16(21));
            testCase.verifyGreaterThan(out{14},uint16(0));
            testCase.verifyEqual(out{5}(1,:),single([100 50]),"AbsTol",single(0.05));
            testCase.verifyGreaterThanOrEqual(min(out{7}),single(0));
            testCase.verifyLessThanOrEqual(max(abs(out{9})),single(0.22+1e-3));
            jerk=diff(double(out{8}))/0.1;
            testCase.verifyLessThanOrEqual(max(abs(jerk)),2.501);
        end

        function staticObstacleSelectsLateralEscape(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("static_avoid");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            localY=double(out{5}(:,2))-double(inputs{1}.Position(2));
            testCase.verifyGreaterThan(max(abs(localY)),2.5);
            testCase.verifyGreaterThan(out{14},uint16(0));
        end

        function avoidanceCommandStartsEarlyLateralManeuver(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            inputs{5}.Mode=RoadsenseTypes.BehaviourMode.AvoidObstacle;
            inputs{5}.TargetSpeed=single(4);
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            localY=double(out{5}(:,2))-double(inputs{1}.Position(2));
            testCase.verifyGreaterThan(max(abs(localY)),3.0);
            firstSide=sign(localY(end));
            y=-25+(0.5:199.5)*0.25;
            inputs{2}.CombinedCost(y*firstSide>0,:)=single(0.90);
            [second{1:26}]=planner(inputs{:});
            secondY=double(second{5}(:,2))-double(inputs{1}.Position(2));
            testCase.verifyEqual(sign(secondY(end)),firstSide);

            tracks=inputs{3}; tracks.Count=uint16(1); tracks.ValidMask(1)=true;
            tracks.TrackIDs(1)=uint32(802); tracks.ExistenceProbabilities(1)=single(0.95);
            tracks.Positions(1,:)=single([35 firstSide 0.9]);
            tracks.Velocities(1,:)=single([-6 0 0]);
            tracks.Dimensions(1,:)=single([2.8 1.4 1.8]);
            inputs{3}=tracks; inputs{2}.CombinedCost(:)=single(0.075);
            [directed{1:26}]=planner(inputs{:});
            directedY=double(directed{5}(:,2))-double(inputs{1}.Position(2));
            testCase.verifyEqual(sign(directedY(end)),-firstSide);
        end

        function displacedEgoKeepsReplanSpatiallyContinuous(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            inputs{1}.Position(2)=inputs{1}.Position(2)+single(1.5);
            inputs{5}.Mode=RoadsenseTypes.BehaviourMode.AvoidObstacle;
            inputs{5}.TargetSpeed=single(3);
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            testCase.verifyLessThan(norm(double(out{5}(1,:)-inputs{1}.Position(1:2).')),0.30);
            testCase.verifyGreaterThan(max(abs(double(out{5}(:,2)-inputs{1}.Position(2)))),1.0);
        end

        function avoidanceSideIsAwayFromClosingVehicle(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            inputs{5}.Mode=RoadsenseTypes.BehaviourMode.AvoidObstacle;
            inputs{5}.TargetSpeed=single(4);
            tracks=inputs{3}; tracks.Count=uint16(1); tracks.ValidMask(1)=true;
            tracks.TrackIDs(1)=uint32(801); tracks.ClassIDs(1)=uint8(4);
            tracks.ExistenceProbabilities(1)=single(0.95);
            tracks.Positions(1,:)=single([30 1 0.9]);
            tracks.Velocities(1,:)=single([-6 0 0]);
            tracks.Dimensions(1,:)=single([2.8 1.4 1.8]);
            inputs{3}=tracks;
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            localY=double(out{5}(:,2))-double(inputs{1}.Position(2));
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            testCase.verifyLessThan(localY(end),-3.0);
        end

        function crossingPedestrianCausesYieldingStop(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("crossing_pedestrian");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16}); testCase.verifyFalse(out{15});
            testCase.verifyLessThan(out{7}(end),single(0.05));
            testCase.verifyLessThan(double(out{5}(end,1)-inputs{1}.Position(1)),10);
            testCase.verifyGreaterThanOrEqual(out{12},single(0));
        end

        function finiteRouteTerminalStopRemainsJerkFeasible(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            inputs{1}.Velocity=single([3;0;0]);
            inputs{1}.Acceleration=single([0;0;0]);
            inputs{5}.TargetSpeed=single(4.5);
            count=uint16(41); x=single(linspace(0,8,double(count)).');
            inputs{6}.Count=count;
            inputs{6}.Positions(:)=single(0);
            inputs{6}.Positions(1:double(count),1)=x;
            inputs{6}.Yaws(:)=single(0);
            inputs{6}.RecommendedSpeeds(:)=single(0);
            inputs{6}.RecommendedSpeeds(1:double(count))=single(4.5);
            inputs{6}.ValidMask(:)=false;
            inputs{6}.ValidMask(1:double(count))=true;
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16});
            testCase.verifyFalse(out{15});
            testCase.verifyGreaterThan(out{14},uint16(0));
            jerk=diff(double(out{8}))/0.1;
            testCase.verifyLessThanOrEqual(max(abs(jerk)),2.501);
            testCase.verifyLessThan(out{7}(end),single(0.15));
        end

        function emergencyCommandAlwaysUsesFallback(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("emergency");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{15}); testCase.verifyTrue(out{16});
            testCase.verifyEqual(out{19},uint8(6));
            testCase.verifyGreaterThanOrEqual(min(out{7}),single(0));
            testCase.verifyLessThan(out{7}(end),single(0.05));
        end

        function fullyBlockedCorridorUsesMinimumRiskFallback(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("blocked");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyEqual(out{14},uint16(0));
            testCase.verifyTrue(out{15}); testCase.verifyTrue(out{16});
            testCase.verifyTrue(out{26});
        end

        function farUncertainTrackDoesNotBlockWholeRoad(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            tracks=inputs{3}; predictions=inputs{4};
            tracks.Count=uint16(1); tracks.ValidMask(1)=true;
            tracks.TrackIDs(1)=uint32(77); tracks.ExistenceProbabilities(1)=single(0.9);
            tracks.Positions(1,:)=single([20 15 0]);
            tracks.Dimensions(1,:)=single([4.5 2 1.6]);
            predictions.Count=uint16(1); predictions.ValidMask(1)=true;
            predictions.TrackIDs(1)=uint32(77); predictions.NumModes(1)=uint8(1);
            predictions.ModeProbabilities(1,1)=single(0.9);
            predictions.Positions(1,:,1,1)=single(20);
            predictions.Positions(1,:,1,2)=single(15);
            for step=1:16
                predictions.PositionCovariances(:,:,step,1,1)=single(eye(2)*1e5);
            end
            inputs{3}=tracks; inputs{4}=predictions;
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16});
            testCase.verifyFalse(out{15});
            testCase.verifyGreaterThan(out{14},uint16(0));
        end

        function followingClearanceStillAllowsLateralPassing(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("cruise");
            inputs{1}.Velocity(:)=single(0);
            inputs{1}.Acceleration=single([-6;0;0]);
            inputs{5}.Mode=RoadsenseTypes.BehaviourMode.Yield;
            inputs{5}.TargetSpeed=single(0);
            inputs{5}.DesiredClearance=single(2.0);
            tracks=inputs{3}; predictions=inputs{4};
            tracks.Count=uint16(1); tracks.ValidMask(1)=true;
            tracks.TrackIDs(1)=uint32(406); tracks.ExistenceProbabilities(1)=single(0.95);
            tracks.Positions(1,:)=single([10 -1 0]);
            tracks.Dimensions(1,:)=single([1.8 0.65 1.6]);
            predictions.Count=uint16(1); predictions.ValidMask(1)=true;
            predictions.TrackIDs(1)=uint32(406); predictions.NumModes(1)=uint8(1);
            predictions.ModeProbabilities(1,1)=single(0.9);
            predictions.Positions(1,:,1,1)=single(10);
            predictions.Positions(1,:,1,2)=single(-1);
            for step=1:16
                predictions.PositionCovariances(:,:,step,1,1)=single(eye(2)*1e5);
            end
            inputs{3}=tracks; inputs{4}=predictions;
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyTrue(out{16});
            testCase.verifyFalse(out{15});
            testCase.verifyGreaterThan(out{14},uint16(0));
            testCase.verifyLessThan(max(abs(double(out{5}(:,2)-inputs{1}.Position(2)))),0.1);
        end

        function invalidReferenceIsRejected(testCase)
            [inputs{1:6}]=createSyntheticRoadsensePlannerInputs("invalid");
            planner=RoadsenseLocalPlannerSystem; [out{1:26}]=planner(inputs{:});
            testCase.verifyFalse(out{16}); testCase.verifyEqual(out{19},uint8(4));
            testCase.verifyTrue(out{25}); testCase.verifyTrue(out{26});
        end

        function generatedModelCompilesAndRuns(testCase)
            modelPath=createRoadsenseLocalPlannerModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName+"/LocalPlan","OutDataTypeStr"), ...
                'Bus: RsLocalPlanBus');
            testCase.verifyEqual(get_param(modelName+"/PlannerStatus","OutDataTypeStr"), ...
                'Bus: RsPlannerStatusBus');
            sim(modelName,"StopTime","0.1");
            clear cleanup; close_system(modelName,0);
        end
    end
end
