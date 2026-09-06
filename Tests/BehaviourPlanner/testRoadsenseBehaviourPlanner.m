classdef testRoadsenseBehaviourPlanner < matlab.unittest.TestCase
    %TESTROADSENSEBEHAVIOURPLANNER Safety assessment and Stateflow logic tests.

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
        function assessmentSeparatesRequiredScenarios(testCase)
            scenarios=["cruise","follow","emergency","yield","creep","avoid", ...
                "cautious","stop","invalid"];
            expectedModes=uint8([1 3 8 4 5 10 2 7 9]);
            for index=1:numel(scenarios)
                [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs(scenarios(index));
                assessment=RoadsenseBehaviourAssessmentSystem;
                [a{1:22}]=assessment(ego,map,tracks,route);
                [mode,~,~]=RoadsenseBehaviourDecisionCore(a{2},a{3},a{4},a{5}, ...
                    a{6},a{7},a{8},a{9},uint8(0),uint16(0),uint16(8), ...
                    uint16(5),uint16(8));
                testCase.verifyEqual(mode,expectedModes(index), ...
                    "Incorrect decision for scenario "+scenarios(index));
            end
        end

        function emergencyUsesTTCAndHoldsAfterClear(testCase)
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("emergency");
            assessment=RoadsenseBehaviourAssessmentSystem;
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyTrue(a{3});
            testCase.verifyLessThan(a{19},single(1.2));
            testCase.verifyNotEqual(bitand(a{22},uint16(2)),uint16(0));
            [mode,hold,clear]=RoadsenseBehaviourDecisionCore(a{2},a{3},a{4},a{5}, ...
                a{6},a{7},a{8},a{9},uint8(1),uint16(0),uint16(8),uint16(5),uint16(8));
            testCase.verifyEqual(mode,uint8(8));
            for tick=1:4
                [mode,hold,clear]=RoadsenseBehaviourDecisionCore(true,false,false,false, ...
                    false,false,false,false,mode,hold,clear,uint16(5),uint16(8));
                testCase.verifyEqual(mode,uint8(8));
            end
            [mode,~,~]=RoadsenseBehaviourDecisionCore(true,false,false,false,false,false, ...
                false,false,mode,hold,clear,uint16(5),uint16(8));
            testCase.verifyEqual(mode,uint8(1));
        end

        function futureRiskRequestsCautionNotPresentCollision(testCase)
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("cruise");
            x=-20+(0.5:319.5)*0.25; y=-25+(0.5:199.5)*0.25;
            mask=abs(y(:))<=1.4 & x>=0 & x<=3;
            map.PredictedRisk(mask)=single(0.99);
            assessment=RoadsenseBehaviourAssessmentSystem;
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyFalse(a{3});
            testCase.verifyTrue(a{9});
            testCase.verifyGreaterThan(a{20},single(0.9));
        end

        function oncomingVehicleSelectsEarlyAvoidance(testCase)
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("cruise");
            tracks.Count=uint16(1); tracks.ValidMask(1)=true;
            tracks.TrackIDs(1)=uint32(711); tracks.ClassIDs(1)=uint8(4);
            tracks.ExistenceProbabilities(1)=single(0.98);
            tracks.Positions(1,:)=single([45 0.2 0.9]);
            tracks.Velocities(1,:)=single([-8 0 0]);
            tracks.Dimensions(1,:)=single([2.8 1.4 1.8]);
            map.PredictedRisk(:)=single(0.90);
            map.CombinedCost=max(map.CombinedCost,map.PredictedRisk);
            assessment=RoadsenseBehaviourAssessmentSystem;
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyFalse(a{3});
            testCase.verifyTrue(a{8});
            testCase.verifyNotEqual(bitand(a{22},uint16(512)),uint16(0));
            [mode,~,~]=RoadsenseBehaviourDecisionCore(a{2},a{3},a{4},a{5}, ...
                a{6},a{7},a{8},a{9},uint8(2),uint16(0),uint16(8), ...
                uint16(5),uint16(8));
            testCase.verifyEqual(mode,uint8(10));
        end

        function unobservedHighwayDoesNotCreepForever(testCase)
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("cruise");
            map.ObservedMask(:)=false;
            ego.Velocity=[8;0;0];
            assessment=RoadsenseBehaviourAssessmentSystem;
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyFalse(a{7});
            testCase.verifyFalse(a{9});
        end

        function yieldRequiresEightClearSamples(testCase)
            [mode,hold,clear]=RoadsenseBehaviourDecisionCore(true,false,false,true,false, ...
                false,false,false,uint8(1),uint16(0),uint16(8),uint16(5),uint16(8));
            testCase.verifyEqual(mode,uint8(4));
            for tick=1:7
                [mode,hold,clear]=RoadsenseBehaviourDecisionCore(true,false,false,false, ...
                    false,false,false,false,mode,hold,clear,uint16(5),uint16(8));
                testCase.verifyEqual(mode,uint8(4));
            end
            [mode,~,~]=RoadsenseBehaviourDecisionCore(true,false,false,false,false,false, ...
                false,false,mode,hold,clear,uint16(5),uint16(8));
            testCase.verifyEqual(mode,uint8(1));
        end

        function commandBuilderSelectsBoundsAndSequence(testCase)
            command=RoadsenseBehaviourCommandSystem;
            inputs={1.0,uint8(1),true,single(8),single(4),single(0),single(1.2), ...
                single(3),single(4),single(1),single(8),uint32(0),single(inf), ...
                single(0.1),single(0),uint16(0)};
            [first{1:16}]=command(inputs{:});
            [second{1:16}]=command(inputs{:});
            testCase.verifyEqual(first{2},uint32(1));
            testCase.verifyTrue(first{14});
            testCase.verifyFalse(second{14});
            testCase.verifyEqual(first{4},single(8));
            inputs{2}=uint8(8);
            [emergency{1:16}]=command(inputs{:});
            testCase.verifyEqual(emergency{2},uint32(2));
            testCase.verifyEqual(emergency{4},single(0));
            testCase.verifyEqual(emergency{6},single(-6));
            testCase.verifyTrue(emergency{15});
        end

        function followTargetAndVulnerableClearanceAreSafe(testCase)
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("follow");
            assessment=RoadsenseBehaviourAssessmentSystem;
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyTrue(a{6});
            testCase.verifyEqual(a{18},uint32(501));
            testCase.verifyLessThan(a{11},a{10});
            [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs("yield");
            release(assessment);
            [a{1:22}]=assessment(ego,map,tracks,route);
            testCase.verifyTrue(a{5});
            testCase.verifyEqual(a{16},single(2));
        end

        function generatedModelContainsStateflowAndUpdates(testCase)
            modelPath=createRoadsenseBehaviourPlannerModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            rootObject=sfroot;
            chart=find(rootObject,"-isa","Stateflow.Chart","Path", ...
                char(modelName+"/Stateflow Supervisory Decision"));
            testCase.verifyNotEmpty(chart);
            state=find(chart,"-isa","Stateflow.State","Name","SupervisoryDecision");
            testCase.verifyNotEmpty(state);
            testCase.verifyEqual(get_param(modelName+"/BehaviourCommand","OutDataTypeStr"), ...
                'Bus: RsBehaviourCommandBus');
            clear cleanup; close_system(modelName,0);
        end
    end
end
