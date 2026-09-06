classdef testRoadsenseSafetySupervisor < matlab.unittest.TestCase
    %TESTROADSENSESAFETYSUPERVISOR Independent monitoring and override tests.
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
        function normalSignalsPassNominalControl(testCase)
            [inputs{1:8}]=createSyntheticRoadsenseSafetyInputs("normal");
            assessment=RoadsenseSafetyAssessmentSystem; [a{1:14}]=assessment(inputs{:});
            testCase.verifyTrue(a{2}); testCase.verifyFalse(a{3});
            [state,~,~,latched]=RoadsenseSafetyDecisionCore(a{2},a{3},a{4},a{5}, ...
                a{6},uint8(0),uint16(0),uint16(0),uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(1)); testCase.verifyFalse(latched);
            command=RoadsenseSafetyCommandSystem;
            [out{1:24}]=command(inputs{8},a{1},state,a{2},a{7},a{8},a{9},a{10}, ...
                a{11},a{12},a{13},a{14},latched);
            testCase.verifyEqual(out{3},inputs{8}.SteeringAngle);
            testCase.verifyEqual(out{5},inputs{8}.AccelerationCommand);
            testCase.verifyFalse(out{21}); testCase.verifyTrue(out{23});
        end

        function degradedStateEnforcesSpeedLimitedAuthority(testCase)
            [inputs{1:8}]=createSyntheticRoadsenseSafetyInputs("degraded");
            assessment=RoadsenseSafetyAssessmentSystem; [a{1:14}]=assessment(inputs{:});
            testCase.verifyTrue(a{5});
            testCase.verifyNotEqual(bitand(a{14},uint16(16)),uint16(0));
            [state,~,~,~]=RoadsenseSafetyDecisionCore(a{2},a{3},a{4},a{5}, ...
                a{6},uint8(1),uint16(0),uint16(0),uint16(50),uint16(25));
            command=RoadsenseSafetyCommandSystem; [out{1:24}]=command(inputs{8},a{1}, ...
                state,a{2},a{7},a{8},a{9},a{10},a{11},a{12},a{13},a{14},false);
            testCase.verifyEqual(state,uint8(2));
            testCase.verifyLessThanOrEqual(out{5},single(0));
            testCase.verifyLessThanOrEqual(out{8},single(4)); testCase.verifyTrue(out{21});

            % The same warning at standstill must allow bounded propulsion;
            % otherwise a persistent warning can strand the vehicle forever.
            inputs{1}.Velocity(:)=single(0); inputs{7}.LongitudinalSpeed=single(0);
            assessment=RoadsenseSafetyAssessmentSystem; [b{1:14}]=assessment(inputs{:});
            command=RoadsenseSafetyCommandSystem; [low{1:24}]=command(inputs{8},b{1}, ...
                state,b{2},b{7},b{8},b{9},b{10},b{11},b{12},b{13},b{14},false);
            testCase.verifyGreaterThan(low{5},single(0));
            testCase.verifyLessThanOrEqual(low{5},single(0.8));
        end

        function boundedAccelerationLimitAloneRemainsNominal(testCase)
            [inputs{1:8}]=createSyntheticRoadsenseSafetyInputs("normal");
            inputs{6}.AccelerationSaturated=true;
            assessment=RoadsenseSafetyAssessmentSystem; [a{1:14}]=assessment(inputs{:});
            testCase.verifyFalse(a{5});
            testCase.verifyEqual(bitand(a{14},uint16(64)),uint16(0));
        end

        function independentTTCForcesEmergencyBrake(testCase)
            [inputs{1:8}]=createSyntheticRoadsenseSafetyInputs("collision");
            assessment=RoadsenseSafetyAssessmentSystem; [a{1:14}]=assessment(inputs{:});
            testCase.verifyTrue(a{4}); testCase.verifyLessThan(a{8},single(1));
            testCase.verifyNotEqual(bitand(a{14},uint16(4)),uint16(0));
            [state,~,~,~]=RoadsenseSafetyDecisionCore(a{2},a{3},a{4},a{5}, ...
                a{6},uint8(1),uint16(0),uint16(0),uint16(50),uint16(25));
            command=RoadsenseSafetyCommandSystem; [out{1:24}]=command(inputs{8},a{1}, ...
                state,a{2},a{7},a{8},a{9},a{10},a{11},a{12},a{13},a{14},false);
            testCase.verifyEqual(state,uint8(3)); testCase.verifyEqual(out{5},single(-6));
            testCase.verifyEqual(out{7},single(1)); testCase.verifyTrue(out{9});
        end

        function criticalFaultsSelectMinimalRiskStop(testCase)
            scenarios=["tracking_fault","unstable","stale","invalid"];
            for index=1:numel(scenarios)
                [inputs{1:8}]=createSyntheticRoadsenseSafetyInputs(scenarios(index));
                assessment=RoadsenseSafetyAssessmentSystem; [a{1:14}]=assessment(inputs{:});
                testCase.verifyTrue(a{3},"Expected fault for "+scenarios(index));
                [state,~,~,~]=RoadsenseSafetyDecisionCore(a{2},a{3},a{4},a{5}, ...
                    a{6},uint8(1),uint16(0),uint16(0),uint16(50),uint16(25));
                testCase.verifyEqual(state,uint8(4));
            end
        end

        function emergencyLatchRejectsTransientClearFrame(testCase)
            [state,hold,recovery,latched]=RoadsenseSafetyDecisionCore(true,false,true, ...
                false,true,uint8(1),uint16(0),uint16(0),uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(3)); testCase.verifyFalse(latched);
            for index=1:49
                [state,hold,recovery,latched]=RoadsenseSafetyDecisionCore(true,false,false, ...
                    false,true,state,hold,recovery,uint16(50),uint16(25));
                testCase.verifyEqual(state,uint8(3)); testCase.verifyTrue(latched);
            end
            [state,~,~,~]=RoadsenseSafetyDecisionCore(true,false,false,false,true, ...
                state,hold,recovery,uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(1));
        end

        function minimalRiskStopHoldsUntilHealthyStandstill(testCase)
            [state,hold,recovery,~]=RoadsenseSafetyDecisionCore(false,true,false,false, ...
                true,uint8(1),uint16(0),uint16(0),uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(4));
            [state,hold,recovery,~]=RoadsenseSafetyDecisionCore(true,false,false,false, ...
                true,state,hold,recovery,uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(4));
            for index=1:24
                [state,hold,recovery,~]=RoadsenseSafetyDecisionCore(true,false,false,false, ...
                    false,state,hold,recovery,uint16(50),uint16(25));
                testCase.verifyEqual(state,uint8(5));
            end
            [state,~,~,~]=RoadsenseSafetyDecisionCore(true,false,false,false,false, ...
                state,hold,recovery,uint16(50),uint16(25));
            testCase.verifyEqual(state,uint8(1));
        end

        function generatedModelContainsStateflowAndRuns(testCase)
            modelPath=createRoadsenseSafetySupervisorModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            rootObject=sfroot; chart=find(rootObject,"-isa","Stateflow.Chart", ...
                "Path",char(modelName+"/Stateflow Safety Latch"));
            testCase.verifyNotEmpty(chart);
            testCase.verifyEqual(get_param(modelName+"/SafeVehicleControl","OutDataTypeStr"), ...
                'Bus: RsVehicleControlBus');
            testCase.verifyEqual(get_param(modelName+"/SafetyStatus","OutDataTypeStr"), ...
                'Bus: RsSafetyStatusBus');
            sim(modelName,"StopTime","0.04");
            clear cleanup; close_system(modelName,0);
        end
    end
end
