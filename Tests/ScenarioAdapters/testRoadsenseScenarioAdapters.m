classdef testRoadsenseScenarioAdapters < matlab.unittest.TestCase
    %TESTROADSENSESCENARIOADAPTERS Sensor normalization, routes, and resets.
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
        function cameraIDsAdvanceOnlyForValidFrames(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs(); adapter=RoadsenseCameraFrameAdapterSystem;
            [~,id1,image1,valid1]=adapter(1,data.CameraImage,true);
            [~,id2,image2,valid2]=adapter(1.1,data.CameraImage,false);
            testCase.verifyTrue(valid1); testCase.verifyEqual(id1,uint32(1));
            testCase.verifyFalse(valid2); testCase.verifyEqual(id2,uint32(1));
            testCase.verifyEqual(image1,data.CameraImage); testCase.verifyEqual(nnz(image2),0);
        end

        function lidarClampsIntensityAndRejectsNonfiniteData(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs(); adapter=RoadsenseLidarFrameAdapterSystem;
            data.LidarIntensity(1)=single(-1); data.LidarIntensity(2)=single(2);
            [~,~,count,~,intensity,~,valid]=adapter(1,data.LidarPoints, ...
                data.LidarIntensity,data.LidarCount,false,true);
            testCase.verifyTrue(valid); testCase.verifyEqual(count,uint32(8));
            testCase.verifyEqual(intensity(1:2),single([0;1]));
            release(adapter); adapter=RoadsenseLidarFrameAdapterSystem;
            data.LidarPoints(3,1)=single(NaN);
            [~,~,count,~,~,~,valid]=adapter(1,data.LidarPoints, ...
                data.LidarIntensity,data.LidarCount,false,true);
            testCase.verifyFalse(valid); testCase.verifyEqual(count,uint32(0));
        end

        function radarCreatesMasksAndPositiveCovariance(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs();
            data.RadarPositionCovariances(1,1,1)=single(-2);
            adapter=RoadsenseRadarObjectAdapterSystem;
            [~,id,count,~,~,covariance,~,scores,mask,overflow,valid]=adapter(1, ...
                data.RadarPositions,data.RadarVelocities,data.RadarPositionCovariances, ...
                data.RadarVelocityCovariances,data.RadarScores,data.RadarCount,false,true);
            testCase.verifyTrue(valid); testCase.verifyEqual(id,uint32(1));
            testCase.verifyEqual(count,uint16(2)); testCase.verifyEqual(nnz(mask),2);
            testCase.verifyGreaterThan(covariance(1,1,1),single(0));
            testCase.verifyGreaterThanOrEqual(scores(1),single(0)); testCase.verifyFalse(overflow);
        end

        function routeIsTransformedIntoCurrentEgoFrame(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs(); adapter=RoadsenseRouteAdapterSystem;
            data.EgoState.Position=[10;0;0]; data.EgoState.Yaw=pi/2;
            [out{1:17}]=adapter(1,data.EgoState,data.ScenarioDefinition);
            testCase.verifyTrue(out{10}); testCase.verifyTrue(out{17});
            testCase.verifyGreaterThan(out{12},uint16(1));
            % A route continuing in world +x appears primarily to ego-right at yaw pi/2.
            testCase.verifyLessThan(out{13}(2,2),single(0));
            testCase.verifyLessThanOrEqual(out{2},out{3});
        end

        function routeProgressDoesNotRegressAndGoalIsDetected(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs(); adapter=RoadsenseRouteAdapterSystem;
            data.EgoState.Position=data.ScenarioDefinition.WorldPositions(26,:).';
            [out1{1:17}]=adapter(1,data.EgoState,data.ScenarioDefinition);
            data.EgoState.Position=data.ScenarioDefinition.WorldPositions(2,:).';
            [out2{1:17}]=adapter(1.1,data.EgoState,data.ScenarioDefinition);
            testCase.verifyGreaterThan(out2{13}(1,1),single(30));
            data.EgoState.Position=data.ScenarioDefinition.WorldPositions( ...
                double(data.ScenarioDefinition.Count),:).';
            [out3{1:17}]=adapter(1.2,data.EgoState,data.ScenarioDefinition);
            testCase.verifyTrue(out3{5});
            testCase.verifyLessThan(out3{4},single(1.5));
            testCase.verifyLessThan(out2{12},out1{12}+uint16(5));
        end

        function resetPulsesOnStartChangeAndExplicitRequest(testCase)
            data=createSyntheticRoadsenseScenarioAdapterInputs(); adapter=RoadsenseScenarioInitializationSystem;
            [first{1:17}]=adapter(0,data.ScenarioDefinition,false);
            [second{1:17}]=adapter(0.02,data.ScenarioDefinition,false);
            data.ScenarioDefinition.ScenarioID=uint16(2);
            [changed{1:17}]=adapter(0.04,data.ScenarioDefinition,false);
            [requested{1:17}]=adapter(0.06,data.ScenarioDefinition,true);
            [held{1:17}]=adapter(0.08,data.ScenarioDefinition,true);
            testCase.verifyTrue(first{17}); testCase.verifyFalse(second{17});
            testCase.verifyTrue(changed{17}); testCase.verifyTrue(requested{17});
            testCase.verifyFalse(held{17});
            testCase.verifyEqual(first{2},data.ScenarioDefinition.InitialPosition);
            testCase.verifyEqual(first{12},single(0.8));
        end

        function generatedModelMatchesIntegrationInputsAndRuns(testCase)
            modelPath=createRoadsenseScenarioAdaptersModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            expected={ 'Bus: RsCameraFrameBus','Bus: RsLidarFrameBus', ...
                'Bus: RsRadarObjectListBus','Bus: RsRouteContextBus', ...
                'Bus: RsReferencePathBus','Bus: RsEgoStateBus', ...
                'Bus: RsRoadConditionBus','boolean'};
            outports=find_system(modelName,"SearchDepth",1,"BlockType","Outport");
            ports=zeros(numel(outports),1);
            for index=1:numel(outports); ports(index)=str2double(get_param(outports{index},"Port")); end
            [~,order]=sort(ports); outports=outports(order);
            for index=1:8
                testCase.verifyEqual(get_param(outports{index},"OutDataTypeStr"),expected{index});
            end
            transitions=find_system(modelName,"SearchDepth",1,"BlockType","RateTransition");
            testCase.verifyNumElements(transitions,5);
            sim(modelName,"StopTime","0.02");
            clear cleanup; close_system(modelName,0);
        end
    end
end
