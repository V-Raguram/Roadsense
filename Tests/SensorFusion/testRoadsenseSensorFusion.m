classdef testRoadsenseSensorFusion < matlab.unittest.TestCase
    %TESTROADSENSESENSORFUSION Component tests for multi-sensor tracking.

    properties
        Root
    end

    methods (TestClassSetup)
        function configure(testCase)
            testFile = mfilename("fullpath");
            testCase.Root = fileparts(fileparts(fileparts(testFile)));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
        end
    end

    methods (Test)
        function invalidTimeProducesSafeEmptyOutput(testCase)
            sequence = createSyntheticRoadsenseFusionSequence(1);
            fusion = RoadsenseSensorFusionSystem;
            [count,~,~,~,~,~,~,~,~,~,~,~,~,~,~,validMask,~,~,overflow,valid] = ...
                fusion(NaN,sequence.Camera,sequence.Lidar,sequence.Radar);
            testCase.verifyEqual(count,uint16(0));
            testCase.verifyFalse(any(validMask));
            testCase.verifyFalse(overflow);
            testCase.verifyFalse(valid);
        end

        function spatialFusionCombinesComplementarySensors(testCase)
            sequence = createSyntheticRoadsenseFusionSequence(1);
            fusion = RoadsenseSensorFusionSystem;
            [count,~,classIDs,classConfidence,existence,positions,~,~,~,~,dimensions, ...
                covariances,~,sensorMasks,~,validMask,time,sourceOverflow,overflow,valid] = ...
                fusion(sequence.Time,sequence.Camera,sequence.Lidar,sequence.Radar);
            accepted = 1:double(count);
            testCase.verifyEqual(count,uint16(3));
            testCase.verifyEqual(sort(classIDs(accepted)),uint8([1;4;7]));
            testCase.verifyEqual(sensorMasks(accepted),repmat(uint8(7),3,1));
            testCase.verifyGreaterThan(classConfidence(accepted),single(0));
            testCase.verifyGreaterThan(existence(accepted),single(0));
            testCase.verifyTrue(all(validMask(accepted)));
            testCase.verifyTrue(all(isfinite(positions(accepted,:)),"all"));
            testCase.verifyTrue(all(dimensions(accepted,:) > 0,"all"));
            testCase.verifyTrue(all(isfinite(covariances(:,:,accepted)),"all"));
            testCase.verifyGreaterThan(time,single(0));
            testCase.verifyFalse(sourceOverflow);
            testCase.verifyFalse(overflow);
            testCase.verifyTrue(valid);
        end

        function lidarOnlyTracksRemainSemanticallyUnknown(testCase)
            sequence = createSyntheticRoadsenseFusionSequence(1);
            sequence.Camera.Valid = false;
            sequence.Radar.Valid = false;
            fusion = RoadsenseSensorFusionSystem;
            [count,~,classIDs,classConfidence] = fusion( ...
                sequence.Time,sequence.Camera,sequence.Lidar,sequence.Radar);
            accepted = 1:double(count);
            testCase.verifyEqual(count,uint16(3));
            testCase.verifyEqual(classIDs(accepted),zeros(3,1,"uint8"));
            testCase.verifyEqual(classConfidence(accepted),zeros(3,1,"single"));
        end

        function tracksPersistAndEstimateVelocity(testCase)
            sequence = createSyntheticRoadsenseFusionSequence();
            fusion = RoadsenseSensorFusionSystem;
            pedestrianIDBefore = uint32(0);
            pedestrianIDAfter = uint32(0);
            finalClasses = [];
            finalVelocities = [];
            finalConfirmed = [];
            for step = 1:numel(sequence)
                sample = sequence(step);
                [count,trackIDs,classIDs,~,~,~,velocities,~,~,~,~,~,~,~,confirmed] = ...
                    fusion(sample.Time,sample.Camera,sample.Lidar,sample.Radar);
                active = 1:double(count);
                pedestrian = find(classIDs(active) == uint8(7),1);
                if step == 17; pedestrianIDBefore = trackIDs(pedestrian); end
                if step == 22; pedestrianIDAfter = trackIDs(pedestrian); end
                if step == numel(sequence)
                    finalClasses = classIDs(active);
                    finalVelocities = velocities(active,:);
                    finalConfirmed = confirmed(active);
                end
            end
            testCase.verifyNotEqual(pedestrianIDBefore,uint32(0));
            testCase.verifyEqual(pedestrianIDAfter,pedestrianIDBefore, ...
                "The pedestrian ID changed across a 0.4 s complete occlusion.");
            testCase.verifyEqual(sort(finalClasses),uint8([1;4;7]));
            testCase.verifyTrue(all(finalConfirmed));
            car = find(finalClasses == uint8(1),1);
            pedestrian = find(finalClasses == uint8(7),1);
            testCase.verifyLessThan(abs(finalVelocities(car,1)-single(3.2)),single(0.8));
            testCase.verifyLessThan(abs(finalVelocities(pedestrian,2)-single(1.35)),single(0.8));
        end

        function mixedRadarAvailabilityKeepsOneMeasurementLayout(testCase)
            sequence = createSyntheticRoadsenseFusionSequence(6);
            fusion = RoadsenseSensorFusionSystem;
            for step = 1:numel(sequence)
                sample=sequence(step);
                sample.Radar.ValidMask(2)=false;
                [count,~,~,~,~,positions] = fusion( ...
                    sample.Time,sample.Camera,sample.Lidar,sample.Radar);
                testCase.verifyGreaterThan(count,uint16(0));
                testCase.verifyTrue(all(isfinite(positions(1:double(count),:)),"all"));
            end
        end

        function repeatedParentEvaluationReturnsCachedFusionTick(testCase)
            sequence=createSyntheticRoadsenseFusionSequence(1);
            fusion=RoadsenseSensorFusionSystem;
            [countA,idsA,classesA,~,~,positionsA]=fusion( ...
                sequence.Time,sequence.Camera,sequence.Lidar,sequence.Radar);
            [countB,idsB,classesB,~,~,positionsB]=fusion( ...
                sequence.Time,sequence.Camera,sequence.Lidar,sequence.Radar);
            testCase.verifyEqual(countB,countA);
            testCase.verifyEqual(idsB,idsA); testCase.verifyEqual(classesB,classesA);
            testCase.verifyEqual(positionsB,positionsA);
        end

        function generatedModelLoadsAndUpdates(testCase)
            modelPath = createRoadsenseSensorFusionModel();
            [~,modelName] = fileparts(modelPath);
            load_system(modelPath);
            cleanup = onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName,"DataDictionary"),'Roadsense_Data.sldd');
            testCase.verifyEqual(get_param(modelName + "/RadarObjects","OutDataTypeStr"), ...
                'Bus: RsRadarObjectListBus');
            testCase.verifyEqual(get_param(modelName + "/FusedTracks","OutDataTypeStr"), ...
                'Bus: RsFusedTrackListBus');
            testCase.verifyEqual(get_param(modelName + ...
                "/Gated Spatial Fusion and JPDA IMM Tracking","CameraLidarGate"), ...
                'RsFusionCameraLidarGate');
            clear cleanup
            close_system(modelName,0);
        end
    end
end
