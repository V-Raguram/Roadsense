classdef testRoadsenseMotionPrediction < matlab.unittest.TestCase
    %TESTROADSENSEMOTIONPREDICTION Component tests for multimodal prediction.

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
        function invalidInputProducesSafeEmptyPrediction(testCase)
            tracks = createSyntheticRoadsenseTrackList;
            tracks.Valid = false;
            predictor = RoadsenseMotionPredictionSystem;
            [count,~,~,~,steps,offsets,~,positions,~,~,validMask,~,overflow,valid] = ...
                predictor(tracks);
            testCase.verifyEqual(count,uint16(0));
            testCase.verifyEqual(steps,uint8(16));
            testCase.verifyEqual(offsets(end),single(3),"AbsTol",single(1e-6));
            testCase.verifySize(positions,[128 16 4 2]);
            testCase.verifyFalse(any(validMask));
            testCase.verifyFalse(overflow);
            testCase.verifyFalse(valid);
        end

        function predictionsRespectContractAndProbabilities(testCase)
            tracks = createSyntheticRoadsenseTrackList;
            predictor = RoadsenseMotionPredictionSystem;
            [count,ids,classes,modes,steps,offsets,probabilities,positions,~,covariances,validMask,~,~,valid] = ...
                predictor(tracks);
            testCase.verifyEqual(count,uint16(5));
            testCase.verifyEqual(ids(1:5),tracks.TrackIDs(1:5));
            testCase.verifyEqual(classes(1:5),tracks.ClassIDs(1:5));
            testCase.verifyEqual(steps,uint8(16));
            testCase.verifyEqual(offsets(1),single(0));
            testCase.verifyEqual(offsets(end),single(3),"AbsTol",single(1e-6));
            for index = 1:5
                activeModes = 1:double(modes(index));
                testCase.verifyEqual(sum(probabilities(index,activeModes)),single(1), ...
                    "AbsTol",single(2e-6));
                for mode = activeModes
                    initial = squeeze(positions(index,1,mode,:)).';
                    testCase.verifyEqual(initial,tracks.Positions(index,1:2), ...
                        "AbsTol",single(1e-5));
                end
            end
            testCase.verifySize(covariances,[2 2 16 4 128]);
            testCase.verifyTrue(all(validMask(1:5)));
            testCase.verifyTrue(valid);
        end

        function vulnerableUsersBranchAndStaticObstacleStaysFixed(testCase)
            tracks = createSyntheticRoadsenseTrackList;
            predictor = RoadsenseMotionPredictionSystem;
            [~,~,~,modes,~,~,~,positions,yaws] = predictor(tracks);
            pedestrian = 3;
            leftEndpoint = squeeze(positions(pedestrian,end,3,:));
            rightEndpoint = squeeze(positions(pedestrian,end,4,:));
            testCase.verifyGreaterThan(norm(leftEndpoint-rightEndpoint),single(4));
            testCase.verifyGreaterThan(abs(yaws(pedestrian,end,3)-yaws(pedestrian,end,4)),single(1));
            staticObject = 5;
            testCase.verifyEqual(modes(staticObject),uint8(1));
            stationary = squeeze(positions(staticObject,:,1,:));
            expected = repmat(tracks.Positions(staticObject,1:2),16,1);
            testCase.verifyEqual(stationary,expected,"AbsTol",single(1e-6));
        end

        function covarianceGrowsAndRespondsToExistence(testCase)
            tracks = createSyntheticRoadsenseTrackList;
            predictor = RoadsenseMotionPredictionSystem;
            [~,~,~,~,~,~,~,~,~,covariances] = predictor(tracks);
            for index = 1:5
                initialTrace = trace(covariances(:,:,1,1,index));
                finalTrace = trace(covariances(:,:,end,1,index));
                testCase.verifyGreaterThan(finalTrace,initialTrace);
                testCase.verifyGreaterThanOrEqual(min(eig(double( ...
                    covariances(:,:,end,1,index)))),-1e-6);
            end
        end

        function brakingHistoryRaisesStoppingProbability(testCase)
            predictor = RoadsenseMotionPredictionSystem;
            first = createSyntheticRoadsenseTrackList(0,4.0);
            [~,~,~,~,~,~,firstProbabilities] = predictor(first);
            second = createSyntheticRoadsenseTrackList(0.1,2.5);
            [~,~,~,~,~,~,secondProbabilities] = predictor(second);
            testCase.verifyGreaterThan(secondProbabilities(1,2),firstProbabilities(1,2));
            testCase.verifyLessThan(secondProbabilities(1,1),firstProbabilities(1,1));
        end

        function acceptsLiveSensorFusionOutput(testCase)
            sample = createSyntheticRoadsenseFusionSequence(1);
            fusion = RoadsenseSensorFusionSystem;
            [count,ids,classes,classConfidence,existence,position,velocity,acceleration, ...
                yaw,yawRate,dimensions,covariance,ages,masks,confirmed,validMask, ...
                processingTime,sourceOverflow,overflow,valid] = ...
                fusion(sample.Time,sample.Camera,sample.Lidar,sample.Radar);
            tracks = struct("Timestamp",sample.Time,"Count",count,"TrackIDs",ids, ...
                "ClassIDs",classes,"ClassConfidences",classConfidence, ...
                "ExistenceProbabilities",existence,"Positions",position, ...
                "Velocities",velocity,"Accelerations",acceleration,"Yaws",yaw, ...
                "YawRates",yawRate,"Dimensions",dimensions,"StateCovariances",covariance, ...
                "Ages",ages,"SensorMasks",masks,"IsConfirmed",confirmed, ...
                "ValidMask",validMask,"ProcessingTime",processingTime, ...
                "SourceOverflow",sourceOverflow,"Overflow",overflow,"Valid",valid);
            predictor = RoadsenseMotionPredictionSystem;
            [predictionCount,trackIDs,~,numModes] = predictor(tracks);
            testCase.verifyEqual(predictionCount,count);
            testCase.verifyEqual(trackIDs(1:double(count)),ids(1:double(count)));
            testCase.verifyEqual(numModes(1:double(count)),repmat(uint8(4),double(count),1));
        end

        function generatedModelLoadsAndUpdates(testCase)
            modelPath = createRoadsenseMotionPredictionModel();
            [~,modelName] = fileparts(modelPath);
            load_system(modelPath);
            cleanup = onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName,"DataDictionary"),'Roadsense_Data.sldd');
            testCase.verifyEqual(get_param(modelName + "/FusedTracks","OutDataTypeStr"), ...
                'Bus: RsFusedTrackListBus');
            testCase.verifyEqual(get_param(modelName + "/Predictions","OutDataTypeStr"), ...
                'Bus: RsMotionPredictionListBus');
            testCase.verifyEqual(get_param(modelName + ...
                "/History Adaptive Multimodal Predictor","PredictionStep"),'RsPredictionStep');
            clear cleanup
            close_system(modelName,0);
        end
    end
end
