classdef testRoadsenseLidarPerception < matlab.unittest.TestCase
    %TESTROADSENSELIDARPERCEPTION Component tests for LiDAR perception.

    properties
        Root
    end

    methods (TestClassSetup)
        function configure(testCase)
            testFile = mfilename("fullpath");
            testCase.Root = fileparts(fileparts(fileparts(testFile)));
            addpath(fullfile(testCase.Root,"Models","SharedData"));
            addpath(fullfile(testCase.Root,"Models","LidarPerception"));
            addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
        end
    end

    methods (Test)
        function invalidScanProducesSafeEmptyOutput(testCase)
            perception = RoadsenseLidarPerceptionSystem;
            points = zeros(120000,3,"single");
            [occupancy,~,~,pothole,~,observed,count,~,~,~,~,~,~,~,validMask,time,overflow,valid] = ...
                perception(points,uint32(0),false,false);
            testCase.verifySize(occupancy,[200 320]);
            testCase.verifyFalse(any(observed,"all"));
            testCase.verifyFalse(any(pothole,"all"));
            testCase.verifyEqual(count,uint16(0));
            testCase.verifyFalse(any(validMask));
            testCase.verifyEqual(time,single(0));
            testCase.verifyFalse(overflow);
            testCase.verifyFalse(valid);
        end

        function syntheticUnevenRoadProducesEvidenceAndObjects(testCase)
            frame = createSyntheticRoadsenseLidarFrame();
            perception = RoadsenseLidarPerceptionSystem;
            [occupancy,groundHeight,roughness,pothole,surface,observed,count,positions, ...
                dimensions,~,classIDs,scores,covariances,pointCounts,validMask,time,overflow,valid] = ...
                perception(frame.Points,frame.Count,frame.Overflow,frame.Valid);
            testCase.verifyTrue(valid);
            testCase.verifyFalse(overflow);
            testCase.verifyGreaterThanOrEqual(double(count),2);
            testCase.verifyGreaterThan(max(occupancy,[],"all"),single(0.5));
            testCase.verifyGreaterThan(max(pothole,[],"all"),single(0.15));
            testCase.verifyGreaterThan(max(surface,[],"all"),single(0.15));
            testCase.verifyTrue(any(observed,"all"));
            testCase.verifyTrue(all(isfinite(groundHeight),"all"));
            testCase.verifyTrue(all(isfinite(roughness),"all"));
            testCase.verifyTrue(all(validMask(1:double(count))));
            testCase.verifyEqual(classIDs(1:double(count)),zeros(double(count),1,"uint8"));
            testCase.verifyGreaterThan(scores(1:double(count)),single(0));
            testCase.verifyGreaterThan(pointCounts(1:double(count)),uint32(0));
            testCase.verifyTrue(all(isfinite(positions(1:double(count),:)),"all"));
            testCase.verifyTrue(all(dimensions(1:double(count),:) > 0,"all"));
            testCase.verifySize(covariances,[3 3 256]);
            testCase.verifyGreaterThan(time,single(0));
        end

        function generatedModelLoadsAndUpdates(testCase)
            modelPath = createRoadsenseLidarPerceptionModel();
            [~,modelName] = fileparts(modelPath);
            load_system(modelPath);
            cleanup = onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName,"DataDictionary"), ...
                'Roadsense_Data.sldd');
            testCase.verifyEqual(get_param(modelName + "/LidarFrame","OutDataTypeStr"), ...
                'Bus: RsLidarFrameBus');
            clear cleanup
            close_system(modelName,0);
        end
    end
end
