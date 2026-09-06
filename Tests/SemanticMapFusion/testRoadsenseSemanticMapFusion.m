classdef testRoadsenseSemanticMapFusion < matlab.unittest.TestCase
    %TESTROADSENSESEMANTICMAPFUSION Component tests for the traversability map.

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
        function invalidSourcesProduceSafeEmptyGrid(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            image.Valid=false; lidar.Valid=false; tracks.Valid=false; predictions.Valid=false;
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,~,~,static,dynamic,risk,~,drivable,~,cost,labels,observed,~,~,valid] = ...
                fusion(image,lidar,tracks,predictions);
            testCase.verifyFalse(valid);
            testCase.verifyFalse(any(static,"all"));
            testCase.verifyFalse(any(dynamic,"all"));
            testCase.verifyFalse(any(risk,"all"));
            testCase.verifyFalse(any(observed,"all"));
            testCase.verifyFalse(any(labels,"all"));
            testCase.verifyEqual(drivable,repmat(single(0.35),200,320));
            testCase.verifyFalse(any(cost,"all"));
        end

        function lidarSurfaceAndObstacleEvidenceArePreserved(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,xLimits,yLimits,static,~,~,occupancy,~,surface,~,labels] = ...
                fusion(image,lidar,tracks,predictions);
            [obstacleRow,obstacleColumn] = gridIndex(28,-5,xLimits,yLimits);
            [potholeRow,potholeColumn] = gridIndex(20,-7,xLimits,yLimits);
            testCase.verifyGreaterThan(static(obstacleRow,obstacleColumn),single(0.8));
            testCase.verifyGreaterThan(occupancy(obstacleRow,obstacleColumn),single(0.8));
            testCase.verifyEqual(labels(obstacleRow,obstacleColumn),uint8(9));
            testCase.verifyGreaterThan(surface(potholeRow,potholeColumn),single(0.6));
            testCase.verifyEqual(labels(potholeRow,potholeColumn),uint8(5));
        end

        function cameraEvidenceProjectsToGround(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            lidar.Valid=false; tracks.Valid=false; predictions.Valid=false;
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,~,~,~,~,~,~,drivability,~,~,labels,observed,~,~,valid] = ...
                fusion(image,lidar,tracks,predictions);
            testCase.verifyTrue(valid);
            testCase.verifyGreaterThan(nnz(observed),100);
            testCase.verifyGreaterThan(max(drivability,[],"all"),single(0.70));
            testCase.verifyTrue(any(labels == uint8(1),"all"));
            testCase.verifyTrue(any(labels == uint8(4),"all"));
        end

        function tracksAndPredictionsCreateRiskLayers(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            [rawRow,rawColumn] = gridIndex(tracks.Positions(1,1), ...
                tracks.Positions(1,2),lidar.XLimits,lidar.YLimits);
            lidar.Occupancy(rawRow,rawColumn)=single(0.98);
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,xLimits,yLimits,static,dynamic,risk,occupancy,~,~,cost,labels,~,time,~,valid] = ...
                fusion(image,lidar,tracks,predictions);
            [carRow,carColumn] = gridIndex(tracks.Positions(1,1),tracks.Positions(1,2),xLimits,yLimits);
            [pedestrianRow,pedestrianColumn] = gridIndex( ...
                tracks.Positions(3,1),tracks.Positions(3,2),xLimits,yLimits);
            testCase.verifyGreaterThan(dynamic(carRow,carColumn),single(0.9));
            testCase.verifyEqual(static(carRow,carColumn),single(0));
            testCase.verifyGreaterThan(occupancy(carRow,carColumn),single(0.9));
            testCase.verifyEqual(labels(carRow,carColumn),uint8(6));
            testCase.verifyEqual(labels(pedestrianRow,pedestrianColumn),uint8(7));
            testCase.verifyGreaterThan(max(risk,[],"all"),single(0.5));
            testCase.verifyGreaterThanOrEqual(cost,maximumOfFour(dynamic,risk,lidar.SurfaceCost, ...
                zeros(200,320,"single"))-single(1e-6));
            testCase.verifyGreaterThan(time,single(0));
            testCase.verifyTrue(valid);
        end

        function unknownSpaceIsPenalizedAndOverflowPropagates(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            tracks.Overflow=true;
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,xLimits,yLimits,~,~,~,~,drivability,~,cost,~,observed,~,overflow,valid] = ...
                fusion(image,lidar,tracks,predictions);
            [row,column] = gridIndex(55,20,xLimits,yLimits);
            testCase.verifyFalse(observed(row,column));
            testCase.verifyEqual(drivability(row,column),single(0.35),"AbsTol",single(1e-6));
            testCase.verifyGreaterThan(cost(row,column),single(0.45));
            testCase.verifyTrue(overflow);
            testCase.verifyTrue(valid);
        end

        function extremeCovarianceRiskRemainsSpatiallyBounded(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            tracks.StateCovariances(:,:,1)=single(eye(6)*1e6);
            for step=1:16
                predictions.PositionCovariances(:,:,step,1,1)=single(eye(2)*1e6);
            end
            fusion=RoadsenseSemanticMapFusionSystem;
            [~,~,~,~,~,dynamic,risk,~,~,~,~,~,~,~,~,valid]= ...
                fusion(image,lidar,tracks,predictions);
            testCase.verifyTrue(valid);
            testCase.verifyLessThan(nnz(dynamic),5000);
            testCase.verifyLessThan(nnz(risk),40000);
        end

        function incompatibleLidarGeometryIsRejected(testCase)
            [image,lidar,tracks,predictions] = createSyntheticRoadsenseMapInputs;
            lidar.Resolution=single(0.5);
            fusion = RoadsenseSemanticMapFusionSystem;
            [~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,valid] = ...
                fusion(image,lidar,tracks,predictions);
            testCase.verifyFalse(valid);
        end

        function generatedModelLoadsAndUpdates(testCase)
            modelPath = createRoadsenseSemanticMapFusionModel();
            [~,modelName] = fileparts(modelPath);
            load_system(modelPath);
            cleanup = onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName,"DataDictionary"),'Roadsense_Data.sldd');
            testCase.verifyEqual(get_param(modelName + "/SemanticGrid","OutDataTypeStr"), ...
                'Bus: RsSemanticGridBus');
            testCase.verifyEqual(get_param(modelName + ...
                "/Camera Lidar Track and Prediction Fusion","UnknownDrivability"), ...
                'RsMapUnknownDrivability');
            clear cleanup
            close_system(modelName,0);
        end
    end
end

function [row,column] = gridIndex(x,y,xLimits,yLimits)
resolution=0.25;
column=floor((double(x)-double(xLimits(1)))/resolution)+1;
row=floor((double(y)-double(yLimits(1)))/resolution)+1;
end

function maximum = maximumOfFour(first,second,third,fourth)
maximum=max(max(first,second),max(third,fourth));
end
