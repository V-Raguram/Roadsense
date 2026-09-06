classdef testRoadsenseSemanticPerception < matlab.unittest.TestCase
    %TESTROADSENSESEMANTICPERCEPTION Component tests for camera semantic AI.

    properties
        Root
    end

    methods (TestClassSetup)
        function configure(testCase)
            testFile = mfilename("fullpath");
            testCase.Root = fileparts(fileparts(fileparts(testFile)));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            addpath(fullfile(testCase.Root,"Models","SharedData"));
            addpath(fullfile(testCase.Root,"Models","SemanticPerception"));
            addpath(fullfile(testCase.Root,"Data","Networks"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
            downloadRoadsenseSemanticBaseline();
        end
    end

    methods (Test)
        function syntheticColorBackendDecodesScenarioCamera(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(1,7);
            camera=RoadsenseScenarioCameraSystem;
            [image,valid]=camera(data.CurrentTime,data.EgoState,data.TruthActors, ...
                data.ScenarioDefinition,data.ScenarioStatus);
            perception=RoadsenseSemanticPerceptionSystem;
            perception.InferenceMode="syntheticColor";
            [labels,confidence,drivable,count,~,classIDs,scores,~,~,mask,~,ready,~,outputValid]= ...
                perception(image,valid);
            testCase.verifyTrue(ready); testCase.verifyTrue(outputValid);
            testCase.verifyGreaterThan(nnz(labels==uint8(1)),1000);
            testCase.verifyGreaterThan(mean(drivable(labels==uint8(1))),single(0.9));
            testCase.verifyGreaterThan(count,uint16(0));
            testCase.verifyTrue(all(scores(mask)>single(0.9)));
            testCase.verifyTrue(all(classIDs(mask)>uint8(0)));
            testCase.verifyGreaterThan(mean(confidence,"all"),single(0.6));
        end

        function syntheticCameraProjectionPreservesActorRange(testCase)
            [definition,truth,status]=RoadsenseScenarioCatalog(1,0.3);
            ego=struct("Timestamp",0.3,"Position",definition.InitialPosition, ...
                "Velocity",zeros(3,1),"Acceleration",zeros(3,1), ...
                "Yaw",definition.InitialYaw,"Pitch",0,"Roll",0,"YawRate",0, ...
                "SteeringAngle",0,"Valid",true);
            camera=RoadsenseScenarioCameraSystem;
            [image,valid]=camera(0.3,ego,truth,definition,status);
            perception=RoadsenseSemanticPerceptionSystem;
            perception.InferenceMode="syntheticColor";
            [~,~,~,count,~,classIDs,~,positions,~,mask]=perception(image,valid);
            candidates=find(mask(1:double(count)) & classIDs(1:double(count))==uint8(6));
            testCase.verifyNotEmpty(candidates);
            actualRange=truth.Positions(2,1)-ego.Position(1);
            testCase.verifyEqual(double(positions(candidates(1),1)),actualRange, ...
                "AbsTol",2.0);
        end

        function invalidFrameIsRejectedWithoutInference(testCase)
            perception = RoadsenseSemanticPerceptionSystem;
            image = zeros(480,640,3,"uint8");
            [labels,confidence,drivable,count,~,~,~,~,~,validMask,time,ready,overflow,valid] = ...
                perception(image,false);
            testCase.verifySize(labels,[120 160]);
            testCase.verifySize(confidence,[120 160]);
            testCase.verifySize(drivable,[120 160]);
            testCase.verifyEqual(count,uint16(0));
            testCase.verifyFalse(any(validMask));
            testCase.verifyEqual(time,single(0));
            testCase.verifyTrue(ready);
            testCase.verifyFalse(overflow);
            testCase.verifyFalse(valid);
        end

        function realNetworkInferenceHasValidContract(testCase)
            perception = RoadsenseSemanticPerceptionSystem;
            image = syntheticRoadImage();
            [labels,confidence,drivable,count,boxes,classIDs,scores,positions,covariances, ...
                validMask,time,ready,~,valid] = perception(image,true);
            testCase.verifySize(labels,[120 160]);
            testCase.verifyGreaterThanOrEqual(confidence,single(0));
            testCase.verifyLessThanOrEqual(confidence,single(1));
            testCase.verifyGreaterThanOrEqual(drivable,single(0));
            testCase.verifyLessThanOrEqual(drivable,single(1));
            testCase.verifyLessThanOrEqual(double(count),256);
            testCase.verifySize(boxes,[256 4]);
            testCase.verifySize(classIDs,[256 1]);
            testCase.verifySize(scores,[256 1]);
            testCase.verifySize(positions,[256 3]);
            testCase.verifySize(covariances,[3 3 256]);
            testCase.verifySize(validMask,[256 1]);
            testCase.verifyGreaterThan(time,single(0));
            testCase.verifyTrue(ready);
            testCase.verifyTrue(valid);
        end

        function generatedModelLoadsAndUpdates(testCase)
            modelPath = createRoadsenseSemanticPerceptionModel();
            [~,modelName] = fileparts(modelPath);
            load_system(modelPath);
            cleanup = onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            testCase.verifyEqual(get_param(modelName,"DataDictionary"), ...
                'Roadsense_Data.sldd');
            testCase.verifyEqual(get_param(modelName + "/CameraFrame","OutDataTypeStr"), ...
                'Bus: RsCameraFrameBus');
            clear cleanup
            close_system(modelName,0);
        end
    end
end

function image = syntheticRoadImage()
image = zeros(480,640,3,"uint8");
image(1:260,:,:) = uint8(150);
for row = 261:480
    halfWidth = round((row-240)*1.25);
    left = max(1,320-halfWidth);
    right = min(640,320+halfWidth);
    image(row,left:right,1) = 70;
    image(row,left:right,2) = 70;
    image(row,left:right,3) = 70;
end
image(330:410,275:365,1) = 180;
image(330:410,275:365,2) = 30;
image(330:410,275:365,3) = 30;
end
