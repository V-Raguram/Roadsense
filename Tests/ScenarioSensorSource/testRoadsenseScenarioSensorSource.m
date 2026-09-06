classdef testRoadsenseScenarioSensorSource < matlab.unittest.TestCase
    %TESTROADSENSESCENARIOSENSORSOURCE Synthetic camera, LiDAR, and radar.
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
        function cameraRendersFixedSizeVisibleActors(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(5,7); sensor=RoadsenseScenarioCameraSystem;
            [image,valid,count,occluded,dropout]=sensor(data.CurrentTime,data.EgoState, ...
                data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            testCase.verifySize(image,[480 640 3]); testCase.verifyClass(image,"uint8");
            testCase.verifyTrue(valid); testCase.verifyFalse(dropout);
            testCase.verifyGreaterThan(count,uint16(0)); testCase.verifyGreaterThan(nnz(image),100000);
            testCase.verifyClass(occluded,"uint16");
        end

        function angularOcclusionSuppressesFarActor(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(1,1);
            data.EgoState.Position=[0;0;0]; data.EgoState.Yaw=0;
            data.TruthActors.Count=uint16(2); data.TruthActors.ValidMask(:)=false;
            data.TruthActors.ValidMask(1:2)=true; data.TruthActors.ActorIDs(1:2)=uint32([1;2]);
            data.TruthActors.Positions(1:2,:)=[12 0 0;25 0.1 0];
            data.TruthActors.Dimensions(1:2,:)=single([4 2 1.5;4 2 1.5]);
            [indices,~,~,occluded]=RoadsenseScenarioSensorUtilities.visibleActors( ...
                data.TruthActors,data.EgoState,80,deg2rad(90));
            testCase.verifyNumElements(indices,1); testCase.verifyEqual(indices(1),1);
            testCase.verifyEqual(occluded,uint16(1));
        end

        function lidarContainsGroundObjectsAndSurfaceDepression(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(5,7); sensor=RoadsenseScenarioLidarSystem;
            [points,intensity,count,overflow,valid,actors,~,dropout]=sensor( ...
                data.CurrentTime,data.EgoState,data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            cloud=points(1:double(count),:);
            testCase.verifyTrue(valid); testCase.verifyFalse(dropout); testCase.verifyFalse(overflow);
            testCase.verifyGreaterThan(count,uint32(5000)); testCase.verifyGreaterThan(actors,uint16(0));
            testCase.verifyLessThan(min(cloud(:,3)),single(-0.10));
            testCase.verifyGreaterThan(max(cloud(:,3)),single(1.0));
            testCase.verifyGreaterThanOrEqual(min(intensity(1:double(count))),single(0));
            testCase.verifyLessThanOrEqual(max(intensity(1:double(count))),single(1));
        end

        function radarIsDeterministicWithPositiveCovariance(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(3,7);
            first=RoadsenseScenarioRadarSystem; [a{1:11}]=first(data.CurrentTime,data.EgoState, ...
                data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            second=RoadsenseScenarioRadarSystem; [b{1:11}]=second(data.CurrentTime,data.EgoState, ...
                data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            testCase.verifyEqual(a,b); testCase.verifyTrue(a{8});
            testCase.verifyGreaterThan(a{6},uint16(0));
            testCase.verifyGreaterThan(a{3}(1,1,1),single(0));
            testCase.verifyGreaterThan(a{4}(2,2,1),single(0));
        end

        function scenarioSpecificDropoutsAreExplicit(testCase)
            cameraData=createSyntheticRoadsenseScenarioSensorInputs(2,5.9);
            camera=RoadsenseScenarioCameraSystem;
            [~,valid,~,~,dropout]=camera(cameraData.CurrentTime,cameraData.EgoState, ...
                cameraData.TruthActors,cameraData.ScenarioDefinition,cameraData.ScenarioStatus);
            testCase.verifyTrue(dropout); testCase.verifyFalse(valid);
            lidarData=createSyntheticRoadsenseScenarioSensorInputs(1,12.05);
            lidar=RoadsenseScenarioLidarSystem; [out{1:8}]=lidar(lidarData.CurrentTime, ...
                lidarData.EgoState,lidarData.TruthActors,lidarData.ScenarioDefinition,lidarData.ScenarioStatus);
            testCase.verifyTrue(out{8}); testCase.verifyFalse(out{5});
            radarData=createSyntheticRoadsenseScenarioSensorInputs(5,6.55);
            radar=RoadsenseScenarioRadarSystem; [outRadar{1:11}]=radar(radarData.CurrentTime, ...
                radarData.EgoState,radarData.TruthActors,radarData.ScenarioDefinition,radarData.ScenarioStatus);
            testCase.verifyTrue(outRadar{11}); testCase.verifyFalse(outRadar{8});
        end

        function worldToEgoUsesCurrentPoseAndVelocity(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(1,1);
            data.EgoState.Position=[10;5;0]; data.EgoState.Yaw=pi/2;
            data.EgoState.Velocity=[0;2;0];
            [position,velocity]=RoadsenseScenarioSensorUtilities.worldToEgo( ...
                data.EgoState,[10 15 0],[0 5 0]);
            testCase.verifyEqual(position,[10 0 0],"AbsTol",1e-10);
            testCase.verifyEqual(velocity,[3 0 0],"AbsTol",1e-10);
        end

        function outputsAreAcceptedByExistingAdapters(testCase)
            data=createSyntheticRoadsenseScenarioSensorInputs(5,7);
            camera=RoadsenseScenarioCameraSystem; [image,cameraValid]=camera(data.CurrentTime, ...
                data.EgoState,data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            cameraAdapter=RoadsenseCameraFrameAdapterSystem;
            [~,~,adaptedImage,adaptedValid]=cameraAdapter(data.CurrentTime,image,cameraValid);
            testCase.verifyTrue(adaptedValid); testCase.verifyEqual(adaptedImage,image);

            lidar=RoadsenseScenarioLidarSystem; [points,intensity,count,overflow,lidarValid]=lidar( ...
                data.CurrentTime,data.EgoState,data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            lidarAdapter=RoadsenseLidarFrameAdapterSystem;
            [~,~,adaptedCount,adaptedPoints,~,~,adaptedValid]=lidarAdapter(data.CurrentTime, ...
                points,intensity,count,overflow,lidarValid);
            testCase.verifyTrue(adaptedValid); testCase.verifyEqual(adaptedCount,count);
            testCase.verifyEqual(adaptedPoints(1:10,:),points(1:10,:));

            radar=RoadsenseScenarioRadarSystem; [positions,velocities,pCov,vCov,scores, ...
                radarCount,radarOverflow,radarValid]=radar(data.CurrentTime,data.EgoState, ...
                data.TruthActors,data.ScenarioDefinition,data.ScenarioStatus);
            radarAdapter=RoadsenseRadarObjectAdapterSystem;
            [~,~,adaptedRadarCount,~,~,~,~,~,mask,~,adaptedRadarValid]=radarAdapter( ...
                data.CurrentTime,positions,velocities,pCov,vCov,scores,radarCount,radarOverflow,radarValid);
            testCase.verifyTrue(adaptedRadarValid); testCase.verifyEqual(adaptedRadarCount,radarCount);
            testCase.verifyEqual(nnz(mask),double(radarCount));
        end

        function generatedModelHasTypedPortsAndRuns(testCase)
            modelPath=createRoadsenseScenarioSensorSourceModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            outports=find_system(modelName,"SearchDepth",1,"BlockType","Outport");
            testCase.verifyNumElements(outports,16);
            status=outports(strcmp(get_param(outports,"Name"),'SensorStatus'));
            testCase.verifyEqual(get_param(status{1},"OutDataTypeStr"),'Bus: RsScenarioSensorStatusBus');
            subsystems=find_system(modelName,"SearchDepth",1,"BlockType","SubSystem");
            testCase.verifyNumElements(subsystems,4);
            for index=1:numel(subsystems)
                testCase.verifyEqual(get_param(subsystems{index},"ContentPreviewEnabled"),'off');
            end
            sim(modelName,"StopTime","0.1");
            clear cleanup; close_system(modelName,0);
        end
    end
end
