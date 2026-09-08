classdef testRoadsenseScenarioLibrary < matlab.unittest.TestCase
    %TESTROADSENSESCENARIOLIBRARY Required scenarios, events, and assets.
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
        function definesFiveValidDistinctRoutes(testCase)
            signatures=strings(5,1);
            for id=1:5
                [definition,actors,status]=RoadsenseScenarioCatalog(id,0);
                testCase.verifyTrue(definition.Valid); testCase.verifyTrue(actors.Valid);
                testCase.verifyTrue(status.Valid); testCase.verifyEqual(status.ScenarioID,uint16(id));
                testCase.verifyGreaterThan(definition.Count,uint16(30));
                testCase.verifyGreaterThan(definition.DesiredSpeed,single(0));
                testCase.verifyLessThanOrEqual(definition.DesiredSpeed,definition.SpeedLimit);
                route=definition.WorldPositions(1:double(definition.Count),:);
                signatures(id)=sprintf("%d_%.2f_%.2f",definition.Count,route(end,1),sum(abs(route(:,2))));
            end
            testCase.verifyEqual(numel(unique(signatures)),5);
        end

        function coversIndianRoadUserClasses(testCase)
            allClasses=zeros(0,1,"uint8");
            for id=1:5
                [~,~,~,plans]=RoadsenseScenarioCatalog(id,0);
                allClasses=[allClasses;vertcat(plans.ClassID)]; %#ok<AGROW>
            end
            required=uint8([2 3 4 5 6 7 8 9]);
            testCase.verifyTrue(all(ismember(required,unique(allClasses))));
        end

        function emitsScenarioSpecificEventWindows(testCase)
            [~,~,village]=RoadsenseScenarioCatalog(1,7);
            [~,~,urban]=RoadsenseScenarioCatalog(2,8);
            [~,~,merge]=RoadsenseScenarioCatalog(3,7);
            [~,~,market]=RoadsenseScenarioCatalog(4,7);
            [~,~,cattle]=RoadsenseScenarioCatalog(5,7);
            testCase.verifyNotEqual(bitand(village.EventMask,uint16(8+64)),uint16(0));
            testCase.verifyNotEqual(bitand(urban.EventMask,uint16(2+8)),uint16(0));
            testCase.verifyNotEqual(bitand(merge.EventMask,uint16(4+32)),uint16(0));
            testCase.verifyNotEqual(bitand(market.EventMask,uint16(2+8)),uint16(0));
            testCase.verifyEqual(bitand(cattle.EventMask,uint16(16+64)),uint16(16+64));
        end

        function actorsActivateAndMoveDeterministically(testCase)
            [~,before]=RoadsenseScenarioCatalog(5,5.0);
            [~,during]=RoadsenseScenarioCatalog(5,7.0);
            [~,again]=RoadsenseScenarioCatalog(5,7.0);
            testCase.verifyGreaterThan(during.Count,before.Count);
            testCase.verifyEqual(during,again);
            cattle=find(during.ClassIDs==uint8(9) & during.ValidMask);
            testCase.verifyGreaterThanOrEqual(numel(cattle),2);
            testCase.verifyTrue(any(abs(during.Velocities(cattle,2))>single(1)));
        end

        function completionAndInvalidSelectionAreExplicit(testCase)
            [definition,actors,status]=RoadsenseScenarioCatalog(3,28);
            testCase.verifyTrue(definition.Valid); testCase.verifyTrue(actors.Valid);
            testCase.verifyNotEqual(bitand(status.EventMask,uint16(128)),uint16(0));
            [definition,actors,status]=RoadsenseScenarioCatalog(99,0);
            testCase.verifyFalse(definition.Valid); testCase.verifyFalse(actors.Valid);
            testCase.verifyFalse(status.Valid); testCase.verifyEqual(definition.Count,uint16(0));
        end

        function buildsNativeAndRoadRunnerAssets(testCase)
            native=createRoadsenseDrivingScenarios(); rr=createRoadsenseRoadRunnerHDMaps();
            testCase.verifyEqual(height(native),5); testCase.verifyEqual(height(rr),5);
            testCase.verifyTrue(all(isfile(native.AssetFile))); testCase.verifyTrue(all(isfile(rr.HDMapFile)));
            testCase.verifyEqual(numel(unique(rr.SceneFile)),5);
            testCase.verifyEqual(numel(unique(rr.ScenarioFile)),5);
            loaded=load(native.AssetFile(4),"scenario","metadata");
            testCase.verifyClass(loaded.scenario,"drivingScenario");
            testCase.verifyEqual(loaded.metadata.ScenarioID,uint16(4));
            map=roadrunnerHDMap; read(map,rr.HDMapFile(2));
            testCase.verifyGreaterThanOrEqual(numel(map.Lanes),2);
            mergeMap=roadrunnerHDMap; read(mergeMap,rr.HDMapFile(3));
            testCase.verifyGreaterThanOrEqual(numel(mergeMap.Lanes),3);
        end

        function generatedModelIsReadableTypedAndExecutable(testCase)
            modelPath=createRoadsenseScenarioLibraryModel(); [~,modelName]=fileparts(modelPath);
            load_system(modelPath); cleanup=onCleanup(@() close_system(modelName,0));
            set_param(modelName,"SimulationCommand","update");
            expected={'Bus: RsScenarioDefinitionBus','Bus: RsScenarioActorListBus', ...
                'Bus: RsScenarioStatusBus'};
            outports=find_system(modelName,"SearchDepth",1,"BlockType","Outport");
            ports=zeros(numel(outports),1);
            for index=1:numel(outports); ports(index)=str2double(get_param(outports{index},"Port")); end
            [~,order]=sort(ports); outports=outports(order);
            for index=1:3
                testCase.verifyEqual(get_param(outports{index},"OutDataTypeStr"),expected{index});
            end
            annotations=find_system(modelName,"FindAll","on","Type","annotation");
            testCase.verifyGreaterThanOrEqual(numel(annotations),3);
            canvas=sscanf(get_param(modelName,"ScreenColor"),'[%f, %f, %f]');
            testCase.verifyEqual(canvas,[0.98;0.98;0.98],"AbsTol",1e-12);
            sim(modelName,"StopTime","0.1");
            clear cleanup; close_system(modelName,0);
        end
    end
end
