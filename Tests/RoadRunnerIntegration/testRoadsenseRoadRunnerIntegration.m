classdef testRoadsenseRoadRunnerIntegration < matlab.unittest.TestCase
    %TESTROADSENSEROADRUNNERINTEGRATION Static co-simulation interface tests.
    properties
        Root
    end
    methods (TestClassSetup)
        function configure(testCase)
            testFile=mfilename("fullpath");
            testCase.Root=fileparts(fileparts(fileparts(testFile)));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            addpath(testCase.Root); addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data", ...
                "Roadsense_Data.sldd"));
        end
    end
    methods (Test)
        function modelUsesNativeRoadRunnerMessageBlocks(testCase)
            path=createRoadsenseRoadRunnerIntegrationModel();
            [~,model]=fileparts(path); load_system(path);
            cleanup=onCleanup(@() close_system(model,0));
            testCase.verifyNumElements(find_system(model,"SearchDepth",1, ...
                "BlockType","RoadRunnerScenario"),1);
            testCase.verifyNumElements(find_system(model,"SearchDepth",1, ...
                "BlockType","RoadRunnerScenarioReader"),1);
            testCase.verifyNumElements(find_system(model,"SearchDepth",1, ...
                "BlockType","RoadRunnerScenarioWriter"),1);
            testCase.verifyNumElements(find_system(model,"SearchDepth",1, ...
                "BlockType","Receive"),1);
            testCase.verifyNumElements(find_system(model,"SearchDepth",1, ...
                "BlockType","Send"),1);
            reference=find_system(model,"SearchDepth",1,"BlockType","ModelReference");
            testCase.verifyEqual(get_param(reference{1},"ModelName"), ...
                'Roadsense_ScenarioClosedLoopHarness');
            writer=find_system(model,"SearchDepth",1, ...
                "BlockType","RoadRunnerScenarioWriter");
            testCase.verifyEqual(get_param(writer{1},"TopicType"), ...
                'Actor Pose (Driving Scenario compatible)');
            workspace=get_param(model,"ModelWorkspace");
            testCase.verifyEqual(workspace.evalin("RsRoadRunnerScenarioID"),uint8(1));
            set_param(model,"SimulationCommand","update");
            clear cleanup; close_system(model,0);
        end

        function manifestCoversFiveDistinctVisualStages(testCase)
            manifest=createRoadsenseRoadRunnerHDMaps();
            testCase.verifyEqual(height(manifest),5);
            testCase.verifyEqual(numel(unique(manifest.SceneFile)),5);
            testCase.verifyEqual(numel(unique(manifest.ScenarioFile)),5);
            testCase.verifyTrue(all(isfile(manifest.HDMapFile)));
        end
    end
end
