classdef testRoadsenseValidationRunner < matlab.unittest.TestCase
    %TESTROADSENSEVALIDATIONRUNNER Benchmark extraction and reporting tests.
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
        function defaultsCoverEveryRequiredScenario(testCase)
            configuration=RoadsenseValidationConfiguration(string(testCase.Root));
            testCase.verifyEqual(configuration.ScenarioIDs,1:5);
            testCase.verifyNumElements(configuration.ScenarioNames,5);
            testCase.verifyEqual(configuration.ContractVersion,uint32(16));
            testCase.verifyFalse(configuration.UseFastRestart);
            testCase.verifyEqual(configuration.InferenceMode,"syntheticColor");
        end

        function extractionPreservesAllAcceptanceMetrics(testCase)
            signals=createSyntheticRoadsenseValidationSignals;
            result=extractRoadsenseValidationSignals(signals,1,3.2,0.4);
            row=result.Metrics;
            testCase.verifyTrue(row.SimulationSucceeded);
            testCase.verifyTrue(row.Completed); testCase.verifyTrue(row.CollisionFree);
            testCase.verifyTrue(row.GoalReached); testCase.verifyTrue(row.Pass);
            testCase.verifyEqual(row.MinimumClearanceM,single(0.65));
            testCase.verifyEqual(row.ReplanCount,uint32(2));
            testCase.verifyEqual(row.MaximumReplanLatencyMs,single(45),"AbsTol",single(1e-5));
            testCase.verifyEqual(row.PipelineReadyPercent,single(80),"AbsTol",single(1e-5));
            testCase.verifyTrue(row.FinalPipelineReady);
            testCase.verifyEqual(row.ReadinessBlockers,"");
            testCase.verifyEqual(result.Timeline.Position(end,1),0.4,"AbsTol",1e-12);
        end

        function dropoutMaskAccumulatesAcrossWholeRun(testCase)
            result=extractRoadsenseValidationSignals( ...
                createSyntheticRoadsenseValidationSignals,5,1,0.4);
            testCase.verifyEqual(result.Metrics.DropoutObservedMask,uint8(7));
        end

        function figuresCsvAndMarkdownCanBeGenerated(testCase)
            result=extractRoadsenseValidationSignals( ...
                createSyntheticRoadsenseValidationSignals,1,1,0.4);
            outputDirectory=fullfile(testCase.Root,"Results","ValidationRunnerTest");
            if ~isfolder(outputDirectory); mkdir(outputDirectory); end
            paths=generateRoadsenseValidationFigures({result},result.Metrics,outputDirectory);
            configuration=RoadsenseValidationConfiguration(string(testCase.Root));
            configuration.OutputDirectory=outputDirectory;
            reportPath=writeRoadsenseValidationReport(result.Metrics,configuration,1);
            writetable(result.Metrics,fullfile(outputDirectory,"validation_summary.csv"));
            testCase.verifyNumElements(paths,2);
            testCase.verifyTrue(all(isfile(paths)));
            testCase.verifyTrue(isfile(reportPath));
            testCase.verifyTrue(isfile(fullfile(outputDirectory,"validation_summary.csv")));
            text=fileread(reportPath);
            testCase.verifySubstring(text,"Scenario completion rate: 100.0%");
        end

        function harnessStoresOnlyLightweightValidationLogs(testCase)
            modelPath=createRoadsenseScenarioClosedLoopHarnessModel();
            [~,modelName]=fileparts(modelPath); load_system(modelPath);
            cleanup=onCleanup(@() close_system(modelName,0));
            expected=["EgoState","LocalPlan","SafeVehicleControl","SafetyStatus", ...
                "IntegrationStatus","SensorStatus","EvaluationStatus", ...
                "PlannerStatus","TrackingStatus","DynamicsStatus","BehaviourCommand"];
            actual=strings(0,1);
            outports=find_system(modelName,"SearchDepth",1,"BlockType","Outport");
            for index=1:numel(outports)
                handles=get_param(outports{index},"PortHandles");
                line=get_param(handles.Inport,"Line");
                sourcePort=get_param(line,"SrcPortHandle");
                if sourcePort~=-1 && strcmp(get_param(sourcePort,"DataLogging"),'on')
                    actual(end+1,1)=string(get_param(sourcePort,"DataLoggingName")); %#ok<AGROW>
                end
            end
            stack=modelName+"/Roadsense Autonomous Driving Stack";
            stackPorts=get_param(stack,"PortHandles");
            for port=[8 9 10 11]
                sourcePort=stackPorts.Outport(port);
                if strcmp(get_param(sourcePort,"DataLogging"),'on')
                    actual(end+1,1)=string(get_param(sourcePort,"DataLoggingName")); %#ok<AGROW>
                end
            end
            testCase.verifyEqual(sort(actual),sort(expected.'));
            testCase.verifyFalse(any(actual=="SemanticGrid"));
            testCase.verifyFalse(any(actual=="FusedTracks"));
            clear cleanup; close_system(modelName,0);
        end

        function invalidScenarioSelectionIsRejectedBeforeSimulation(testCase)
            testCase.verifyError(@() runRoadsenseValidationSuite( ...
                ScenarioIDs=6,GenerateFigures=false,ShowProgress=false), ...
                "Roadsense:Validation:ScenarioID");
        end
    end
end
