classdef testRoadsenseScenarioTuning < matlab.unittest.TestCase
    %TESTROADSENSESCENARIOTUNING Profiles, ranking, and physical timing tests.
    properties
        Root
    end
    methods (TestClassSetup)
        function configure(testCase)
            testCase.Root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            addpath(fullfile(testCase.Root,"Data"));
            createRoadsenseDataDictionary(fullfile(testCase.Root,"Data","Roadsense_Data.sldd"));
        end
    end
    methods (Test)
        function profilesNeverAlterAcceptanceThresholds(testCase)
            forbidden=["RsScenarioMinimumClearance","RsScenarioMaximumReplanLatency", ...
                "RsScenarioMaximumPathCurvatureRMS","RsScenarioMaximumJerk", ...
                "RsScenarioMaximumCrossTrackError","RsScenarioMinimumReadyRatio"];
            for name=["baseline","balanced","cautious"]
                profile=RoadsenseTuningProfile(name);
                testCase.verifyEmpty(intersect(string(fieldnames(profile.Values)),forbidden));
                testCase.verifyEqual(profile.ContractVersion,uint32(16));
            end
        end

        function profileApplicationCanBeRolledBackExactly(testCase)
            path=fullfile(testCase.Root,"Data","Roadsense_Data.sldd");
            before=readValue(path,"RsControllerAccelerationSlew");
            snapshot=applyRoadsenseTuningProfile(RoadsenseTuningProfile("balanced"),path);
            during=readValue(path,"RsControllerAccelerationSlew");
            restoreRoadsenseTuningSnapshot(snapshot); after=readValue(path,"RsControllerAccelerationSlew");
            testCase.verifyEqual(during,single(3.5));
            testCase.verifyEqual(after,before);
        end

        function safetyFirstScorePrefersAcceptedRun(testCase)
            good=extractRoadsenseValidationSignals(createSyntheticRoadsenseValidationSignals,1,1,1).Metrics;
            bad=good; bad.CollisionFree=false; bad.Completed=false; bad.GoalReached=false; bad.Pass=false;
            a=scoreRoadsenseTuningMetrics(good,"good"); b=scoreRoadsenseTuningMetrics(bad,"bad");
            testCase.verifyLessThan(a.Objective,b.Objective);
        end

        function scenarioDurationsAllowPhysicalRouteTraversal(testCase)
            for id=1:5
                [definition,~,status]=RoadsenseScenarioCatalog(id,0);
                route=definition.WorldPositions(definition.ValidMask,1:2);
                routeLength=sum(vecnorm(diff(route),2,2));
                minimumTime=routeLength/max(double(definition.DesiredSpeed),0.1);
                testCase.verifyGreaterThanOrEqual(double(status.Duration),minimumTime+3);
            end
        end

        function comparisonArtifactsAreGenerated(testCase)
            row=extractRoadsenseValidationSignals(createSyntheticRoadsenseValidationSignals,1,1,1).Metrics;
            comparison=[scoreRoadsenseTuningMetrics(row,"baseline"); ...
                scoreRoadsenseTuningMetrics(row,"balanced")];
            folder=fullfile(testCase.Root,"Results","ScenarioTuningTest");
            if ~isfolder(folder); mkdir(folder); end
            figurePath=generateRoadsenseTuningFigure(comparison,folder);
            options=struct("OutputDirectory",string(folder),"InferenceMode","syntheticColor", ...
                "ScenarioIDs",1:5);
            reportPath=writeRoadsenseTuningReport(comparison,"balanced",options,1.2);
            testCase.verifyTrue(isfile(figurePath)); testCase.verifyTrue(isfile(reportPath));
            testCase.verifySubstring(fileread(reportPath),"Evaluator thresholds are never modified");
        end

        function invalidProfileIsRejected(testCase)
            testCase.verifyError(@() runRoadsenseScenarioTuningSuite(Profiles="unsafe"), ...
                "Roadsense:Tuning:Profile");
        end

        function deploymentFreezeRejectsIncompleteEvidence(testCase)
            folder=fullfile(testCase.Root,"Results","ScenarioTuning","BalancedSmokeFinal","balanced");
            testCase.verifyError(@() freezeRoadsenseDeploymentProfile( ...
                "balanced",folder),"Roadsense:Deployment:Evidence");
        end
    end
end

function value=readValue(path,name)
dictionary=Simulink.data.dictionary.open(path); cleanup=onCleanup(@() close(dictionary));
entry=getEntry(getSection(dictionary,"Design Data"),name); parameter=getValue(entry);
value=parameter.Value; clear cleanup;
end
