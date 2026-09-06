classdef testRoadsenseSubmissionPackaging < matlab.unittest.TestCase
    properties
        Root
        EvidenceDirectory
    end
    methods (TestClassSetup)
        function configure(testCase)
            testCase.Root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            testCase.EvidenceDirectory=fullfile(testCase.Root,"Results","SubmissionPackagingTest","accepted");
            if ~isfolder(testCase.EvidenceDirectory); mkdir(testCase.EvidenceDirectory); end
            rows=cell(1,5);
            names=RoadsenseValidationConfiguration().ScenarioNames;
            for id=1:5
                row=extractRoadsenseValidationSignals( ...
                    createSyntheticRoadsenseValidationSignals,id,1,1).Metrics;
                row.ScenarioID=uint16(id); row.ScenarioName=names(id);
                rows{id}=row;
            end
            summary=vertcat(rows{:});
            writetable(summary,fullfile(testCase.EvidenceDirectory,"validation_summary.csv"));
        end
    end
    methods (Test)
        function technicalReportUsesMeasuredEvidence(testCase)
            path=generateRoadsenseTechnicalReport(testCase.EvidenceDirectory, ...
                fullfile(testCase.Root,"Results","SubmissionPackagingTest","report.md"));
            testCase.verifyTrue(isfile(path));
            content=string(fileread(path));
            testCase.verifySubstring(content,"Required scenario results");
            testCase.verifySubstring(content,"Unmarked Village Road");
        end

        function packageRejectsOneFailedScenario(testCase)
            summary=readtable(fullfile(testCase.EvidenceDirectory,"validation_summary.csv"));
            summary.Pass(3)=false;
            folder=fullfile(testCase.Root,"Results","SubmissionPackagingTest","failed");
            if ~isfolder(folder); mkdir(folder); end
            writetable(summary,fullfile(folder,"validation_summary.csv"));
            testCase.verifyError(@() buildRoadsenseSubmissionPackage(folder, ...
                fullfile(testCase.Root,"Results","SubmissionPackagingTest","must_not_build"), ...
                GenerateVideos=false,CreateZip=false),"Roadsense:Submission:Evidence");
        end
    end
end
