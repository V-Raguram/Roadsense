classdef testRoadsensePresentationDashboard < matlab.unittest.TestCase
    properties
        Root
        Result
    end
    methods (TestClassSetup)
        function configure(testCase)
            testCase.Root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(genpath(fullfile(testCase.Root,"Models")));
            signals=createSyntheticRoadsenseValidationSignals;
            testCase.Result=extractRoadsenseValidationSignals(signals,1,1,0.4);
        end
    end
    methods (Test)
        function staticDashboardIsExported(testCase)
            folder=fullfile(testCase.Root,"Results","PresentationTest");
            path=createRoadsensePresentationDashboard(testCase.Result, ...
                fullfile(folder,"dashboard.png"));
            testCase.verifyTrue(isfile(path));
            info=dir(path); testCase.verifyGreaterThan(info.bytes,10000);
        end

        function shortVideoIsPlayableArtifact(testCase)
            folder=fullfile(testCase.Root,"Results","PresentationTest");
            path=renderRoadsenseDemonstrationVideo(testCase.Result, ...
                fullfile(folder,"demo.mp4"),FrameRate=5,PlaybackSpeed=4,MaximumFrames=3);
            testCase.verifyTrue(isfile(path));
            info=dir(path); testCase.verifyGreaterThan(info.bytes,1000);
        end

        function invalidSourceIsRejected(testCase)
            testCase.verifyError(@() resolveRoadsensePresentationResult("missing.mat"), ...
                "Roadsense:Presentation:Source");
        end
    end
end
