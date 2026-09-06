function project = setupRoadsense()
%SETUPROADSENSE Configure MATLAB for the Roadsense repository.
%   PROJECT = SETUPROADSENSE adds project source and test directories to the
%   MATLAB path, creates or updates the shared Simulink data dictionary, and
%   returns resolved project paths.

root = fileparts(mfilename("fullpath"));
sharedDataDir = fullfile(root,"Models","SharedData");
modelsDir = fullfile(root,"Models");
testsDir = fullfile(root,"Tests");
dataDir = fullfile(root,"Data");

addpath(genpath(modelsDir));
addpath(genpath(testsDir));
addpath(dataDir);

if ~isfolder(dataDir)
    mkdir(dataDir);
end

dictionaryPath = createRoadsenseDataDictionary( ...
    fullfile(dataDir,"Roadsense_Data.sldd"));
deployment=activateRoadsenseDeploymentProfile(string(root));

project = struct( ...
    "Root",root, ...
    "Models",modelsDir, ...
    "SharedData",sharedDataDir, ...
    "Tests",testsDir, ...
    "DataDictionary",dictionaryPath, ...
    "DeploymentProfile",deployment);

fprintf("Roadsense configured.\nData dictionary: %s\n",dictionaryPath);
end
