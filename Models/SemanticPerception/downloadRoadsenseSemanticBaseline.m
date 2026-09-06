function modelPath = downloadRoadsenseSemanticBaseline()
%DOWNLOADROADSENSESEMANTICBASELINE Download MathWorks' CamVid SegNet model.
%   The 106 MB baseline is used to verify real deep-learning inference and
%   the complete Simulink data path before the network is fine-tuned on IDD.

componentDir = fileparts(mfilename("fullpath"));
root = fileparts(fileparts(componentDir));
networkDir = fullfile(root,"Data","Networks");
if ~isfolder(networkDir)
    mkdir(networkDir);
end

modelPath = fullfile(networkDir,"segnetVGG16CamVid.mat");
url = "https://ssd.mathworks.com/supportfiles/vision/data/segnetVGG16CamVid.mat";

if ~isfile(modelPath)
    fprintf("Downloading semantic baseline to %s ...\n",modelPath);
    websave(modelPath,url);
end

contents = whos("-file",modelPath);
assert(any(string({contents.name}) == "net"), ...
    "Roadsense:SemanticPerception:InvalidBaseline", ...
    "Downloaded baseline does not contain the expected variable 'net'.");
fprintf("Semantic baseline ready: %s\n",modelPath);
end

