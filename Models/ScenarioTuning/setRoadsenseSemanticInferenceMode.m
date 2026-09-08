function previousMode=setRoadsenseSemanticInferenceMode(mode,options)
%SETROADSENSESEMANTICINFERENCEMODE Select full or synthetic-camera backend.
arguments
    mode (1,1) string {mustBeMember(mode,["network","syntheticColor"])}
    options.Persist (1,1) logical = true
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
addpath(genpath(fullfile(root,"Models"))); modelName="Roadsense_SemanticPerception";
modelPath=fullfile(root,"Models","SemanticPerception",modelName+".slx");
if ~isfile(modelPath); createRoadsenseSemanticPerceptionModel(); end
wasLoaded=bdIsLoaded(modelName); if ~wasLoaded; load_system(modelPath); end
block=modelName+"/Semantic AI and Postprocessor";
parameters=get_param(block,"ObjectParameters");
if ~isfield(parameters,"InferenceMode")
    close_system(modelName,0); createRoadsenseSemanticPerceptionModel();
    load_system(modelPath); wasLoaded=false;
end
previousMode=string(get_param(block,"InferenceMode"));
if previousMode~=mode
    set_param(block,"InferenceMode",mode);
    if options.Persist; save_system(modelName,modelPath); end
end
% A transient selection must remain loaded for the parent model or
% RoadRunner actor behavior that is about to execute. Persistent selections
% retain the original load/close behavior used by configuration utilities.
if ~wasLoaded && options.Persist; close_system(modelName,0); end
end
