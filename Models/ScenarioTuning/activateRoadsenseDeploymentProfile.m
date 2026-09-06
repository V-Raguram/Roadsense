function manifest=activateRoadsenseDeploymentProfile(root)
%ACTIVATEROADSENSEDEPLOYMENTPROFILE Apply the persisted deployment overlay.
arguments
    root (1,1) string = string(fileparts(fileparts(fileparts(mfilename("fullpath")))))
end
path=fullfile(root,"Data","Roadsense_DeploymentProfile.mat");
if ~isfile(path); manifest=struct.empty; return; end
loaded=load(path,"manifest"); manifest=loaded.manifest;
profile=RoadsenseTuningProfile(string(manifest.ProfileName));
applyRoadsenseTuningProfile(profile,fullfile(root,"Data","Roadsense_Data.sldd"));
end
