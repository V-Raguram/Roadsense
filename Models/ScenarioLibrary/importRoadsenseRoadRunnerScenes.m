function manifest=importRoadsenseRoadRunnerScenes(projectFolder)
%IMPORTROADSENSEROADRUNNERSCENES Import all five generated maps as RR scenes.
% RoadRunner desktop is launched without display. Existing project content is
% preserved; scene files are written under the supplied RoadRunner project.
arguments
    projectFolder (1,1) string
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
manifest=createRoadsenseRoadRunnerHDMaps();
mapDir=fullfile(root,"Scenarios","RoadRunner");
if ~isfolder(projectFolder)
    installationFolder=defaultRoadRunnerInstallation();
    bootstrapProject=fullfile(installationFolder,"AssetsInstall","Sample Project");
    rrApp=roadrunner(ProjectFolder=bootstrapProject, ...
        InstallationFolder=installationFolder,NoDisplay=true);
    newProject(rrApp,projectFolder);
else
    rrApp=roadrunner(ProjectFolder=projectFolder,NoDisplay=true);
end
cleanup=onCleanup(@() close(rrApp));
for index=1:height(manifest)
    newScene(rrApp);
    [~,sourceName,sourceExtension]=fileparts(manifest.HDMapFile(index));
    importScene(rrApp,fullfile(mapDir,sourceName+sourceExtension),"RoadRunner HD Map");
    saveScene(rrApp,manifest.SceneFile(index));
    copyfile(fullfile(projectFolder,"Scenes",manifest.SceneFile(index)), ...
        fullfile(mapDir,manifest.SceneFile(index)));
end
fprintf("Imported five Roadsense RoadRunner scenes into %s\n",projectFolder);
end

function installationFolder=defaultRoadRunnerInstallation()
installationFolder="C:\Program Files\RoadRunner "+string(matlabRelease.Release)+ ...
    "\bin\win64";
if ~isfolder(installationFolder)
    error("Roadsense:RoadRunner:InstallationNotFound", ...
        "RoadRunner installation was not found at %s.",installationFolder);
end
end
