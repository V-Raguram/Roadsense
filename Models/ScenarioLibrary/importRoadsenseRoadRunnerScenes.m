function importRoadsenseRoadRunnerScenes(projectFolder)
%IMPORTROADSENSEROADRUNNERSCENES Import the two generated maps as RR scenes.
% RoadRunner desktop is launched without display. Existing project content is
% preserved; scene files are written under the supplied RoadRunner project.
arguments
    projectFolder (1,1) string
end
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
createRoadsenseRoadRunnerHDMaps();
mapDir=fullfile(root,"Scenarios","RoadRunner");
if ~isfolder(projectFolder)
    rrApp=roadrunner;
    newProject(rrApp,projectFolder);
else
    rrApp=roadrunner(projectFolder,"NoDisplay",true);
end
cleanup=onCleanup(@() close(rrApp));
sourceFiles=["roadsense_unmarked_village.rrhd", ...
    "roadsense_uncontrolled_urban_intersection.rrhd"];
sceneFiles=["Roadsense_Unmarked_Village.rrscene", ...
    "Roadsense_Uncontrolled_Urban_Intersection.rrscene"];
for index=1:2
    newScene(rrApp);
    importScene(rrApp,fullfile(mapDir,sourceFiles(index)),"RoadRunner HD Map");
    saveScene(rrApp,sceneFiles(index));
end
fprintf("Imported Roadsense RoadRunner scenes into %s\n",projectFolder);
end
