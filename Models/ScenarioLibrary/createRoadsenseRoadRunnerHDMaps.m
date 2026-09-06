function manifest=createRoadsenseRoadRunnerHDMaps()
%CREATEROADSENSEROADRUNNERHDMAPS Build two importable RoadRunner HD maps.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
assetDir=fullfile(root,"Scenarios","RoadRunner");
if ~isfolder(assetDir); mkdir(assetDir); end

[village,~,~,~]=RoadsenseScenarioCatalog(1,0);
villageRoute=village.WorldPositions(1:double(village.Count),:);
villageMap=roadrunnerHDMap(Author="Roadsense");
villageMap.Lanes=roadrunner.hdmap.Lane(ID="VillageSharedRoad", ...
    Geometry=villageRoute,LaneType="Driving",TravelDirection="Bidirectional");
[left,right]=boundaries(villageRoute,3.0,"VillageLeftEdge","VillageRightEdge");
villageMap.LaneBoundaries=[left right];
villageFile=fullfile(assetDir,"roadsense_unmarked_village.rrhd");
write(villageMap,villageFile);

urbanMap=roadrunnerHDMap(Author="Roadsense");
eastWest=[(-45:2:55).' zeros(51,1) zeros(51,1)];
northSouth=[zeros(46,1) (-45:2:45).' zeros(46,1)];
urbanMap.Lanes=[ ...
    roadrunner.hdmap.Lane(ID="UrbanEastWest",Geometry=eastWest, ...
        LaneType="Driving",TravelDirection="Bidirectional") ...
    roadrunner.hdmap.Lane(ID="UrbanNorthSouth",Geometry=northSouth, ...
        LaneType="Driving",TravelDirection="Bidirectional")];
[ewLeft,ewRight]=boundaries(eastWest,4.0,"EWNorthEdge","EWSouthEdge");
[nsLeft,nsRight]=boundaries(northSouth,3.5,"NSWestEdge","NSEastEdge");
urbanMap.LaneBoundaries=[ewLeft ewRight nsLeft nsRight];
urbanFile=fullfile(assetDir,"roadsense_uncontrolled_urban_intersection.rrhd");
write(urbanMap,urbanFile);

manifest=table([1;2],["Unmarked Village Road";"Uncontrolled Urban Intersection"], ...
    ["wide bidirectional lane without centre markings"; ...
     "crossing bidirectional roads without signal control"], ...
    [string(villageFile);string(urbanFile)], ...
    'VariableNames',{'ScenarioID','Name','RoadRunnerIntent','HDMapFile'});
writetable(manifest,fullfile(assetDir,"roadrunner_scene_manifest.csv"));
fprintf("Generated two RoadRunner HD maps in %s\n",assetDir);
end

function [left,right]=boundaries(centerline,halfWidth,leftID,rightID)
delta=gradient(centerline(:,1:2)); length2=hypot(delta(:,1),delta(:,2));
length2=max(length2,eps); normal=[-delta(:,2)./length2 delta(:,1)./length2];
leftGeometry=[centerline(:,1:2)+halfWidth*normal centerline(:,3)];
rightGeometry=[centerline(:,1:2)-halfWidth*normal centerline(:,3)];
left=roadrunner.hdmap.LaneBoundary(ID=leftID,Geometry=leftGeometry);
right=roadrunner.hdmap.LaneBoundary(ID=rightID,Geometry=rightGeometry);
end
