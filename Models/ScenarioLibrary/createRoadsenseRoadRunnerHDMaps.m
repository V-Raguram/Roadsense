function manifest=createRoadsenseRoadRunnerHDMaps()
%CREATEROADSENSEROADRUNNERHDMAPS Build all five importable RoadRunner maps.
componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
assetDir=fullfile(root,"Scenarios","RoadRunner");
if ~isfolder(assetDir); mkdir(assetDir); end

names=["Unmarked Village Road";"Uncontrolled Urban Intersection"; ...
    "Highway Merge With Slow Vehicles";"Dense Market Mixed Traffic"; ...
    "Sudden Cattle Crossing"];
intents=["curved shared village road without centre markings"; ...
    "wide crossing roads without signal control"; ...
    "two-lane highway with an informal converging merge ramp"; ...
    "narrow shared market street for dense mixed traffic"; ...
    "open rural road with sufficient sight distance for cattle response"];
baseNames=["roadsense_unmarked_village"; ...
    "roadsense_uncontrolled_urban_intersection"; ...
    "roadsense_highway_slow_vehicle_merge"; ...
    "roadsense_dense_market_mixed_traffic"; ...
    "roadsense_sudden_cattle_crossing"];
files=strings(5,1);

for scenarioID=1:5
    map=roadrunnerHDMap(Author="Roadsense");
    switch scenarioID
        case 1
            [definition,~,~,~]=RoadsenseScenarioCatalog(1,0);
            route=definition.WorldPositions(1:double(definition.Count),:);
            [lanes,edges]=laneWithEdges(route,3.0,"VillageSharedRoad", ...
                "VillageLeftEdge","VillageRightEdge","Bidirectional");
        case 2
            eastWest=[(-55:2:65).' zeros(61,1) zeros(61,1)];
            northSouth=[zeros(56,1) (-55:2:55).' zeros(56,1)];
            [laneEW,edgeEW]=laneWithEdges(eastWest,4.0,"UrbanEastWest", ...
                "EWNorthEdge","EWSouthEdge","Bidirectional");
            [laneNS,edgeNS]=laneWithEdges(northSouth,3.5,"UrbanNorthSouth", ...
                "NSWestEdge","NSEastEdge","Bidirectional");
            lanes=[laneEW laneNS]; edges=[edgeEW edgeNS];
        case 3
            x=(0:5:280).';
            mainLeft=[x 3.5*ones(size(x)) zeros(size(x))];
            mainRight=[x zeros(size(x)) zeros(size(x))];
            rampX=(0:3:105).';
            rampY=-7+7*smoothstep(rampX/max(rampX));
            ramp=[rampX rampY zeros(size(rampX))];
            [lane1,edge1]=laneWithEdges(mainLeft,1.75,"HighwayPassingLane", ...
                "HighwayPassingLeft","HighwayPassingRight","Forward");
            [lane2,edge2]=laneWithEdges(mainRight,1.75,"HighwaySlowLane", ...
                "HighwaySlowLeft","HighwaySlowRight","Forward");
            [lane3,edge3]=laneWithEdges(ramp,1.75,"InformalMergeRamp", ...
                "MergeRampLeft","MergeRampRight","Forward");
            lanes=[lane1 lane2 lane3]; edges=[edge1 edge2 edge3];
        case 4
            [definition,~,~,~]=RoadsenseScenarioCatalog(4,0);
            route=definition.WorldPositions(1:double(definition.Count),:);
            [lanes,edges]=laneWithEdges(route,3.0,"MarketSharedStreet", ...
                "MarketShopEdgeA","MarketShopEdgeB","Bidirectional");
        case 5
            [definition,~,~,~]=RoadsenseScenarioCatalog(5,0);
            route=definition.WorldPositions(1:double(definition.Count),:);
            [lanes,edges]=laneWithEdges(route,4.0,"CattleCrossingRoad", ...
                "CattleRoadLeftEdge","CattleRoadRightEdge","Bidirectional");
    end
    map.Lanes=lanes; map.LaneBoundaries=edges;
    files(scenarioID)=string(fullfile(assetDir,baseNames(scenarioID)+".rrhd"));
    write(map,files(scenarioID));
end

manifest=table((1:5).',names,intents,files,baseNames+".rrscene", ...
    baseNames+".rrscenario",'VariableNames',{'ScenarioID','Name', ...
    'RoadRunnerIntent','HDMapFile','SceneFile','ScenarioFile'});
portable=manifest;
portable.HDMapFile="Scenarios/RoadRunner/"+baseNames+".rrhd";
writetable(portable,fullfile(assetDir,"roadrunner_scene_manifest.csv"));
fprintf("Generated five RoadRunner HD maps in %s\n",assetDir);
end

function value=smoothstep(value)
value=value.^2.*(3-2*value);
end

function [lane,edges]=laneWithEdges(centerline,halfWidth,laneID,leftID,rightID,direction)
lane=roadrunner.hdmap.Lane(ID=laneID,Geometry=centerline, ...
    LaneType="Driving",TravelDirection=direction);
[left,right]=boundaries(centerline,halfWidth,leftID,rightID);
edges=[left right];
end

function [left,right]=boundaries(centerline,halfWidth,leftID,rightID)
delta=gradient(centerline(:,1:2)); length2=hypot(delta(:,1),delta(:,2));
length2=max(length2,eps); normal=[-delta(:,2)./length2 delta(:,1)./length2];
leftGeometry=[centerline(:,1:2)+halfWidth*normal centerline(:,3)];
rightGeometry=[centerline(:,1:2)-halfWidth*normal centerline(:,3)];
left=roadrunner.hdmap.LaneBoundary(ID=leftID,Geometry=leftGeometry);
right=roadrunner.hdmap.LaneBoundary(ID=rightID,Geometry=rightGeometry);
end
