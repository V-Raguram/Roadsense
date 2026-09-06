function manifest=createRoadsenseDrivingScenarios()
%CREATEROADSENSEDRIVINGSCENARIOS Build five native drivingScenario assets.
componentDir=fileparts(mfilename("fullpath"));
root=fileparts(fileparts(componentDir));
assetDir=fullfile(root,"Scenarios","MATLAB");
if ~isfolder(assetDir); mkdir(assetDir); end

identifiers=["unmarked_village_road","uncontrolled_urban_intersection", ...
    "highway_merge_slow_vehicles","dense_market_mixed_traffic", ...
    "sudden_cattle_crossing"];
displayNames=["Unmarked Village Road","Uncontrolled Urban Intersection", ...
    "Highway Merge With Slow Vehicles","Dense Market Mixed Traffic", ...
    "Sudden Cattle Crossing"];
files=strings(5,1); durations=zeros(5,1); routePoints=zeros(5,1);
actorCounts=zeros(5,1); requiredEvents=strings(5,1);

for scenarioID=1:5
    [definition,~,status,plans]=RoadsenseScenarioCatalog(scenarioID,0);
    duration=double(status.Duration);
    route=definition.WorldPositions(1:double(definition.Count),:);
    scenario=drivingScenario("SampleTime",0.10,"StopTime",duration);
    addRoadNetwork(scenario,scenarioID,route);

    ego=vehicle(scenario,"ClassID",1,"Name","Roadsense Ego", ...
        "Length",4.35,"Width",1.82,"Height",1.48,"PlotColor",[0.05 0.35 0.85]);
    trajectory(ego,route,TimeOfArrival=linspace(0,duration,size(route,1)).');

    for actorIndex=1:numel(plans)
        plan=plans(actorIndex); t0=plan.Times(1); t1=plan.Times(end);
        actorArgs={"ClassID",double(plan.ClassID), ...
            "Name",sprintf("%s %03d",className(plan.ClassID),plan.ActorID), ...
            "Length",double(plan.Dimensions(1)),"Width",double(plan.Dimensions(2)), ...
            "Height",double(plan.Dimensions(3)),"PlotColor",classColor(plan.ClassID), ...
            "ExitTime",t1};
        if t0>0; actorArgs=[actorArgs {"EntryTime",t0}]; end %#ok<AGROW>
        roadUser=actor(scenario,actorArgs{:});
        if norm(plan.Waypoints(end,:)-plan.Waypoints(1,:))<1e-6
            roadUser.Position=plan.Waypoints(1,:);
        else
            trajectory(roadUser,plan.Waypoints,TimeOfArrival=plan.Times(:));
        end
    end

    metadata=struct("ScenarioID",uint16(scenarioID),"Identifier",identifiers(scenarioID), ...
        "DisplayName",displayNames(scenarioID),"Duration",duration, ...
        "RoutePointCount",definition.Count,"TrafficActorCount",uint16(numel(plans)), ...
        "EventBits",eventDescription(scenarioID),"Definition",definition);
    files(scenarioID)=fullfile(assetDir,identifiers(scenarioID)+".mat");
    save(files(scenarioID),"scenario","metadata");
    durations(scenarioID)=duration; routePoints(scenarioID)=size(route,1);
    actorCounts(scenarioID)=numel(plans); requiredEvents(scenarioID)=metadata.EventBits;
end

manifest=table((1:5).',displayNames.',identifiers.',durations,routePoints, ...
    actorCounts,requiredEvents,files,'VariableNames',{'ScenarioID','Name', ...
    'Identifier','Duration_s','RoutePoints','TrafficActors','RequiredEvents','AssetFile'});
writetable(manifest,fullfile(assetDir,"scenario_manifest.csv"));
fprintf("Generated five drivingScenario assets in %s\n",assetDir);
end

function addRoadNetwork(scenario,id,route)
switch id
    case 1
        road(scenario,route,6.0,"Name","Unmarked village road");
    case 2
        road(scenario,route,8.0,"Name","East-west urban road");
        road(scenario,[0 -45 0;0 45 0],7.0,"Name","North-south urban road");
    case 3
        road(scenario,route,10.5,"Name","Highway carriageway");
        road(scenario,[0 -8 0;55 -7 0;100 0 0],4.0,"Name","Informal merge ramp");
    case 4
        road(scenario,route,5.5,"Name","Market street");
        road(scenario,[25 -12 0;25 12 0],3.5,"Name","Market side street");
        road(scenario,[67 -10 0;67 10 0],3.5,"Name","Vendor access street");
    case 5
        road(scenario,route,7.0,"Name","Rural connector");
end
end

function name=className(classID)
names=["Unknown","Car","Truck","Bus","AutoRickshaw","Motorcycle", ...
    "Bicycle","Pedestrian","Pushcart","Animal","StaticObstacle"];
index=min(max(double(classID)+1,1),numel(names)); name=names(index);
end

function color=classColor(classID)
palette=[0.45 0.45 0.45; 0.15 0.45 0.85; 0.55 0.25 0.10; ...
    0.85 0.30 0.15; 0.95 0.60 0.10; 0.55 0.20 0.75; ...
    0.10 0.65 0.55; 0.10 0.75 0.20; 0.75 0.45 0.10; ...
    0.60 0.38 0.18; 0.25 0.25 0.25];
index=min(max(double(classID)+1,1),size(palette,1)); color=palette(index,:);
end

function text=eventDescription(id)
values=["pedestrian + animal + degraded surface", ...
    "uncontrolled intersection + pedestrian + occlusion", ...
    "informal merge + slow vehicle", ...
    "pedestrian + dense unstructured traffic + occlusion", ...
    "sudden animal crossing + degraded surface"];
text=values(id);
end
