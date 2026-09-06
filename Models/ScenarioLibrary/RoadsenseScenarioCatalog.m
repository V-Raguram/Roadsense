function [scenario,actors,status,plans]=RoadsenseScenarioCatalog(scenarioID,currentTime)
%ROADSENSESCENARIOCATALOG Deterministic Indian-road scenario definitions.
arguments
    scenarioID (1,1) double
    currentTime (1,1) double = 0
end
maximumRoutePoints=256; maximumActors=64;
[name,duration,route,speeds,settings,plans,eventMask]=definition(round(scenarioID),currentTime);
count=size(route,1); world=zeros(maximumRoutePoints,3); world(1:count,:)=route;
recommended=zeros(maximumRoutePoints,1,"single"); recommended(1:count)=single(speeds(:));
routeMask=false(maximumRoutePoints,1); routeMask(1:count)=true;
valid=~strcmp(name,"Unsupported");
scenario=struct("Timestamp",currentTime,"ScenarioID",uint16(max(0,round(scenarioID))), ...
    "Count",uint16(count),"WorldPositions",world,"RecommendedSpeeds",recommended, ...
    "ValidMask",routeMask,"DesiredSpeed",single(settings.DesiredSpeed), ...
    "SpeedLimit",single(settings.SpeedLimit),"GoalRadius",single(settings.GoalRadius), ...
    "AllowObstacleAvoidance",settings.AllowAvoidance, ...
    "UncontrolledIntersection",settings.UncontrolledIntersection, ...
    "OccludedArea",settings.OccludedArea,"MergeRequired",settings.MergeRequired, ...
    "InitialPosition",settings.InitialPosition(:),"InitialYaw",settings.InitialYaw, ...
    "InitialSpeed",single(settings.InitialSpeed), ...
    "FrictionCoefficient",single(settings.Friction),"Grade",single(settings.Grade), ...
    "Bank",single(settings.Bank),"RollingResistance",single(settings.RollingResistance), ...
    "Valid",valid);

actorIDs=zeros(maximumActors,1,"uint32"); classIDs=zeros(maximumActors,1,"uint8");
positions=zeros(maximumActors,3); velocities=zeros(maximumActors,3,"single");
yaws=zeros(maximumActors,1,"single"); dimensions=zeros(maximumActors,3,"single");
actorMask=false(maximumActors,1); active=0;
for index=1:numel(plans)
    [isActive,position,velocity,yaw]=samplePlan(plans(index),currentTime);
    if ~isActive; continue; end
    active=active+1; if active>maximumActors; break; end
    actorIDs(active)=plans(index).ActorID; classIDs(active)=plans(index).ClassID;
    positions(active,:)=position; velocities(active,:)=single(velocity);
    yaws(active)=single(yaw); dimensions(active,:)=plans(index).Dimensions;
    actorMask(active)=true;
end
overflow=active>maximumActors; active=min(active,maximumActors);
actors=struct("Timestamp",currentTime,"Count",uint16(active),"ActorIDs",actorIDs, ...
    "ClassIDs",classIDs,"Positions",positions,"Velocities",velocities, ...
    "Yaws",yaws,"Dimensions",dimensions,"ValidMask",actorMask, ...
    "Overflow",overflow,"Valid",valid && isfinite(currentTime));
if currentTime>=duration; eventMask=bitor(eventMask,uint16(128)); end
status=struct("Timestamp",currentTime,"ScenarioID",uint16(max(0,round(scenarioID))), ...
    "ElapsedTime",single(max(currentTime,0)),"Duration",single(duration), ...
    "EventMask",eventMask,"ActiveActorCount",uint16(active),"Valid",valid);
end

function [name,duration,route,speeds,s,p,eventMask]=definition(id,time)
s=struct("DesiredSpeed",6,"SpeedLimit",8,"GoalRadius",2, ...
    "AllowAvoidance",true,"UncontrolledIntersection",false,"OccludedArea",false, ...
    "MergeRequired",false,"InitialPosition",[0;0;0],"InitialYaw",0, ...
    "InitialSpeed",0,"Friction",0.75,"Grade",0,"Bank",0, ...
    "RollingResistance",0.018);
p=repmat(emptyPlan(),0,1); eventMask=uint16(1); name="Unsupported"; duration=0;
switch id
    case 1 % Unmarked village road
        name="Unmarked Village Road"; duration=28;
        % The village route terminates just beyond the oncoming-vehicle
        % conflict zone.  Sub-metre samples preserve a dense reference while
        % making the goal represent safe passage rather than an arbitrary
        % empty-road tail.
        x=(0:0.6:18).'; y=1.4*sin(x/24); route=[x y zeros(size(x))];
        speeds=6-1.5*(abs(y)>1); s.InitialYaw=atan2(y(2)-y(1),1);
        s.GoalRadius=single(3);
        s.OccludedArea=true; s.Friction=0.62; eventMask=bitor(uint16(1),uint16(2+64));
        p=[makePlan(101,4,[2.8 1.4 1.8],[0 18],[100 1.3 0;20 1.3 0]); ...
           makePlan(102,6,[1.8 0.6 1.2],[0 22],[18 -1.4 0;82 -1.4 0]); ...
           makePlan(103,7,[0.6 0.6 1.75],[5 11],[55 -5 0;55 5 0]); ...
           makePlan(104,8,[2.2 1.1 1.6],[0 22],[72 -2.2 0;72 -2.2 0]); ...
           makePlan(105,9,[1.8 0.7 1.4],[10 17],[92 4 0;88 -4 0])];
        if time>=5 && time<=11; eventMask=bitor(eventMask,uint16(8)); end
        if time>=10 && time<=17; eventMask=bitor(eventMask,uint16(16)); end
    case 2 % Uncontrolled urban intersection
        name="Uncontrolled Urban Intersection"; duration=28;
        x=(-45:1:11).'; route=[x zeros(size(x)) zeros(size(x))]; speeds=4*ones(size(x));
        s.InitialPosition=[-45;0;0]; s.DesiredSpeed=4; s.SpeedLimit=7;
        s.GoalRadius=single(12);
        s.UncontrolledIntersection=true; s.OccludedArea=true;
        eventMask=bitor(uint16(1),uint16(2));
        p=[makePlan(201,2,[8 2.5 3.2],[2 14],[0 -42 0;0 42 0]); ...
           makePlan(202,4,[2.8 1.4 1.8],[0 16],[-28 1 0;38 1 0]); ...
           makePlan(203,5,[2.1 0.8 1.3],[4 11],[25 -28 0;-14 25 0]); ...
           makePlan(204,7,[0.6 0.6 1.75],[6 11],[-6 -8 0;-6 8 0]); ...
           makePlan(205,7,[0.6 0.6 1.75],[8 13],[7 8 0;7 -8 0])];
        if time>=6 && time<=13; eventMask=bitor(eventMask,uint16(8)); end
    case 3 % Highway merge with slow vehicles
        name="Highway Merge With Slow Vehicles"; duration=28;
        x=(0:5:250).'; route=[x zeros(size(x)) zeros(size(x))]; speeds=16*ones(size(x));
        s.DesiredSpeed=16; s.SpeedLimit=22; s.MergeRequired=true; s.Friction=0.85;
        s.GoalRadius=single(10);
        eventMask=bitor(uint16(1),uint16(4+32));
        p=[makePlan(301,3,[10.5 2.6 3.2],[0 28],[45 0 0;185 0 0]); ...
           makePlan(302,1,[4.4 1.8 1.6],[2 10],[20 -6 0;90 0 0]); ...
           makePlan(303,2,[8 2.5 3.2],[0 28],[85 3.5 0;260 3.5 0]); ...
           makePlan(304,5,[2.1 0.8 1.3],[3 20],[25 0.8 0;210 0.8 0])];
    case 4 % Dense market
        name="Dense Market Mixed Traffic"; duration=46;
        x=(0:2:90).'; route=[x 0.5*sin(x/12) zeros(size(x))]; speeds=2.5*ones(size(x));
        s.DesiredSpeed=2.5; s.SpeedLimit=5; s.OccludedArea=true; s.Friction=0.68;
        s.GoalRadius=single(8);
        eventMask=bitor(uint16(1),uint16(2+8));
        p=[makePlan(401,8,[2.2 1.1 1.6],[0 24],[22 -2.1 0;22 -2.1 0]); ...
           makePlan(402,8,[2.2 1.1 1.6],[0 24],[48 2.0 0;48 2.0 0]); ...
           makePlan(403,4,[2.8 1.4 1.8],[0 24],[35 1.2 0;78 1.2 0]); ...
           makePlan(404,7,[0.6 0.6 1.75],[9 15],[28 -4 0;34 4 0]); ...
           makePlan(405,7,[0.6 0.6 1.75],[20 27],[55 4 0;50 -4 0]); ...
           makePlan(406,6,[1.8 0.6 1.2],[0 46],[10 -1 0;100 -1 0]); ...
           makePlan(407,5,[2.1 0.8 1.3],[5 40],[20 0 0;95 0 0])];
    case 5 % Sudden cattle crossing
        name="Sudden Cattle Crossing"; duration=23;
        x=(0:3:120).'; route=[x zeros(size(x)) zeros(size(x))]; speeds=10*ones(size(x));
        s.DesiredSpeed=10; s.SpeedLimit=12; s.Friction=0.58;
        s.GoalRadius=single(8);
        eventMask=bitor(uint16(1),uint16(2+64));
        p=[makePlan(501,9,[1.8 0.7 1.4],[5.5 8.5],[62 -8 0;62 8 0]); ...
           makePlan(502,9,[1.7 0.7 1.35],[6.2 9.5],[67 8 0;67 -8 0]); ...
           makePlan(503,9,[1.6 0.65 1.3],[7 11],[72 -7 0;74 7 0]); ...
           makePlan(504,6,[1.8 0.6 1.2],[0 20],[30 1.5 0;100 1.5 0])];
        if time>=5.5 && time<=11; eventMask=bitor(eventMask,uint16(16)); end
    case 6 % Temporary confidence demo (not part of the five-scenario benchmark)
        name="Temporary 3D Confidence Demo"; duration=12;
        x=(0:0.25:12).'; y=0.10*sin(pi*x/12); route=[x y zeros(size(x))];
        speeds=single(2.0*ones(size(x)));
        s.DesiredSpeed=single(2.0); s.SpeedLimit=single(3.0);
        s.GoalRadius=single(3.00); s.Friction=single(0.82);
        s.InitialYaw=atan2(y(2)-y(1),x(2)-x(1));
        % This confidence route deliberately contains no traffic actors. It
        % proves the complete stack can initialize, plan, control, move and
        % stop at a goal before obstacle scenarios are tuned independently.
        p=repmat(emptyPlan(),0,1);
        eventMask=bitor(uint16(1),uint16(2));
    otherwise
        route=zeros(0,3); speeds=zeros(0,1); s.InitialPosition=[0;0;0];
end
end

function p=emptyPlan()
p=struct("ActorID",uint32(0),"ClassID",uint8(0), ...
    "Dimensions",zeros(1,3,"single"),"Times",zeros(1,2), ...
    "Waypoints",zeros(2,3));
end
function p=makePlan(id,classID,dimensions,times,waypoints)
p=struct("ActorID",uint32(id),"ClassID",uint8(classID), ...
    "Dimensions",single(dimensions),"Times",double(times(:).'), ...
    "Waypoints",double(waypoints));
end
function [active,position,velocity,yaw]=samplePlan(plan,time)
active=time>=plan.Times(1) && time<=plan.Times(end);
position=zeros(1,3); velocity=zeros(1,3); yaw=0;
if ~active; return; end
if numel(plan.Times)==1 || plan.Times(end)<=plan.Times(1)
    position=plan.Waypoints(1,:); return
end
fraction=min(max((time-plan.Times(1))/(plan.Times(end)-plan.Times(1)),0),1);
position=plan.Waypoints(1,:)+fraction*(plan.Waypoints(end,:)-plan.Waypoints(1,:));
velocity=(plan.Waypoints(end,:)-plan.Waypoints(1,:))/(plan.Times(end)-plan.Times(1));
if hypot(velocity(1),velocity(2))>1e-6; yaw=atan2(velocity(2),velocity(1)); end
end
