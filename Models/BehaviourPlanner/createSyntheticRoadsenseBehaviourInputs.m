function [ego,map,tracks,route] = createSyntheticRoadsenseBehaviourInputs(scenario)
%CREATESYNTHETICROADSENSEBEHAVIOURINPUTS Build deterministic decision cases.

arguments
    scenario (1,1) string {mustBeMember(scenario,["cruise","follow","emergency", ...
        "yield","creep","avoid","cautious","stop","invalid"])} = "cruise"
end
timestamp=1.0;
ego=struct("Timestamp",timestamp,"Position",[0;0;0],"Velocity",[5;0;0], ...
    "Acceleration",[0;0;0],"Yaw",0,"Pitch",0,"Roll",0,"YawRate",0, ...
    "SteeringAngle",0,"Valid",true);

rows=200; cols=320; resolution=single(0.25);
xLimits=single([-20;60]); yLimits=single([-25;25]);
map=struct("Timestamp",timestamp,"Resolution",resolution,"XLimits",xLimits, ...
    "YLimits",yLimits,"StaticOccupancy",zeros(rows,cols,"single"), ...
    "DynamicOccupancy",zeros(rows,cols,"single"), ...
    "PredictedRisk",zeros(rows,cols,"single"),"Occupancy",zeros(rows,cols,"single"), ...
    "Drivability",repmat(single(0.90),rows,cols), ...
    "SurfaceCost",zeros(rows,cols,"single"), ...
    "CombinedCost",repmat(single(0.075),rows,cols), ...
    "SemanticLabel",ones(rows,cols,"uint8"),"ObservedMask",true(rows,cols), ...
    "ProcessingTime",single(0.02),"SourceOverflow",false,"Valid",true);

tracks=createSyntheticRoadsenseTrackList(timestamp);
tracks.Count=uint16(0); tracks.ValidMask(:)=false;
route=struct("Timestamp",timestamp,"DesiredSpeed",single(8), ...
    "SpeedLimit",single(10),"DistanceToGoal",single(100),"GoalReached",false, ...
    "AllowObstacleAvoidance",true,"UncontrolledIntersection",false, ...
    "OccludedArea",false,"MergeRequired",false,"Valid",true);

switch scenario
    case "follow"
        tracks=oneTrack(tracks,uint8(1),[10 0 0.8],[-1 0 0],[4.4 1.8 1.6]);
        route.AllowObstacleAvoidance=false;
    case "emergency"
        tracks=oneTrack(tracks,uint8(1),[5 0 0.8],[-6 0 0],[4.4 1.8 1.6]);
        map=paintRectangle(map,2,7,-1.3,1.3,"dynamic",single(0.98));
    case "yield"
        tracks=oneTrack(tracks,uint8(7),[8 1 0.9],[-0.5 -1.0 0],[0.6 0.6 1.75]);
    case "creep"
        route.UncontrolledIntersection=true;
        route.OccludedArea=true;
        map.ObservedMask=paintMask(map.ObservedMask,map,0,25,-4,4,false);
        map.Drivability(~map.ObservedMask)=single(0.35);
        map.CombinedCost(~map.ObservedMask)=single(0.4875);
        ego.Velocity=[0.5;0;0];
    case "avoid"
        tracks=oneTrack(tracks,uint8(10),[8 0 0.5],[0 0 0],[1.2 1.2 1.0]);
        map=paintRectangle(map,6,10,-1.2,1.2,"static",single(0.95));
    case "cautious"
        map.SurfaceCost=paintNumeric(map.SurfaceCost,map,4,14,-1.5,1.5,single(0.70));
        map.CombinedCost=max(map.CombinedCost,map.SurfaceCost);
    case "stop"
        route.GoalReached=true;
        route.DistanceToGoal=single(0.5);
    case "invalid"
        map.Valid=false;
end
end

function tracks=oneTrack(tracks,classID,position,velocity,dimensions)
tracks.Count=uint16(1); tracks.ValidMask(:)=false; tracks.ValidMask(1)=true;
tracks.TrackIDs(1)=uint32(501); tracks.ClassIDs(1)=classID;
tracks.ClassConfidences(1)=single(0.95); tracks.ExistenceProbabilities(1)=single(0.99);
tracks.Positions(1,:)=single(position); tracks.Velocities(1,:)=single(velocity);
tracks.Dimensions(1,:)=single(dimensions); tracks.SensorMasks(1)=uint8(7);
tracks.IsConfirmed(1)=true;
tracks.StateCovariances(:,:,1)=single(diag([0.05 0.05 0.08 0.15 0.15 0.20]));
end

function map=paintRectangle(map,xMinimum,xMaximum,yMinimum,yMaximum,kind,value)
mask=gridMask(map,xMinimum,xMaximum,yMinimum,yMaximum);
if kind=="dynamic"
    map.DynamicOccupancy(mask)=value;
else
    map.StaticOccupancy(mask)=value;
end
map.Occupancy(mask)=value;
map.CombinedCost(mask)=value;
end

function output=paintNumeric(input,map,xMinimum,xMaximum,yMinimum,yMaximum,value)
output=input; output(gridMask(map,xMinimum,xMaximum,yMinimum,yMaximum))=value;
end

function output=paintMask(input,map,xMinimum,xMaximum,yMinimum,yMaximum,value)
output=input; output(gridMask(map,xMinimum,xMaximum,yMinimum,yMaximum))=value;
end

function mask=gridMask(map,xMinimum,xMaximum,yMinimum,yMaximum)
x=double(map.XLimits(1))+(0.5:319.5)*double(map.Resolution);
y=double(map.YLimits(1))+(0.5:199.5)*double(map.Resolution);
[xGrid,yGrid]=meshgrid(x,y);
mask=xGrid>=xMinimum & xGrid<=xMaximum & yGrid>=yMinimum & yGrid<=yMaximum;
end
