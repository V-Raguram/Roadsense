function [ego,map,tracks,predictions,behaviour,reference]= ...
        createSyntheticRoadsensePlannerInputs(scenario)
%CREATESYNTHETICROADSENSEPLANNERINPUTS Deterministic local-planner cases.
arguments
    scenario (1,1) string {mustBeMember(scenario,["cruise","static_avoid", ...
        "crossing_pedestrian","emergency","blocked","invalid"])}="cruise"
end

[ego,map,tracks,~]=createSyntheticRoadsenseBehaviourInputs("cruise");
ego.Position=[100;50;0]; ego.Yaw=0; ego.Velocity=[5;0;0];
tracks.Count=uint16(0); tracks.ValidMask(:)=false;
predictions=emptyPredictions(ego.Timestamp);
behaviour=makeBehaviour(ego.Timestamp,RoadsenseTypes.BehaviourMode.Cruise,single(8));
reference=makeReference(ego.Timestamp);

switch scenario
    case "static_avoid"
        behaviour.Mode=RoadsenseTypes.BehaviourMode.AvoidObstacle;
        behaviour.TargetSpeed=single(4); behaviour.ReplanRequested=true;
        map=paintStatic(map,10,20,-1.2,1.2,single(0.96));
    case "crossing_pedestrian"
        behaviour.Mode=RoadsenseTypes.BehaviourMode.Yield;
        behaviour.TargetSpeed=single(0); behaviour.DesiredClearance=single(2);
        [tracks,predictions]=crossingPedestrian(tracks,predictions);
    case "emergency"
        behaviour.Mode=RoadsenseTypes.BehaviourMode.EmergencyBrake;
        behaviour.TargetSpeed=single(0); behaviour.MinimumAcceleration=single(-6);
        behaviour.EmergencyRequested=true;
    case "blocked"
        behaviour.Mode=RoadsenseTypes.BehaviourMode.AvoidObstacle;
        behaviour.TargetSpeed=single(4);
        map=paintStatic(map,5,16,-8,8,single(0.98));
    case "invalid"
        reference.Valid=false;
end
end

function reference=makeReference(timestamp)
n=161; x=single(linspace(0,80,n).'); y=single(0.30*sin(double(x)/18));
yaw=single(atan2(gradient(double(y)),gradient(double(x))));
positions=zeros(256,2,"single"); positions(1:n,:)=[x y];
yaws=zeros(256,1,"single"); yaws(1:n)=yaw;
speeds=zeros(256,1,"single"); speeds(1:n)=single(9);
mask=false(256,1); mask(1:n)=true;
reference=struct("Timestamp",timestamp,"Count",uint16(n),"Positions",positions, ...
    "Yaws",yaws,"RecommendedSpeeds",speeds,"ValidMask",mask,"Valid",true);
end

function behaviour=makeBehaviour(timestamp,mode,targetSpeed)
behaviour=struct("Timestamp",timestamp,"SequenceID",uint32(1),"Mode",mode, ...
    "TargetSpeed",targetSpeed,"MaximumAcceleration",single(2), ...
    "MinimumAcceleration",single(-3),"DesiredClearance",single(1), ...
    "StopDistance",single(12),"LeadTrackID",uint32(0),"MinimumTTC",single(inf), ...
    "ForwardRisk",single(0),"UnknownFraction",single(0),"ReasonMask",uint16(0), ...
    "ReplanRequested",false,"EmergencyRequested",false,"Valid",true);
end

function predictions=emptyPredictions(timestamp)
predictions=struct("Timestamp",timestamp,"Count",uint16(0), ...
    "TrackIDs",zeros(128,1,"uint32"),"ClassIDs",zeros(128,1,"uint8"), ...
    "NumModes",zeros(128,1,"uint8"),"NumSteps",uint8(16), ...
    "TimeOffsets",single((0:15).')*single(0.2), ...
    "ModeProbabilities",zeros(128,4,"single"), ...
    "Positions",zeros(128,16,4,2,"single"), ...
    "Yaws",zeros(128,16,4,"single"), ...
    "PositionCovariances",zeros(2,2,16,4,128,"single"), ...
    "ValidMask",false(128,1),"ProcessingTime",single(0), ...
    "Overflow",false,"Valid",true);
end

function map=paintStatic(map,xmin,xmax,ymin,ymax,value)
x=double(map.XLimits(1))+(0.5:319.5)*double(map.Resolution);
y=double(map.YLimits(1))+(0.5:199.5)*double(map.Resolution);
[xg,yg]=meshgrid(x,y); mask=xg>=xmin & xg<=xmax & yg>=ymin & yg<=ymax;
map.StaticOccupancy(mask)=value; map.Occupancy(mask)=value;
map.CombinedCost(mask)=value; map.Drivability(mask)=single(0.02);
end

function [tracks,predictions]=crossingPedestrian(tracks,predictions)
tracks.Count=uint16(1); tracks.ValidMask(1)=true; tracks.TrackIDs(1)=uint32(901);
tracks.ClassIDs(1)=uint8(7); tracks.ExistenceProbabilities(1)=single(0.99);
tracks.ClassConfidences(1)=single(0.95); tracks.Positions(1,:)=single([14 5 0.9]);
tracks.Velocities(1,:)=single([0 -2 0]); tracks.Dimensions(1,:)=single([0.6 0.6 1.75]);
tracks.StateCovariances(:,:,1)=single(diag([0.04 0.04 0.1 0.2 0.2 0.2]));
tracks.SensorMasks(1)=uint8(7); tracks.IsConfirmed(1)=true;
predictions.Count=uint16(1); predictions.TrackIDs(1)=uint32(901);
predictions.ClassIDs(1)=uint8(7); predictions.NumModes(1)=uint8(1);
predictions.ModeProbabilities(1,1)=single(1); predictions.ValidMask(1)=true;
for k=1:16
    t=double(predictions.TimeOffsets(k));
    predictions.Positions(1,k,1,1)=single(14);
    predictions.Positions(1,k,1,2)=single(5-2*t);
    predictions.Yaws(1,k,1)=single(-pi/2);
    predictions.PositionCovariances(:,:,k,1,1)=single(diag([0.06+0.02*t 0.06+0.04*t]));
end
end
