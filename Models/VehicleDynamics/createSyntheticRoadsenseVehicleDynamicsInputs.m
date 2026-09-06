function [control,initialState,road]=createSyntheticRoadsenseVehicleDynamicsInputs(scenario)
%CREATESYNTHETICROADSENSEVEHICLEDYNAMICSINPUTS Deterministic plant cases.
arguments
    scenario (1,1) string {mustBeMember(scenario,["straight","turn","brake", ...
        "low_friction","uphill","invalid"])}="straight"
end
initialState=struct("Timestamp",0,"Position",[0;0;0],"Velocity",[5;0;0], ...
    "Acceleration",[0;0;0],"Yaw",0,"Pitch",0,"Roll",0,"YawRate",0, ...
    "SteeringAngle",0,"Valid",true);
control=struct("Timestamp",0,"PlanID",uint32(1),"SteeringAngle",single(0), ...
    "SteeringRate",single(0),"AccelerationCommand",single(1.5), ...
    "Throttle",single(0.5),"Brake",single(0),"TargetSpeed",single(12), ...
    "EmergencyStop",false,"Valid",true);
road=struct("Timestamp",0,"FrictionCoefficient",single(0.85), ...
    "Grade",single(0),"Bank",single(0),"RollingResistance",single(0.012),"Valid",true);
switch scenario
    case "turn"
        initialState.Velocity=[10;0;0]; control.SteeringAngle=single(0.12);
        control.AccelerationCommand=single(0);
    case "brake"
        initialState.Velocity=[15;0;0]; control.AccelerationCommand=single(-6);
        control.Brake=single(1); control.Throttle=single(0); control.EmergencyStop=true;
    case "low_friction"
        initialState.Velocity=[15;0;0]; control.AccelerationCommand=single(-6);
        control.Brake=single(1); control.Throttle=single(0); control.EmergencyStop=true;
        road.FrictionCoefficient=single(0.20);
    case "uphill"
        road.Grade=single(deg2rad(8));
    case "invalid"
        control.Valid=false; control.AccelerationCommand=single(NaN);
end
end
