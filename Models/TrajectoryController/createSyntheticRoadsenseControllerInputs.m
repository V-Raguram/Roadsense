function [ego,plan,status]=createSyntheticRoadsenseControllerInputs(scenario)
%CREATESYNTHETICROADSENSECONTROLLERINPUTS Build repeatable tracking cases.
arguments
    scenario (1,1) string {mustBeMember(scenario,["aligned","offset", ...
        "curved","overspeed","emergency","expired","invalid"])}="aligned"
end

plannerScenario="cruise";
if scenario=="curved"; plannerScenario="static_avoid"; end
[plannerInputs{1:6}]=createSyntheticRoadsensePlannerInputs(plannerScenario);
planner=RoadsenseLocalPlannerSystem;
[out{1:26}]=planner(plannerInputs{:});
ego=plannerInputs{1};
plan=struct("Timestamp",out{1},"PlanID",out{2},"Count",out{3}, ...
    "TimeFromStart",out{4},"Positions",out{5},"Yaws",out{6}, ...
    "Speeds",out{7},"Accelerations",out{8},"Curvatures",out{9}, ...
    "SteeringAngles",out{10},"TotalCost",out{11}, ...
    "MinimumClearance",out{12},"CandidateCount",out{13}, ...
    "FeasibleCount",out{14},"EmergencyFallback",out{15},"Valid",out{16});
behaviour=RoadsenseTypes.BehaviourMode.Cruise;
if scenario=="curved"; behaviour=RoadsenseTypes.BehaviourMode.AvoidObstacle; end
status=struct("Timestamp",out{17},"PlanID",out{18}, ...
    "Code",RoadsenseTypes.PlannerCode.Success,"Behaviour",behaviour, ...
    "CandidateCount",out{21},"FeasibleCount",out{22}, ...
    "ExecutionTime",out{23},"Deadline",out{24}, ...
    "ReplanRequested",out{25},"EmergencyRequested",false);

switch scenario
    case "offset"
        ego.Position(2)=ego.Position(2)+1.2; ego.Yaw=ego.Yaw+0.08;
        speed=norm(ego.Velocity(1:2));
        ego.Velocity(1:2)=speed*[cos(ego.Yaw);sin(ego.Yaw)];
    case "overspeed"
        ego.Velocity(1:2)=single(12)*[cos(ego.Yaw);sin(ego.Yaw)];
    case "emergency"
        plan.EmergencyFallback=true; plan.Speeds(:)=single(0);
        status.Code=RoadsenseTypes.PlannerCode.EmergencyFallback;
        status.EmergencyRequested=true;
    case "expired"
        ego.Timestamp=plan.Timestamp+double(plan.TimeFromStart(plan.Count))+1;
    case "invalid"
        plan.Valid=false;
end
end
