function profile=RoadsenseTuningProfile(name)
%ROADSENSETUNINGPROFILE Reproducible, evaluator-independent parameter sets.
arguments
    name (1,1) string {mustBeMember(name,["baseline","balanced","cautious"])}="balanced"
end
baseline=struct( ...
    "RsPlannerSpeedScales",single([0.70;1.00;1.15]), ...
    "RsPlannerCollisionProbability",single(0.05), ...
    "RsPlannerPredictionSigma",single(2.0), ...
    "RsPlannerMaximumLongitudinalInflation",single(3.0), ...
    "RsPlannerMaximumLateralInflation",single(2.5), ...
    "RsPlannerLateralClearanceRatio",single(0.35), ...
    "RsPlannerMaximumLateralClearance",single(0.80), ...
    "RsControllerSpeedKp",single(0.80), ...
    "RsControllerSpeedKi",single(0.15), ...
    "RsControllerAccelerationSlew",single(5.0), ...
    "RsControllerPreviewTime",single(1.0), ...
    "RsBehaviourReactionTime",single(0.60), ...
    "RsBehaviourEmergencyTTC",single(1.20), ...
    "RsBehaviourFollowTimeGap",single(1.80), ...
    "RsBehaviourCautiousSpeed",single(5.0), ...
    "RsBehaviourAvoidSpeed",single(4.0), ...
    "RsSafetyEmergencyTTC",single(1.0), ...
    "RsSafetyEmergencyDistance",single(2.5), ...
    "RsSafetyDegradedSpeed",single(4.0), ...
    "RsSafetyEmergencyHoldTicks",uint16(50), ...
    "RsSafetyRecoveryTicks",uint16(25));
values=baseline;
switch name
    case "balanced"
        values.RsPlannerSpeedScales=single([0.65;0.90;1.00]);
        values.RsPlannerCollisionProbability=single(0.035);
        values.RsPlannerPredictionSigma=single(2.3);
        values.RsPlannerMaximumLongitudinalInflation=single(3.0);
        values.RsPlannerMaximumLateralInflation=single(2.5);
        values.RsPlannerLateralClearanceRatio=single(0.35);
        values.RsPlannerMaximumLateralClearance=single(0.80);
        values.RsControllerSpeedKp=single(0.72);
        values.RsControllerSpeedKi=single(0.12);
        values.RsControllerAccelerationSlew=single(3.5);
        values.RsControllerPreviewTime=single(1.0);
        values.RsBehaviourReactionTime=single(0.75);
        values.RsBehaviourEmergencyTTC=single(1.40);
        values.RsBehaviourFollowTimeGap=single(2.0);
        values.RsBehaviourCautiousSpeed=single(5.5);
        values.RsBehaviourAvoidSpeed=single(3.6);
        values.RsSafetyEmergencyTTC=single(1.15);
        values.RsSafetyEmergencyDistance=single(2.8);
        values.RsSafetyDegradedSpeed=single(3.5);
        values.RsSafetyEmergencyHoldTicks=uint16(30);
        values.RsSafetyRecoveryTicks=uint16(12);
    case "cautious"
        values.RsPlannerSpeedScales=single([0.55;0.75;0.90]);
        values.RsPlannerCollisionProbability=single(0.025);
        values.RsPlannerPredictionSigma=single(2.7);
        values.RsPlannerMaximumLongitudinalInflation=single(4.0);
        values.RsPlannerMaximumLateralInflation=single(3.0);
        values.RsPlannerLateralClearanceRatio=single(0.40);
        values.RsPlannerMaximumLateralClearance=single(1.00);
        values.RsControllerSpeedKp=single(0.65);
        values.RsControllerSpeedKi=single(0.10);
        values.RsControllerAccelerationSlew=single(3.0);
        values.RsControllerPreviewTime=single(1.2);
        values.RsBehaviourReactionTime=single(0.90);
        values.RsBehaviourEmergencyTTC=single(1.60);
        values.RsBehaviourFollowTimeGap=single(2.3);
        values.RsBehaviourCautiousSpeed=single(4.0);
        values.RsBehaviourAvoidSpeed=single(3.0);
        values.RsSafetyEmergencyTTC=single(1.30);
        values.RsSafetyEmergencyDistance=single(3.2);
        values.RsSafetyDegradedSpeed=single(3.0);
        values.RsSafetyEmergencyHoldTicks=uint16(35);
        values.RsSafetyRecoveryTicks=uint16(15);
end
profile=struct("Name",name,"Values",values, ...
    "Description",description(name),"ContractVersion",uint32(16), ...
    "TuningSchemaVersion",uint32(1));
end

function value=description(name)
switch name
    case "baseline"; value="Original component defaults for an auditable control run.";
    case "balanced"; value="Recommended mixed-traffic profile: smoother control and wider uncertainty margins.";
    otherwise; value="Low-speed, high-margin fallback for dense or highly uncertain traffic.";
end
end
