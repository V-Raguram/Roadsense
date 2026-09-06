function signals=createSyntheticRoadsenseValidationSignals()
%CREATESYNTHETICROADSENSEVALIDATIONSIGNALS Deterministic metric test fixture.
time=(0:0.1:0.4).'; count=numel(time);
signals=struct;
evaluation=struct;
evaluation.ElapsedTime=timeseries(single(time),time);
evaluation.MinimumClearance=timeseries(single([5;3;1.2;0.8;0.65]),time);
evaluation.Collision=timeseries(false(count,1),time);
evaluation.GoalReached=timeseries(logical([0;0;0;0;1]),time);
evaluation.ScenarioFinished=timeseries(logical([0;0;0;0;1]),time);
evaluation.ReplanCount=timeseries(uint32([0;1;1;2;2]),time);
evaluation.MaximumReplanLatency=timeseries(single([0;0.03;0.03;0.045;0.045]),time);
evaluation.PathSmoothness=timeseries(single([0;0.03;0.04;0.05;0.05]),time);
evaluation.MaximumJerk=timeseries(single([0;1;2;2.5;2.5]),time);
evaluation.MaximumCrossTrackError=timeseries(single([0;0.1;0.2;0.25;0.25]),time);
evaluation.SafetyInterventionCount=timeseries(uint32([0;0;1;1;1]),time);
evaluation.PipelineReadyRatio=timeseries(single([0;0.5;0.67;0.75;0.8]),time);
evaluation.Pass=timeseries(logical([0;0;0;0;1]),time);
evaluation.Valid=timeseries(true(count,1),time);
signals.EvaluationStatus=evaluation;

sensor=struct;
sensor.CameraDropout=timeseries(logical([0;1;0;0;0]),time);
sensor.LidarDropout=timeseries(logical([0;0;0;1;0]),time);
sensor.RadarDropout=timeseries(logical([0;0;1;0;0]),time);
signals.SensorStatus=sensor;

ego=struct; ego.Position=timeseries([time zeros(count,2)],time);
ego.Velocity=timeseries([2*ones(count,1) zeros(count,2)],time);
signals.EgoState=ego;
control=struct; control.AccelerationCommand=timeseries(single([0;1;1;0.5;0]),time);
control.SteeringAngle=timeseries(single([0;0.01;0.02;0.01;0]),time);
signals.SafeVehicleControl=control;
safety=struct; safety.OverrideActive=timeseries(logical([0;0;1;0;0]),time);
signals.SafetyStatus=safety;
integration=struct; integration.PipelineReady=timeseries(logical([0;0;1;1;1]),time);
healthFields=["PerceptionValid","FusionValid","PredictionValid","MapValid", ...
    "BehaviourValid","PlanValid","TrackingValid","SafetyValid","VehicleValid"];
for index=1:numel(healthFields)
    integration.(healthFields(index))=timeseries(true(count,1),time);
end
signals.IntegrationStatus=integration;
planTime=time;
signals.LocalPlan=struct("PlanID",timeseries(uint32((1:count).'),planTime), ...
    "Valid",timeseries(true(count,1),planTime), ...
    "EmergencyFallback",timeseries(false(count,1),planTime));
signals.PlannerStatus=struct("PlanID",timeseries(uint32((1:count).'),time), ...
    "Code",timeseries(uint8(ones(count,1)),time), ...
    "CandidateCount",timeseries(uint16(21*ones(count,1)),time), ...
    "FeasibleCount",timeseries(uint16(7*ones(count,1)),time), ...
    "ExecutionTime",timeseries(single(0.02*ones(count,1)),time), ...
    "EmergencyRequested",timeseries(false(count,1),time));
signals.TrackingStatus=struct("PlanID",timeseries(uint32((1:count).'),time), ...
    "Valid",timeseries(true(count,1),time), ...
    "PlanExpired",timeseries(false(count,1),time), ...
    "ReferenceAge",timeseries(zeros(count,1,"single"),time));
signals.SafeVehicleControl.Valid=timeseries(true(count,1),time);
signals.SafeVehicleControl.EmergencyStop=timeseries(false(count,1),time);
signals.SafetyStatus.ReasonMask=timeseries(zeros(count,1,"uint16"),time);
signals.SafetyStatus.State=timeseries(uint8(ones(count,1)),time);
signals.SafetyStatus.MinimumTTC=timeseries(single(inf(count,1)),time);
signals.SafetyStatus.NearestDistance=timeseries(single(inf(count,1)),time);
signals.SafetyStatus.MaximumInputAge=timeseries(zeros(count,1,"single"),time);
signals.SafetyStatus.FrictionUtilization=timeseries(zeros(count,1,"single"),time);
signals.DynamicsStatus=struct( ...
    "LongitudinalSpeed",timeseries(single(2*ones(count,1)),time), ...
    "FrictionUtilization",timeseries(zeros(count,1,"single"),time), ...
    "TireForceSaturated",timeseries(false(count,1),time));
signals.BehaviourCommand=struct( ...
    "Mode",timeseries(uint8(ones(count,1)),time), ...
    "TargetSpeed",timeseries(single(2*ones(count,1)),time), ...
    "ForwardRisk",timeseries(zeros(count,1,"single"),time), ...
    "UnknownFraction",timeseries(zeros(count,1,"single"),time), ...
    "ReasonMask",timeseries(zeros(count,1,"uint16"),time), ...
    "EmergencyRequested",timeseries(false(count,1),time));
end
