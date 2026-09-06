function result=extractRoadsenseValidationSignals(signals,scenarioID,wallTime,stopTime)
%EXTRACTROADSENSEVALIDATIONSIGNALS Summarize logged or synthetic bus signals.
arguments
    signals (1,1) struct
    scenarioID (1,1) double
    wallTime (1,1) double = NaN
    stopTime (1,1) double = NaN
end
evaluation=signals.EvaluationStatus; sensor=signals.SensorStatus;
names=RoadsenseValidationConfiguration().ScenarioNames; scenarioName=names(scenarioID);
integration=signals.IntegrationStatus;
healthFields=["PerceptionValid","FusionValid","PredictionValid","MapValid", ...
    "BehaviourValid","PlanValid","TrackingValid","SafetyValid","VehicleValid"];
healthValues=false(size(healthFields));
for index=1:numel(healthFields)
    healthValues(index)=logical(last(integration.(healthFields(index))));
end
readinessBlockers=strjoin(healthFields(~healthValues),", ");
finalPipelineReady=logical(last(integration.PipelineReady));
dropoutMask=uint8(logical(series(sensor.CameraDropout))) + ...
    bitshift(uint8(logical(series(sensor.LidarDropout))),1) + ...
    bitshift(uint8(logical(series(sensor.RadarDropout))),2);
dropoutObserved=uint8(0);
for value=dropoutMask(:).'; dropoutObserved=bitor(dropoutObserved,value); end

completed=logical(last(evaluation.ScenarioFinished));
collision=logical(last(evaluation.Collision)); goal=logical(last(evaluation.GoalReached));
passed=logical(last(evaluation.Pass)); valid=logical(last(evaluation.Valid));
metrics=table(uint16(scenarioID),scenarioName,stopTime,true,completed,~collision,goal, ...
    single(last(evaluation.MinimumClearance)),uint32(last(evaluation.ReplanCount)), ...
    single(1000*last(evaluation.MaximumReplanLatency)),single(last(evaluation.PathSmoothness)), ...
    single(last(evaluation.MaximumJerk)),single(last(evaluation.MaximumCrossTrackError)), ...
    uint32(last(evaluation.SafetyInterventionCount)), ...
    single(100*last(evaluation.PipelineReadyRatio)),dropoutObserved,passed,valid, ...
    wallTime,finalPipelineReady,readinessBlockers,"",'VariableNames',metricVariableNames());

timeline=struct;
timeline.Time=double(series(evaluation.ElapsedTime));
timeline.MinimumClearance=single(series(evaluation.MinimumClearance));
timeline.Collision=logical(series(evaluation.Collision));
timeline.GoalReached=logical(series(evaluation.GoalReached));
timeline.ReplanCount=uint32(series(evaluation.ReplanCount));
timeline.MaximumReplanLatency=single(series(evaluation.MaximumReplanLatency));
timeline.PathSmoothness=single(series(evaluation.PathSmoothness));
timeline.MaximumJerk=single(series(evaluation.MaximumJerk));
timeline.MaximumCrossTrackError=single(series(evaluation.MaximumCrossTrackError));
timeline.PipelineReadyRatio=single(series(evaluation.PipelineReadyRatio));
timeline.Pass=logical(series(evaluation.Pass));
timeline.SensorTime=signalTime(sensor.CameraDropout);
timeline.SensorDropoutMask=dropoutMask(:);

ego=signals.EgoState; timeline.EgoTime=signalTime(ego.Position);
timeline.Position=sampleMatrix(ego.Position,3);
velocity=sampleMatrix(ego.Velocity,3);
timeline.Speed=sqrt(sum(velocity.^2,2));
control=signals.SafeVehicleControl;
timeline.ControlTime=signalTime(control.AccelerationCommand);
timeline.Acceleration=single(series(control.AccelerationCommand));
timeline.Steering=single(series(control.SteeringAngle));
safety=signals.SafetyStatus;
timeline.SafetyTime=signalTime(safety.OverrideActive);
timeline.SafetyOverride=logical(series(safety.OverrideActive));
timeline.ReadyTime=signalTime(integration.PipelineReady);
timeline.PipelineReady=logical(series(integration.PipelineReady));
for index=1:numel(healthFields)
    timeline.(healthFields(index))=logical(series(integration.(healthFields(index))));
end
plan=signals.LocalPlan;
timeline.PlanTime=signalTime(plan.PlanID); timeline.PlanID=uint32(series(plan.PlanID));
timeline.PlanValid=logical(series(plan.Valid));
timeline.EmergencyFallback=logical(series(plan.EmergencyFallback));
planner=signals.PlannerStatus;
timeline.PlannerTime=signalTime(planner.PlanID);
timeline.PlannerPlanID=uint32(series(planner.PlanID));
timeline.PlannerCode=uint8(series(planner.Code));
timeline.PlannerCandidateCount=uint16(series(planner.CandidateCount));
timeline.PlannerFeasibleCount=uint16(series(planner.FeasibleCount));
timeline.PlannerExecutionTime=single(series(planner.ExecutionTime));
timeline.PlannerEmergencyRequested=logical(series(planner.EmergencyRequested));
tracking=signals.TrackingStatus;
timeline.TrackingTime=signalTime(tracking.PlanID);
timeline.TrackingPlanID=uint32(series(tracking.PlanID));
timeline.TrackingValid=logical(series(tracking.Valid));
timeline.PlanExpired=logical(series(tracking.PlanExpired));
timeline.ReferenceAge=single(series(tracking.ReferenceAge));
timeline.ControlValid=logical(series(control.Valid));
timeline.EmergencyStop=logical(series(control.EmergencyStop));
timeline.SafetyReasonMask=uint16(series(safety.ReasonMask));
timeline.SafetyState=uint8(series(safety.State));
timeline.SafetyMinimumTTC=single(series(safety.MinimumTTC));
timeline.SafetyNearestDistance=single(series(safety.NearestDistance));
timeline.SafetyMaximumInputAge=single(series(safety.MaximumInputAge));
timeline.SafetyFrictionUtilization=single(series(safety.FrictionUtilization));
behaviour=signals.BehaviourCommand;
timeline.BehaviourTime=signalTime(behaviour.Mode);
timeline.BehaviourMode=uint8(series(behaviour.Mode));
timeline.BehaviourTargetSpeed=single(series(behaviour.TargetSpeed));
timeline.BehaviourForwardRisk=single(series(behaviour.ForwardRisk));
timeline.BehaviourUnknownFraction=single(series(behaviour.UnknownFraction));
timeline.BehaviourReasonMask=uint16(series(behaviour.ReasonMask));
timeline.BehaviourEmergencyRequested=logical(series(behaviour.EmergencyRequested));
dynamics=signals.DynamicsStatus;
timeline.DynamicsTime=signalTime(dynamics.LongitudinalSpeed);
timeline.LongitudinalSpeed=single(series(dynamics.LongitudinalSpeed));
timeline.FrictionUtilization=single(series(dynamics.FrictionUtilization));
timeline.TireForceSaturated=logical(series(dynamics.TireForceSaturated));

[definition,~,~]=RoadsenseScenarioCatalog(scenarioID,0);
route=definition.WorldPositions(definition.ValidMask,1:2);
result=struct("ScenarioID",uint16(scenarioID),"ScenarioName",scenarioName, ...
    "StopTime",stopTime,"WallTime",wallTime,"Metrics",metrics, ...
    "Timeline",timeline,"Route",route,"ErrorMessage","");
end

function values=series(signal)
if isa(signal,"timeseries"); values=squeeze(signal.Data); else; values=squeeze(signal); end
values=values(:);
end

function value=last(signal)
values=series(signal); value=values(end);
end

function time=signalTime(signal)
if isa(signal,"timeseries"); time=double(signal.Time(:));
else; time=(0:numel(series(signal))-1).'; end
end

function samples=sampleMatrix(signal,width)
if isa(signal,"timeseries"); data=signal.Data; else; data=signal; end
data=squeeze(data);
if isvector(data) && width==1; samples=data(:); return; end
if size(data,1)==width
    samples=double(data.');
elseif size(data,2)==width
    samples=double(data);
else
    samples=reshape(double(data),width,[]).';
end
end

function names=metricVariableNames()
names=["ScenarioID","ScenarioName","RequestedStopTime","SimulationSucceeded", ...
    "Completed","CollisionFree","GoalReached","MinimumClearanceM","ReplanCount", ...
    "MaximumReplanLatencyMs","PathSmoothnessInvM","MaximumJerkMps3", ...
    "MaximumCrossTrackErrorM","SafetyInterventions","PipelineReadyPercent", ...
    "DropoutObservedMask","Pass","Valid","WallTimeSeconds", ...
    "FinalPipelineReady","ReadinessBlockers","ErrorMessage"];
end
