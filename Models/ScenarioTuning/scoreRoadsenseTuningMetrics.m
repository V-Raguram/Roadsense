function score=scoreRoadsenseTuningMetrics(summary,profileName)
%SCOREROADSENSETUNINGMETRICS Rank safety first without changing pass limits.
arguments
    summary table
    profileName (1,1) string="candidate"
end
n=max(height(summary),1);
successful=sum(summary.SimulationSucceeded); collisionFree=sum(summary.CollisionFree);
completed=sum(summary.Completed); goals=sum(summary.GoalReached); passed=sum(summary.Pass);
latency=max(double(summary.MaximumReplanLatencyMs),[],'omitnan');
smoothness=max(double(summary.PathSmoothnessInvM),[],'omitnan');
jerk=max(double(summary.MaximumJerkMps3),[],'omitnan');
crossTrack=max(double(summary.MaximumCrossTrackErrorM),[],'omitnan');
readiness=mean(double(summary.PipelineReadyPercent),'omitnan');
if isempty(latency)||isnan(latency); latency=inf; end
if isempty(smoothness)||isnan(smoothness); smoothness=inf; end
if isempty(jerk)||isnan(jerk); jerk=inf; end
if isempty(crossTrack)||isnan(crossTrack); crossTrack=inf; end
if isnan(readiness); readiness=0; end
objective=1e6*(n-successful)+2e5*(n-collisionFree)+1e5*(n-completed)+ ...
    1e5*(n-goals)+5e4*(n-passed)+100*max(latency-100,0)+ ...
    1e4*max(smoothness-0.18,0)+1e3*max(jerk-10,0)+ ...
    1e4*max(crossTrack-2,0)+100*max(70-readiness,0)+ ...
    sum(double(summary.SafetyInterventions));
score=table(profileName,uint16(n),uint16(successful),uint16(collisionFree), ...
    uint16(completed),uint16(goals),uint16(passed),readiness,latency,smoothness, ...
    jerk,crossTrack,objective,'VariableNames',["Profile","Scenarios", ...
    "Successful","CollisionFree","Completed","GoalReached","Passed", ...
    "MeanReadyPercent","WorstLatencyMs","WorstSmoothnessInvM", ...
    "WorstJerkMps3","WorstCrossTrackM","Objective"]);
end
