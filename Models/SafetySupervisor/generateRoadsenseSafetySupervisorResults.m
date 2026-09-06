function metrics=generateRoadsenseSafetySupervisorResults()
%GENERATEROADSENSESAFETYSUPERVISORRESULTS Save state timeline and dwell metrics.
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
resultDir=fullfile(root,"Results","SafetySupervisor");
if ~isfolder(resultDir); mkdir(resultDir); end
result=runRoadsenseSafetySupervisorDemo(true);
exportgraphics(gcf,fullfile(resultDir,"safety_supervisor_timeline.png"),"Resolution",160);
close(gcf); dt=0.02;
stateNames=["Normal";"Degraded";"EmergencyBrake";"MinimalRiskStop";"StandstillHold"];
stateCodes=(1:5).'; dwellSeconds=zeros(5,1);
for index=1:5; dwellSeconds(index)=nnz(result.State==stateCodes(index))*dt; end
metrics=table(stateNames,stateCodes,dwellSeconds, ...
    'VariableNames',{'State','Code','DwellSeconds'});
writetable(metrics,fullfile(resultDir,"safety_state_dwell.csv"));
steadyExecution=result.ExecutionTime(11:end);
summary=table(100*mean(result.Override),100*mean(result.SafeToDrive), ...
    1000*max(steadyExecution),1000*prctile(steadyExecution,95), ...
    nnz(diff(double(result.State))~=0), ...
    'VariableNames',{'OverridePercent','SafeToDrivePercent', ...
    'MaximumSteadyExecutionMilliseconds','P95ExecutionMilliseconds','StateTransitions'});
writetable(summary,fullfile(resultDir,"safety_supervisor_metrics.csv"));
disp(metrics); disp(summary);
end
