function metrics=generateRoadsenseLocalPlannerResults()
%GENERATEROADSENSELOCALPLANNERRESULTS Save repeatable plots and metrics.
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
resultDir=fullfile(root,"Results","LocalPlanner");
if ~isfolder(resultDir); mkdir(resultDir); end
scenarios=["cruise";"static_avoid";"crossing_pedestrian";"emergency";"blocked"];
latency=zeros(numel(scenarios),1); candidates=zeros(numel(scenarios),1);
feasible=zeros(numel(scenarios),1); fallback=false(numel(scenarios),1);
valid=false(numel(scenarios),1); code=zeros(numel(scenarios),1);
for index=1:numel(scenarios)
    [inputs{1:6}]=createSyntheticRoadsensePlannerInputs(scenarios(index));
    planner=RoadsenseLocalPlannerSystem;
    for repetition=1:3
        [out{1:26}]=planner(inputs{:});
    end
    latency(index)=1000*double(out{23}); candidates(index)=double(out{13});
    feasible(index)=double(out{14}); fallback(index)=out{15};
    valid(index)=out{16}; code(index)=double(out{19});
end
metrics=table(scenarios,latency,candidates,feasible,fallback,valid,code, ...
    'VariableNames',{'Scenario','LatencyMilliseconds','CandidateCount', ...
    'FeasibleCount','EmergencyFallback','Valid','PlannerCode'});
writetable(metrics,fullfile(resultDir,"local_planner_metrics.csv"));
runRoadsenseLocalPlannerDemo("static_avoid");
exportgraphics(gcf,fullfile(resultDir,"static_avoidance_demo.png"),"Resolution",160);
close(gcf);
runRoadsenseLocalPlannerDemo("crossing_pedestrian");
exportgraphics(gcf,fullfile(resultDir,"pedestrian_yield_demo.png"),"Resolution",160);
close(gcf);
disp(metrics);
end
