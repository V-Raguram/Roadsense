function result=runRoadsenseLocalPlannerDemo(scenario)
%RUNROADSENSELOCALPLANNERDEMO Run and plot one deterministic planning case.
arguments
    scenario (1,1) string="static_avoid"
end
[ego,map,tracks,predictions,behaviour,reference]= ...
    createSyntheticRoadsensePlannerInputs(scenario);
planner=RoadsenseLocalPlannerSystem;
[warmup{1:26}]=planner(ego,map,tracks,predictions,behaviour,reference); %#ok<NASGU>
[output{1:26}]=planner(ego,map,tracks,predictions,behaviour,reference);
result=struct("Timestamp",output{1},"PlanID",output{2},"Count",output{3}, ...
    "TimeFromStart",output{4},"Positions",output{5},"Yaws",output{6}, ...
    "Speeds",output{7},"Accelerations",output{8},"Curvatures",output{9}, ...
    "SteeringAngles",output{10},"TotalCost",output{11}, ...
    "MinimumClearance",output{12},"CandidateCount",output{13}, ...
    "FeasibleCount",output{14},"EmergencyFallback",output{15}, ...
    "Valid",output{16},"PlannerCode",output{19},"ExecutionTime",output{23});

n=double(result.Count); local=(double(result.Positions(1:n,:))-ego.Position(1:2).');
rotation=[cos(ego.Yaw) sin(ego.Yaw);-sin(ego.Yaw) cos(ego.Yaw)]; local=local*rotation.';
figure("Name","Roadsense Local Planner - "+scenario,"Color","w");
tiledlayout(2,1);
ax1=nexttile; imagesc(double(map.XLimits),double(map.YLimits),double(map.CombinedCost));
axis xy equal tight; hold on; colorbar; clim([0 1]); colormap(ax1,"parula");
set(ax1,"XColor","k","YColor","k","Color","w");
plot(reference.Positions(1:reference.Count,1),reference.Positions(1:reference.Count,2), ...
    "k--","LineWidth",1.2,"DisplayName","route");
if predictions.Valid
    for objectIndex=1:double(predictions.Count)
        for mode=1:double(predictions.NumModes(objectIndex))
            predicted=squeeze(predictions.Positions(objectIndex,:,mode,:));
            plot(predicted(:,1),predicted(:,2),":","Color",[1 0.2 0.8], ...
                "LineWidth",1.3,"HandleVisibility","off");
        end
    end
end
if tracks.Valid && tracks.Count>0
    scatter(tracks.Positions(1:tracks.Count,1),tracks.Positions(1:tracks.Count,2), ...
        45,[1 0.2 0.8],"filled","DisplayName","tracked agent");
end
plot(local(:,1),local(:,2),"r-","LineWidth",2.2,"DisplayName","selected plan");
xlabel("forward x (m)"); ylabel("left y (m)");
title("Cost map, route and selected plan","Color","k");
ax2=nexttile; set(ax2,"XColor","k","YColor","k","Color","w"); hold on;
plot(result.TimeFromStart(1:n),result.Speeds(1:n),"LineWidth",1.8);
plot(result.TimeFromStart(1:n),result.Accelerations(1:n),"LineWidth",1.5);
grid on; xlabel("time (s)"); lgd=legend("speed (m/s)","acceleration (m/s^2)");
set(lgd,"TextColor","k","Color","w");
title(sprintf("code=%d, feasible=%d/%d, latency=%.1f ms",result.PlannerCode, ...
    result.FeasibleCount,result.CandidateCount,1000*result.ExecutionTime),"Color","k");
end
