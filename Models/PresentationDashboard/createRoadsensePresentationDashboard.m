function path=createRoadsensePresentationDashboard(source,outputPath)
%CREATEROADSENSEPRESENTATIONDASHBOARD Render judge-facing scenario evidence.
arguments
    source
    outputPath (1,1) string = ""
end
result=resolveRoadsensePresentationResult(source);
if strlength(outputPath)==0
    root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
    outputPath=fullfile(root,"Results","Presentation", ...
        "scenario_"+result.ScenarioID+"_dashboard.png");
end
folder=fileparts(outputPath); if ~isfolder(folder); mkdir(folder); end
t=result.Timeline; row=result.Metrics;
f=figure("Visible","off","Color","white","Position",[80 60 1500 900]);
cleanup=onCleanup(@() close(f)); layout=tiledlayout(f,3,3,"Padding","compact","TileSpacing","compact");
ax=nexttile(layout,[2 2]); plot(ax,result.Route(:,1),result.Route(:,2),"k--","LineWidth",1.4); hold(ax,"on");
plot(ax,t.Position(:,1),t.Position(:,2),"Color",[0.00 0.45 0.74],"LineWidth",2.5);
scatter(ax,t.Position(end,1),t.Position(end,2),65,[0.85 0.15 0.12],"filled"); axis(ax,"equal"); grid(ax,"on");
xlabel(ax,"World x (m)"); ylabel(ax,"World y (m)"); title(ax,result.ScenarioName+" - route and ego trajectory");
legend(ax,"Reference route","Ego trajectory","Final position","Location","best");

ax=nexttile(layout); plot(ax,t.Time,t.MinimumClearance,"LineWidth",1.8); yline(ax,0.10,"r--","Limit");
xlabel(ax,"Time (s)"); ylabel(ax,"Clearance (m)"); title(ax,"Truth-isolated clearance"); grid(ax,"on");
ax=nexttile(layout); yyaxis(ax,"left"); plot(ax,t.EgoTime,t.Speed,"LineWidth",1.8); ylabel(ax,"Speed (m/s)");
yyaxis(ax,"right"); plot(ax,t.ControlTime,t.Acceleration,"LineWidth",1.3); ylabel(ax,"Acceleration (m/s^2)");
xlabel(ax,"Time (s)"); title(ax,"Vehicle response"); grid(ax,"on");
ax=nexttile(layout); stairs(ax,t.ReadyTime,double(t.PipelineReady),"LineWidth",1.7); hold(ax,"on");
stairs(ax,t.SafetyTime,1.05*double(t.SafetyOverride),"LineWidth",1.4); ylim(ax,[-0.1 1.25]);
yticks(ax,[0 1]); yticklabels(ax,["No","Yes"]); xlabel(ax,"Time (s)"); title(ax,"Readiness and safety override"); grid(ax,"on"); legend(ax,"Pipeline ready","Safety override");
ax=nexttile(layout); axis(ax,"off");
status=compose("SCENARIO RESULT\n\nSimulation: %s\nCollision-free: %s\nCompleted: %s\nGoal reached: %s\nPass: %s\n\n" + ...
    "Minimum clearance: %.3f m\nReplan latency: %.2f ms\nSmoothness: %.4f 1/m\nNominal jerk: %.2f m/s^3\nReady: %.1f %%", ...
    yesno(row.SimulationSucceeded),yesno(row.CollisionFree),yesno(row.Completed), ...
    yesno(row.GoalReached),yesno(row.Pass),row.MinimumClearanceM, ...
    row.MaximumReplanLatencyMs,row.PathSmoothnessInvM,row.MaximumJerkMps3,row.PipelineReadyPercent);
text(ax,0.02,0.98,status,"VerticalAlignment","top","FontName","Consolas","FontSize",11, ...
    "Color",statusColor(row));
title(layout,"ROADSENSE  |  ADAPTIVE AUTONOMY ON UNSTRUCTURED INDIAN ROADS", ...
    "FontSize",17,"FontWeight","bold");
exportgraphics(f,outputPath,"Resolution",180); path=outputPath;
end

function value=yesno(flag)
if flag; value="YES"; else; value="NO"; end
end
function color=statusColor(row)
if row.Pass; color=[0.05 0.45 0.16]; elseif row.CollisionFree; color=[0.75 0.42 0.02]; else; color=[0.75 0.08 0.08]; end
end
