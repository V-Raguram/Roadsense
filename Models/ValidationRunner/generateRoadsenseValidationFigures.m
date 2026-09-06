function paths=generateRoadsenseValidationFigures(results,summary,outputDirectory)
%GENERATEROADSENSEVALIDATIONFIGURES Create compact judge-ready evidence plots.
if ~isfolder(outputDirectory); mkdir(outputDirectory); end
paths=strings(0,1);
for index=1:numel(results)
    result=results{index};
    if ~result.Metrics.SimulationSucceeded || isempty(fieldnames(result.Timeline)); continue; end
    timeline=result.Timeline;
    figureHandle=figure("Visible","off","Color","white", ...
        "Position",[100 100 1400 820]); cleanup=onCleanup(@() close(figureHandle));
    tiledlayout(2,2,"TileSpacing","compact","Padding","compact");
    nexttile; plot(result.Route(:,1),result.Route(:,2),"k--","LineWidth",1.5); hold on;
    plot(timeline.Position(:,1),timeline.Position(:,2),"b-","LineWidth",2);
    axis equal; grid on; xlabel("World x (m)"); ylabel("World y (m)");
    title("Route completion"); legend("Reference route","Ego trajectory","Location","best");
    nexttile; plot(timeline.Time,timeline.MinimumClearance,"LineWidth",1.8); hold on;
    yline(0.10,"r--","Required clearance"); grid on; xlabel("Time (s)");
    ylabel("Minimum clearance (m)"); title("Truth-isolated safety clearance");
    nexttile; yyaxis left; plot(timeline.EgoTime,timeline.Speed,"LineWidth",1.7);
    ylabel("Speed (m/s)"); yyaxis right;
    plot(timeline.ControlTime,timeline.Acceleration,"LineWidth",1.3);
    ylabel("Acceleration command (m/s^2)"); grid on; xlabel("Time (s)");
    title("Vehicle response"); legend("Speed","Acceleration","Location","best");
    nexttile; plot(timeline.Time,100*timeline.PipelineReadyRatio,"LineWidth",1.7); hold on;
    plot(timeline.Time,timeline.MaximumCrossTrackError,"LineWidth",1.4);
    grid on; xlabel("Time (s)"); title("Closed-loop quality");
    legend("Pipeline ready (%)","Maximum cross-track error (m)","Location","best");
    header=sgtitle(sprintf("Roadsense | Scenario %d: %s | Pass: %s", ...
        result.ScenarioID,result.ScenarioName,string(result.Metrics.Pass)),"FontWeight","bold");
    header.Color="black";
    applyLightTheme(figureHandle);
    path=fullfile(outputDirectory,sprintf("scenario_%d_evidence.png",result.ScenarioID));
    exportgraphics(figureHandle,path,"Resolution",180); paths(end+1,1)=string(path); %#ok<AGROW>
    clear cleanup;
end

figureHandle=figure("Visible","off","Color","white","Position",[100 100 1500 760]);
cleanup=onCleanup(@() close(figureHandle)); tiledlayout(2,3,"TileSpacing","compact");
labels="S"+string(summary.ScenarioID);
nexttile; bar(categorical(labels),double(summary.MinimumClearanceM));
yline(0.10,"r--"); title("Minimum clearance (m)"); grid on;
nexttile; bar(categorical(labels),double(summary.MaximumReplanLatencyMs));
yline(100,"r--"); title("Maximum replan latency (ms)"); grid on;
nexttile; bar(categorical(labels),double(summary.PathSmoothnessInvM));
yline(0.18,"r--"); title("RMS path curvature (1/m)"); grid on;
nexttile; bar(categorical(labels),double(summary.MaximumJerkMps3));
yline(10,"r--"); title("Maximum jerk (m/s^3)"); grid on;
nexttile; bar(categorical(labels),double(summary.PipelineReadyPercent));
yline(70,"r--"); ylim([0 105]); title("Pipeline readiness (%)"); grid on;
nexttile; values=[summary.Completed summary.CollisionFree summary.GoalReached summary.Pass];
bar(categorical(labels),double(values)); ylim([0 1.15]);
title("Acceptance scorecard"); legend("Completed","Collision-free","Goal","Pass", ...
    "Location","southoutside","Orientation","horizontal"); grid on;
header=sgtitle("Roadsense | Five-scenario closed-loop validation","FontWeight","bold");
header.Color="black";
applyLightTheme(figureHandle);
path=fullfile(outputDirectory,"validation_scorecard.png");
exportgraphics(figureHandle,path,"Resolution",180); paths(end+1,1)=string(path);
clear cleanup;
end

function applyLightTheme(figureHandle)
axesHandles=findall(figureHandle,"Type","axes");
for index=1:numel(axesHandles)
    axisHandle=axesHandles(index);
    set(axisHandle,"Color","white","XColor","black","GridColor",[0.72 0.72 0.72]);
    for axisIndex=1:numel(axisHandle.YAxis); axisHandle.YAxis(axisIndex).Color="black"; end
    axisHandle.Title.Color="black";
    axisHandle.XLabel.Color="black"; axisHandle.YLabel.Color="black";
end
textHandles=findall(figureHandle,"Type","text");
for index=1:numel(textHandles); textHandles(index).Color="black"; end
legendHandles=findall(figureHandle,"Type","legend");
for index=1:numel(legendHandles)
    set(legendHandles(index),"Color","white","TextColor","black", ...
        "EdgeColor",[0.35 0.35 0.35]);
end
end
