function path=renderRoadsenseDemonstrationVideo(source,outputPath,options)
%RENDERROADSENSEDEMONSTRATIONVIDEO Export an annotated MP4 replay.
arguments
    source
    outputPath (1,1) string
    options.FrameRate (1,1) double {mustBePositive}=20
    options.PlaybackSpeed (1,1) double {mustBePositive}=2
    options.MaximumFrames (1,1) double {mustBeInteger,mustBePositive}=1200
end
result=resolveRoadsensePresentationResult(source); t=result.Timeline;
folder=fileparts(outputPath); if ~isfolder(folder); mkdir(folder); end
duration=max(double(t.Time(end)),eps); frameCount=min(options.MaximumFrames, ...
    max(2,ceil(duration/options.PlaybackSpeed*options.FrameRate)+1));
frameTimes=linspace(0,duration,frameCount);
writer=VideoWriter(outputPath,"MPEG-4"); writer.FrameRate=options.FrameRate; writer.Quality=92;
open(writer); writerCleanup=onCleanup(@() close(writer));
f=figure("Visible","off","Color","white","Position",[50 50 1280 720]);
figureCleanup=onCleanup(@() close(f)); layout=tiledlayout(f,2,3,"Padding","compact","TileSpacing","compact");
mapAxis=nexttile(layout,[2 2]); metricAxis=nexttile(layout); statusAxis=nexttile(layout);
for frame=1:frameCount
    current=frameTimes(frame); egoIndex=nearest(t.EgoTime,current); controlIndex=nearest(t.ControlTime,current);
    readyIndex=nearest(t.ReadyTime,current); safetyIndex=nearest(t.SafetyTime,current);
    cla(mapAxis); plot(mapAxis,result.Route(:,1),result.Route(:,2),"k--","LineWidth",1.2); hold(mapAxis,"on");
    plot(mapAxis,t.Position(1:egoIndex,1),t.Position(1:egoIndex,2),"b-","LineWidth",2.3);
    scatter(mapAxis,t.Position(egoIndex,1),t.Position(egoIndex,2),90,[0.00 0.45 0.74],"filled");
    [~,actors]=RoadsenseScenarioCatalog(double(result.ScenarioID),current);
    valid=find(actors.ValidMask);
    if ~isempty(valid)
        scatter(mapAxis,actors.Positions(valid,1),actors.Positions(valid,2),70, ...
            double(actors.ClassIDs(valid)),"filled","MarkerEdgeColor","k");
    end
    axis(mapAxis,"equal"); grid(mapAxis,"on"); xlabel(mapAxis,"World x (m)"); ylabel(mapAxis,"World y (m)");
    title(mapAxis,result.ScenarioName+compose("  |  t = %.1f s",current));
    xlim(mapAxis,paddedLimits([result.Route(:,1);t.Position(:,1)],5));
    ylim(mapAxis,paddedLimits([result.Route(:,2);t.Position(:,2)],4));

    cla(metricAxis); yyaxis(metricAxis,"left"); plot(metricAxis,t.EgoTime,t.Speed,"b","LineWidth",1.5); hold(metricAxis,"on");
    xline(metricAxis,current,"k:"); ylabel(metricAxis,"Speed (m/s)");
    yyaxis(metricAxis,"right"); plot(metricAxis,t.ControlTime,t.Acceleration,"Color",[0.85 0.33 0.10],"LineWidth",1.2);
    ylabel(metricAxis,"Acceleration (m/s^2)"); xlim(metricAxis,[0 duration]); grid(metricAxis,"on"); title(metricAxis,"Closed-loop response");

    cla(statusAxis); axis(statusAxis,"off"); ready=t.PipelineReady(readyIndex); override=t.SafetyOverride(safetyIndex);
    status=compose("ROADSENSE LIVE STATUS\n\nSpeed       %6.2f m/s\nAcceleration %5.2f m/s^2\n" + ...
        "Pipeline     %s\nSafety override %s\nCollision      %s\n\nReplans       %u\nMinimum clearance %.2f m", ...
        t.Speed(egoIndex),t.Acceleration(controlIndex),stateText(ready),stateText(override), ...
        stateText(any(t.Collision(t.Time<=current))),t.ReplanCount(nearest(t.Time,current)), ...
        t.MinimumClearance(nearest(t.Time,current)));
    text(statusAxis,0.03,0.96,status,"VerticalAlignment","top","FontName","Consolas", ...
        "FontSize",12,"Color",[0.08 0.18 0.28]);
    title(layout,"ROADSENSE - SENSOR FUSION, PREDICTION, PLANNING AND SAFETY", ...
        "FontSize",15,"FontWeight","bold"); drawnow; writeVideo(writer,getframe(f));
end
clear figureCleanup writerCleanup; path=outputPath;
end

function index=nearest(time,value)
[~,index]=min(abs(double(time(:))-value));
end
function limits=paddedLimits(values,padding)
limits=[min(values)-padding max(values)+padding]; if diff(limits)<2*padding; limits=limits+[-padding padding]; end
end
function textValue=stateText(value)
if value; textValue="YES"; else; textValue="NO"; end
end
