function [world,result]=runRoadsense3DReplay(options)
%RUNROADSENSE3DREPLAY Render an actual Roadsense closed-loop run in Unreal.
%   RUNROADSENSE3DREPLAY() runs village-road scenario 1 through the complete
%   autonomy stack, then replays the resulting ego trajectory in the native
%   Simulink 3D Animation viewer. The ego is an Unreal SUV mesh, not a
%   drivingScenario cuboid. Close the 3D viewer or call close(world) when done.
%
%   Example:
%     runRoadsense3DReplay(ScenarioID=5,StopTime=32,PlaybackRate=1.0)

arguments
    options.ScenarioID (1,1) double {mustBeInteger,mustBeInRange(options.ScenarioID,1,5)} = 1
    options.StopTime (1,1) double {mustBePositive} = 32
    options.PlaybackRate (1,1) double {mustBePositive} = 1
    options.RunClosedLoop (1,1) logical = true
    options.ResultFile (1,1) string = ""
end

componentDir=fileparts(mfilename("fullpath"));
root=fileparts(fileparts(componentDir));
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
setupRoadsense;

if options.RunClosedLoop
    simulationOutput=runRoadsenseScenarioClosedLoopHarnessDemo( ...
        options.ScenarioID,options.StopTime);
    result=extractRoadsenseValidationResult(simulationOutput, ...
        options.ScenarioID,NaN,options.StopTime);
elseif strlength(options.ResultFile)>0
    loaded=load(options.ResultFile,"result"); result=loaded.result;
else
    error("Roadsense:3DReplay:NoTrajectory", ...
        "Set RunClosedLoop=true or supply ResultFile containing result.");
end

timeline=normalizeTimeline(result.Timeline);
duration=timeline.Time(end)-timeline.Time(1);
if duration<=0
    error("Roadsense:3DReplay:InvalidTimeline", ...
        "The closed-loop result does not contain a time-varying ego trajectory.");
end

world=sim3d.World(Name="Roadsense 3D Replay",Scene="Blank scene", ...
    SampleTime=0.05,StopTime=duration/options.PlaybackRate, ...
    Setup=@setupWorld,Update=@updateWorld,Release=@releaseWorld);
world.EnablePacing=true; world.PacingRate=1;
world.UserData=struct("Timeline",timeline,"PlaybackRate",options.PlaybackRate, ...
    "ScenarioID",options.ScenarioID,"Root",root,"Ego",[],"Road",[], ...
    "Terrain",[],"View",[]);
run(world);
view(world);

    function setupWorld(activeWorld)
        data=activeWorld.UserData;
        % A broad textured ground plane plus a dark, unmarked road emphasize
        % the intended Indian-road setting. The native scene supplies sky,
        % lighting, shadows, and physically rendered terrain surroundings.
        terrain=sim3d.Actor(ActorName="Roadsense Terrain", ...
            Mobility=sim3d.utils.MobilityTypes.Static,Color=[0.24 0.38 0.16]);
        createShape(terrain,"plane"); terrain.Scale=[180 120 1];
        add(activeWorld,terrain);

        road=sim3d.Actor(ActorName="Unmarked Village Road", ...
            Mobility=sim3d.utils.MobilityTypes.Static,Color=[0.12 0.12 0.12]);
        createShape(road,"plane"); road.Scale=[95 6.5 1];
        road.Translation=[45 0 0.02];
        add(activeWorld,road);

        ego=sim3d.vehicle.ground.PassengerVehicle( ...
            ActorName="Roadsense Autonomous SUV", ...
            VehicleType="SportUtilityVehicle",Color="Blue", ...
            CoordinateSystem="ISO8855");
        ego.X=data.Timeline.Position(1,1); ego.Y=data.Timeline.Position(1,2);
        ego.Yaw=data.Timeline.Yaw(1); ego.EnableLightControls=true;
        add(activeWorld,ego);

        viewPoint=createViewpoint(activeWorld,Name="Roadsense Chase View", ...
            Translation=[-12 -8 5],Rotation=[0 -12 28]);
        setView(activeWorld,viewPoint);
        data.Ego=ego; data.Road=road; data.Terrain=terrain; data.View=viewPoint;
        activeWorld.UserData=data;
    end

    function updateWorld(activeWorld)
        data=activeWorld.UserData;
        replayTime=data.Timeline.Time(1)+activeWorld.SimulationTime*data.PlaybackRate;
        x=interp1(data.Timeline.Time,data.Timeline.Position(:,1),replayTime,"linear","extrap");
        y=interp1(data.Timeline.Time,data.Timeline.Position(:,2),replayTime,"linear","extrap");
        yaw=interp1(data.Timeline.Time,data.Timeline.UnwrappedYaw,replayTime,"linear","extrap");
        data.Ego.X=x; data.Ego.Y=y; data.Ego.Yaw=wrapAngle(yaw);
        % Brake lamps provide a clear visual cue during safety interventions.
        index=find(data.Timeline.Time<=replayTime,1,"last");
        braking=data.Timeline.Acceleration(max(1,index))<-0.4;
        data.Ego.LightControls=[true false braking false false false];
    end

    function releaseWorld(~)
        % Resource ownership remains with the returned world object so users
        % can inspect the final scene before explicitly closing it.
    end
end

function timeline=normalizeTimeline(timeline)
time=double(timeline.EgoTime(:)); position=double(timeline.Position);
count=min(numel(time),size(position,1)); time=time(1:count); position=position(1:count,1:2);
valid=isfinite(time) & all(isfinite(position),2);
time=time(valid); position=position(valid,:);
[time,indices]=unique(time,"stable"); position=position(indices,:);
if numel(time)<2; error("Roadsense:3DReplay:TooFewSamples","Need two ego samples."); end
delta=[diff(position); position(end,:)-position(max(1,end-1),:)];
yaw=atan2(delta(:,2),delta(:,1));
if numel(yaw)>1; yaw(end)=yaw(end-1); end
timeline.Time=time; timeline.Position=position; timeline.Yaw=yaw;
timeline.UnwrappedYaw=unwrap(yaw);
acceleration=double(timeline.Acceleration(:));
if isempty(acceleration); acceleration=zeros(numel(time),1); end
timeline.Acceleration=interp1(linspace(time(1),time(end),numel(acceleration)).', ...
    acceleration,time,"previous","extrap");
end

function angle=wrapAngle(angle)
angle=mod(angle+pi,2*pi)-pi;
end
