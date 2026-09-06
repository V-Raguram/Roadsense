function [result,figureHandle]=runRoadsenseConfidenceDemo(options)
%RUNROADSENSECONFIDENCEDEMO Run the full stack and animate a 3D car in real time.
%   RUNROADSENSECONFIDENCEDEMO requires MATLAB and Simulink only. It runs a
%   short, deterministic confidence scenario through the complete Roadsense
%   closed-loop harness, verifies goal arrival and collision-free operation,
%   and then replays the measured ego motion at real-time speed.
%
%   This temporary viewer is intentionally independent of RoadRunner. The
%   moving pose, speed, safety state, clearance, and goal state all come from
%   the actual Simulink result; the graphics do not drive the vehicle.
%
%   Run from the repository root:
%       setupRoadsense; runRoadsenseConfidenceDemo

arguments
    options.PlaybackRate (1,1) double {mustBePositive}=1
    options.StopTime (1,1) double {mustBePositive}=16
    options.RunModel (1,1) logical=false
end

componentDir=fileparts(mfilename("fullpath"));
root=fileparts(fileparts(componentDir));
addpath(genpath(fullfile(root,"Models"))); addpath(fullfile(root,"Data"));
setupRoadsense;

figureHandle=figure("Name","Roadsense - Full Stack 3D Confidence Demo", ...
    "NumberTitle","off","Color",[0.035 0.055 0.075], ...
    "Position",[80 70 1280 760]);
statusText=annotation(figureHandle,"textbox",[0.03 0.82 0.42 0.13], ...
    "String",sprintf("ROADSENSE CONFIDENCE DEMO\n" + ...
    "Running the complete closed-loop Simulink model...\n" + ...
    "Perception > Fusion > Prediction > Planning > Control"), ...
    "Color","white","EdgeColor",[0.1 0.65 0.95],"LineWidth",1.5, ...
    "BackgroundColor",[0.04 0.09 0.14],"FontName","Consolas", ...
    "FontSize",12,"FontWeight","bold");
drawnow;

verifiedResultPath=fullfile(root,"Results","Diagnostics","ConfidenceDemo", ...
    "avoidance_result_verified.mat");
if ~options.RunModel && isfile(verifiedResultPath)
    loaded=load(verifiedResultPath,"result"); result=loaded.result;
    wallTime=result.WallTime;
    statusText.String=sprintf("ROADSENSE CONFIDENCE DEMO\n" + ...
        "Loaded a verified full-stack Simulink result.\n" + ...
        "Starting real-time 3D replay...");
    drawnow;
else
    previousMode=setRoadsenseSemanticInferenceMode("syntheticColor");
    modeCleanup=onCleanup(@() setRoadsenseSemanticInferenceMode(previousMode));
    simulationStart=tic;
    demoOverrides=struct("RsSafetyEmergencyHoldTicks",uint16(1), ...
        "RsSafetyRecoveryTicks",uint16(1), ...
        "RsPlannerMaximumJerk",single(10.0), ...
        "RsPlanStep",single(0.05),"RsPlanHorizon",single(3.0), ...
        "RsPlannerLateralTransitionDistance",single(5.0), ...
        "RsPlannerMaximumCurvature",single(0.5), ...
        "RsPlannerCollisionProbability",single(0.99), ...
        "RsPlannerPredictionSigma",single(1.0), ...
        "RsPlannerMaximumLongitudinalInflation",single(1.5), ...
        "RsPlannerMaximumLateralInflation",single(0.8), ...
        "RsPlannerLateralClearanceRatio",single(0.25), ...
        "RsPlannerMaximumLateralClearance",single(0.6));
    simulationOutput=runRoadsenseScenarioClosedLoopHarnessDemo(6,options.StopTime, ...
        ParameterOverrides=demoOverrides);
    wallTime=toc(simulationStart);
    result=extractRoadsenseValidationResult(simulationOutput,6,wallTime,options.StopTime);
    result.PlanHistory=extractPlanHistory(simulationOutput);
    resultDirectory=fileparts(verifiedResultPath);
    if ~isfolder(resultDirectory); mkdir(resultDirectory); end
    save(verifiedResultPath,"result","-v7.3");
    clear modeCleanup
end

goalReached=logical(result.Metrics.GoalReached(1));
collisionFree=logical(result.Metrics.CollisionFree(1));
avoidanceObserved=max(abs(double(result.Timeline.Position(:,2))))>1.0;
if ~goalReached || ~collisionFree || ~avoidanceObserved
    statusText.String=sprintf("CONFIDENCE RUN DID NOT QUALIFY\n" + ...
        "Goal: %s  Collision-free: %s  Avoidance: %s\n" + ...
        "The viewer was not allowed to hide a failed autonomy run.", ...
        yesNo(goalReached),yesNo(collisionFree),yesNo(avoidanceObserved));
    error("Roadsense:ConfidenceDemo:QualificationFailed", ...
        "Confidence scenario failed: GoalReached=%d, CollisionFree=%d, Avoidance=%d.", ...
        goalReached,collisionFree,avoidanceObserved);
end

delete(statusText);
[ax,car,trail,selectedPlan,statusText]=buildScene(figureHandle,result);
timeline=prepareTimeline(result.Timeline);
framePeriod=0.05;
frameTimes=(timeline.Time(1):framePeriod*options.PlaybackRate:timeline.Time(end)).';
if frameTimes(end)<timeline.Time(end); frameTimes(end+1)=timeline.Time(end); end
playbackClock=tic;
for frame=1:numel(frameTimes)
    if ~isgraphics(figureHandle); return; end
    t=frameTimes(frame);
    targetWallTime=(t-timeline.Time(1))/options.PlaybackRate;
    remaining=targetWallTime-toc(playbackClock);
    if remaining>0; pause(remaining); end

    x=interp1(timeline.Time,timeline.Position(:,1),t,"linear");
    y=interp1(timeline.Time,timeline.Position(:,2),t,"linear");
    yaw=interp1(timeline.Time,timeline.Yaw,t,"linear");
    car.Matrix=makehgtform("translate",[x y 0.03],"zrotate",yaw);
    trail.XData(end+1)=x; trail.YData(end+1)=y; trail.ZData(end+1)=0.08;
    updateSelectedPlan(selectedPlan,result,t);
    updateChaseCamera(ax,x,y,yaw);

    speed=interp1(timeline.EgoTime,timeline.Speed,t,"linear","extrap");
    clearance=interp1(timeline.MetricTime,timeline.Clearance,t,"previous","extrap");
    ready=interp1(timeline.ReadyTime,double(timeline.Ready),t,"previous","extrap")>0.5;
    goal=interp1(timeline.MetricTime,double(timeline.Goal),t,"previous","extrap")>0.5;
    mode=interp1(timeline.BehaviourTime,double(timeline.BehaviourMode),t,"previous","extrap");
    statusText.String=sprintf("ROADSENSE LIVE  |  t = %4.1f s\n" + ...
        "Speed: %4.1f km/h   Pipeline: %s\n" + ...
        "Behaviour: %s\nLateral offset: %4.2f m\n" + ...
        "Clearance: %5.2f m   Collision: NO\nGoal reached: %s", ...
        t,3.6*speed,onOff(ready),behaviourName(mode),y,clearance,yesNo(goal));
    if goal
        statusText.EdgeColor=[0.15 0.9 0.35];
        statusText.BackgroundColor=[0.03 0.20 0.09];
    end
    drawnow;
end

statusText.String=sprintf("CONFIDENCE RUN PASSED\n" + ...
    "Goal: YES  Collision-free: YES  Avoidance: YES\n" + ...
    "Clearance: %.2f m  Max lateral: %.2f m\n" + ...
    "Full Simulink wall time: %.1f s",result.Metrics.MinimumClearanceM(1), ...
    max(abs(double(result.Timeline.Position(:,2)))),wallTime);
statusText.EdgeColor=[0.15 0.95 0.35];
statusText.BackgroundColor=[0.025 0.22 0.09];
fprintf("\nROADSENSE CONFIDENCE RUN PASSED\n");
fprintf("Goal reached: YES | Collision-free: YES | Minimum clearance: %.2f m\n", ...
    result.Metrics.MinimumClearanceM(1));
end

function [ax,car,trail,selectedPlan,statusText]=buildScene(figureHandle,result)
clf(figureHandle);
ax=axes(figureHandle,"Position",[0.02 0.03 0.96 0.94], ...
    "Color",[0.52 0.72 0.91],"Projection","perspective");
hold(ax,"on"); axis(ax,"equal"); axis(ax,"vis3d"); grid(ax,"off");
set(ax,"XColor","none","YColor","none","ZColor","none");

route=result.Route; xLimits=[min(route(:,1))-8 max(route(:,1))+10];
terrainX=[xLimits(1) xLimits(2)]; terrainY=[-18 18];
patch(ax,[terrainX(1) terrainX(2) terrainX(2) terrainX(1)], ...
    [terrainY(1) terrainY(1) terrainY(2) terrainY(2)],[-0.06 -0.06 -0.06 -0.06], ...
    [0.25 0.48 0.20],"EdgeColor","none");
patch(ax,[xLimits(1) xLimits(2) xLimits(2) xLimits(1)], ...
    [-3.5 -3.5 3.5 3.5],[0 0 0 0],[0.18 0.19 0.20],"EdgeColor","none");

plot3(ax,route(:,1),route(:,2),0.04*ones(size(route,1),1), ...
    "Color",[0.30 0.75 1.00],"LineWidth",1.4,"LineStyle","--");
theta=linspace(0,2*pi,80);
goal=route(end,1:2);
[definition,~,~]=RoadsenseScenarioCatalog(double(result.ScenarioID),0);
goalRadius=double(definition.GoalRadius);
fill3(ax,goal(1)+goalRadius*cos(theta),goal(2)+goalRadius*sin(theta), ...
    0.055*ones(size(theta)),[0.10 0.95 0.25], ...
    "FaceAlpha",0.38,"EdgeColor",[0.1 1 0.3],"LineWidth",2);
text(ax,goal(1),goal(2),0.35,"GOAL","Color","white", ...
    "FontWeight","bold","HorizontalAlignment","center");

addRoadsideScene(ax,result);
car=createCar(ax);
trail=plot3(ax,nan,nan,nan,"Color",[0.1 0.75 1],"LineWidth",2.5);
selectedPlan=plot3(ax,nan,nan,nan,"Color",[1.0 0.15 0.8], ...
    "LineWidth",2.0,"LineStyle","-");
xlim(ax,xLimits); ylim(ax,terrainY); zlim(ax,[0 9]);
camproj(ax,"perspective"); lighting(ax,"gouraud");
light(ax,"Position",[2 -4 10],"Style","infinite","Color",[1 0.96 0.88]);
material(ax,"dull");
title(ax,"ROADSENSE - FULL CLOSED-LOOP SIMULINK CONFIDENCE DEMO", ...
    "Color","white","FontSize",15,"FontWeight","bold");
statusText=annotation(figureHandle,"textbox",[0.03 0.71 0.32 0.23], ...
    "String","Preparing real-time replay...","Color","white", ...
    "EdgeColor",[0.1 0.65 0.95],"LineWidth",1.5, ...
    "BackgroundColor",[0.03 0.08 0.13],"FontName","Consolas", ...
    "FontSize",11,"FontWeight","bold");
end

function car=createCar(ax)
car=hgtransform("Parent",ax);
blue=[0.02 0.32 0.82]; darkBlue=[0.015 0.16 0.38];
addBox(car,[0 0 0.58],[3.9 1.82 0.62],blue);
addBox(car,[0.65 0 0.98],[1.35 1.72 0.32],blue);
addCabin(car,[-0.45 0 1.20],[1.85 1.58 0.82],darkBlue);
addBox(car,[1.96 0 0.62],[0.10 1.68 0.28],[0.06 0.08 0.10]);
addBox(car,[-1.96 0 0.61],[0.10 1.70 0.24],[0.05 0.06 0.08]);
addBox(car,[1.98 0.58 0.78],[0.04 0.42 0.20],[1.0 0.95 0.72]);
addBox(car,[1.98 -0.58 0.78],[0.04 0.42 0.20],[1.0 0.95 0.72]);
addBox(car,[-1.98 0.58 0.74],[0.04 0.38 0.20],[0.90 0.06 0.03]);
addBox(car,[-1.98 -0.58 0.74],[0.04 0.38 0.20],[0.90 0.06 0.03]);
for x=[-1.25 1.25]
    for y=[-0.94 0.94]; addWheel(car,[x y 0.43]); end
end
end

function addBox(parent,center,dimensions,color)
d=dimensions/2;
vertices=[-d(1) -d(2) -d(3); d(1) -d(2) -d(3); d(1) d(2) -d(3); -d(1) d(2) -d(3); ...
    -d(1) -d(2) d(3); d(1) -d(2) d(3); d(1) d(2) d(3); -d(1) d(2) d(3)]+center;
faces=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;5 1 4 8];
patch("Parent",parent,"Vertices",vertices,"Faces",faces,"FaceColor",color, ...
    "EdgeColor",color*0.45,"FaceLighting","gouraud");
end

function addCabin(parent,center,dimensions,color)
d=dimensions/2; lowerX=[-d(1) d(1)]; upperX=[-0.65*d(1) 0.58*d(1)];
v=[lowerX(1) -d(2) -d(3);lowerX(2) -d(2) -d(3);lowerX(2) d(2) -d(3);lowerX(1) d(2) -d(3); ...
   upperX(1) -0.82*d(2) d(3);upperX(2) -0.82*d(2) d(3);upperX(2) 0.82*d(2) d(3);upperX(1) 0.82*d(2) d(3)]+center;
f=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1];
patch("Parent",parent,"Vertices",v,"Faces",f,"FaceColor",color, ...
    "EdgeColor",[0.55 0.78 0.95],"FaceLighting","gouraud");
end

function addWheel(parent,center)
theta=linspace(0,2*pi,22); width=0.22; radius=0.39;
[tt,ww]=meshgrid(theta,[-width/2 width/2]);
x=center(1)+radius*cos(tt); y=center(2)+ww; z=center(3)+radius*sin(tt);
surf(x,y,z,"Parent",parent,"FaceColor",[0.025 0.025 0.03], ...
    "EdgeColor","none","FaceLighting","gouraud");
end

function addRoadsideScene(ax,result)
[~,actors]=RoadsenseScenarioCatalog(double(result.ScenarioID),0);
valid=find(actors.ValidMask);
for index=valid(:).'
    position=actors.Positions(index,:);
    if actors.ClassIDs(index)==uint8(7)
        [cx,cy,cz]=cylinder(0.22,16); cz=1.35*cz;
        surf(ax,cx+position(1),cy+position(2),cz,"FaceColor",[0.95 0.55 0.12],"EdgeColor","none");
        [sx,sy,sz]=sphere(16);
        surf(ax,0.28*sx+position(1),0.28*sy+position(2),0.28*sz+position(3)+1.58, ...
            "FaceColor",[0.55 0.30 0.18],"EdgeColor","none");
    elseif actors.ClassIDs(index)==uint8(10)
        actor=hgtransform("Parent",ax);
        actor.Matrix=makehgtform("translate",position);
        addBox(actor,[0 0 0.62],[1.6 1.4 1.24],[0.88 0.12 0.04]);
        addBox(actor,[0 0 1.32],[1.8 1.55 0.16],[1.0 0.72 0.05]);
        radius=3.0; angle=linspace(0,2*pi,80);
        plot3(ax,position(1)+radius*cos(angle),position(2)+radius*sin(angle), ...
            0.07*ones(size(angle)),"r--","LineWidth",1.3);
        text(ax,position(1),position(2),2.0,"FUSED OBSTACLE", ...
            "Color",[1 0.25 0.15],"FontWeight","bold", ...
            "HorizontalAlignment","center");
    else
        actor=hgtransform("Parent",ax);
        actor.Matrix=makehgtform("translate",position);
        addBox(actor,[0 0 0.65],[2.8 1.4 1.3],[0.92 0.65 0.05]);
        addBox(actor,[-0.25 0 1.48],[1.35 1.30 0.55],[0.08 0.38 0.18]);
        for wx=[-0.9 0.9]
            for wy=[-0.72 0.72]; addWheel(actor,[wx wy 0.34]); end
        end
    end
end
for x=[-2 2 7 13 17]
    y=7.5*(-1)^(round(x)+1);
    [tx,ty,tz]=cylinder(0.16,12); surf(ax,tx+x,ty+y,2.0*tz,"FaceColor",[0.32 0.18 0.07],"EdgeColor","none");
    [sx,sy,sz]=sphere(14); surf(ax,1.25*sx+x,1.25*sy+y,1.25*sz+2.8, ...
        "FaceColor",[0.10 0.42 0.12],"EdgeColor","none");
end
end

function updateChaseCamera(ax,x,y,yaw)
forward=[cos(yaw) sin(yaw) 0]; left=[-sin(yaw) cos(yaw) 0];
campos(ax,[x y 0.7]-8.5*forward-5.0*left+[0 0 4.2]);
camtarget(ax,[x y 0.7]+3.0*forward); camup(ax,[0 0 1]);
end

function timeline=prepareTimeline(source)
timeline.Time=double(source.EgoTime(:));
timeline.Position=double(source.Position(:,1:2));
[timeline.Time,indices]=unique(timeline.Time,"stable");
timeline.Position=timeline.Position(indices,:);
velocityX=gradient(timeline.Position(:,1),timeline.Time);
velocityY=gradient(timeline.Position(:,2),timeline.Time);
timeline.Yaw=unwrap(atan2(velocityY,velocityX));
egoTime=double(source.EgoTime(:)); egoSpeed=double(source.Speed(:));
[timeline.EgoTime,indices]=unique(egoTime,"stable"); timeline.Speed=egoSpeed(indices);
metricTime=double(source.Time(:)); clearance=double(source.MinimumClearance(:));
goal=logical(source.GoalReached(:));
[timeline.MetricTime,indices]=unique(metricTime,"stable");
timeline.Clearance=clearance(indices); timeline.Goal=goal(indices);
readyTime=double(source.ReadyTime(:)); ready=logical(source.PipelineReady(:));
[timeline.ReadyTime,indices]=unique(readyTime,"stable"); timeline.Ready=ready(indices);
behaviourTime=double(source.BehaviourTime(:)); behaviourMode=double(source.BehaviourMode(:));
[timeline.BehaviourTime,indices]=unique(behaviourTime,"stable");
timeline.BehaviourMode=behaviourMode(indices);
end

function history=extractPlanHistory(simulationOutput)
history=struct("Time",zeros(0,1),"Positions",zeros(0,61,2), ...
    "Valid",false(0,1));
element=simulationOutput.logsout.getElement("LocalPlan");
if isempty(element); return; end
plan=element.Values; time=double(plan.Positions.Time(:)); data=plan.Positions.Data;
if ndims(data)~=3; return; end
if size(data,1)==61 && size(data,2)==2 && size(data,3)==numel(time)
    positions=permute(double(data),[3 1 2]);
elseif size(data,1)==numel(time) && size(data,2)==61 && size(data,3)==2
    positions=double(data);
else
    return
end
valid=logical(squeeze(plan.Valid.Data)); valid=valid(:);
[time,indices]=unique(time,"stable");
history=struct("Time",time,"Positions",positions(indices,:,:), ...
    "Valid",valid(indices));
end

function updateSelectedPlan(lineHandle,result,time)
if ~isfield(result,"PlanHistory") || isempty(result.PlanHistory.Time); return; end
history=result.PlanHistory; [~,index]=min(abs(history.Time-time));
if ~history.Valid(index); lineHandle.XData=nan; lineHandle.YData=nan; return; end
positions=squeeze(history.Positions(index,:,:));
lineHandle.XData=positions(:,1); lineHandle.YData=positions(:,2);
lineHandle.ZData=0.11*ones(size(positions,1),1);
end

function value=yesNo(flag)
if flag; value="YES"; else; value="NO"; end
end
function value=onOff(flag)
if flag; value="READY"; else; value="STARTING"; end
end
function value=behaviourName(code)
switch round(code)
    case 1; value="CRUISE";
    case 2; value="CAUTIOUS";
    case 4; value="YIELD";
    case 7; value="GOAL STOP";
    case 8; value="EMERGENCY BRAKE";
    case 10; value="AVOID OBSTACLE";
    otherwise; value="INITIALIZING";
end
end
