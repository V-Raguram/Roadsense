function result=runRoadsenseSafetySupervisorDemo(showPlot)
%RUNROADSENSESAFETYSUPERVISORDEMO Exercise priority, latch, stop, and recovery.
arguments
    showPlot (1,1) logical=true
end
[ego,map,tracks,behaviour,planner,tracking,dynamics,nominal]= ...
    createSyntheticRoadsenseSafetyInputs("normal");
assessment=RoadsenseSafetyAssessmentSystem; gate=RoadsenseSafetyCommandSystem;
dt=0.02; duration=6; samples=round(duration/dt)+1; time=(0:samples-1).'*dt;
state=zeros(samples,1,"uint8"); reason=zeros(samples,1,"uint16");
acceleration=zeros(samples,1); brake=zeros(samples,1); override=false(samples,1);
latched=false(samples,1); safe=false(samples,1); ttc=inf(samples,1);
crossTrack=zeros(samples,1); maximumAge=zeros(samples,1); execution=zeros(samples,1);
previousState=uint8(0); emergencyCounter=uint16(0); recovery=uint16(0);
for index=1:samples
    now=1+time(index);
    ego.Timestamp=now; map.Timestamp=now; tracks.Timestamp=now; behaviour.Timestamp=now;
    planner.Timestamp=now; tracking.Timestamp=now; dynamics.Timestamp=now; nominal.Timestamp=now;
    tracking.CrossTrackError=single(0.05); tracking.HeadingError=single(0.01);
    tracking.SteeringSaturated=false; tracking.AccelerationSaturated=false;
    dynamics.SideslipAngle=single(0); dynamics.FrictionUtilization=single(0.2);
    dynamics.TireForceSaturated=false; planner.Code=RoadsenseTypes.PlannerCode.Success;
    planner.EmergencyRequested=false; behaviour.EmergencyRequested=false;
    tracks.Count=uint16(0); tracks.ValidMask(:)=false; ego.Velocity=[5;0;0];
    if time(index)>=0.5 && time(index)<0.9
        tracking.CrossTrackError=single(1.0);
    elseif time(index)>=1.0 && time(index)<1.02
        tracks.Count=uint16(1); tracks.ValidMask(1)=true; tracks.TrackIDs(1)=uint32(55);
        tracks.Positions(1,:)=single([5 0 0.8]); tracks.Velocities(1,:)=single([-5 0 0]);
        tracks.Dimensions(1,:)=single([4 1.8 1.5]);
    elseif time(index)>=2.5 && time(index)<2.8
        map.Timestamp=now-1;
    elseif time(index)>=3.2 && time(index)<3.9
        ego.Velocity(:)=0;
    elseif time(index)>=4.2 && time(index)<4.7
        planner.Code=RoadsenseTypes.PlannerCode.DeadlineMiss;
    elseif time(index)>=5.2 && time(index)<5.4
        planner.EmergencyRequested=true;
    end
    tick=tic;
    [a{1:14}]=assessment(ego,map,tracks,behaviour,planner,tracking,dynamics,nominal);
    [state(index),emergencyCounter,recovery,latched(index)]=RoadsenseSafetyDecisionCore( ...
        a{2},a{3},a{4},a{5},a{6},previousState,emergencyCounter,recovery,uint16(50),uint16(25));
    [g{1:24}]=gate(nominal,a{1},state(index),a{2},a{7},a{8},a{9},a{10}, ...
        a{11},a{12},a{13},a{14},latched(index));
    execution(index)=toc(tick); previousState=state(index);
    reason(index)=a{14}; acceleration(index)=g{5}; brake(index)=g{7};
    override(index)=g{21}; safe(index)=g{23}; ttc(index)=a{8};
    crossTrack(index)=a{11}; maximumAge(index)=a{10};
end
result=struct("Time",time,"State",state,"ReasonMask",reason, ...
    "Acceleration",acceleration,"Brake",brake,"Override",override, ...
    "EmergencyLatched",latched,"SafeToDrive",safe,"MinimumTTC",ttc, ...
    "CrossTrackError",crossTrack,"MaximumInputAge",maximumAge, ...
    "ExecutionTime",execution);
if ~showPlot; return; end

figure("Name","Roadsense Safety Supervisor","Color","w","Position",[100 100 1200 820]);
tiledlayout(3,1,"Padding","compact","TileSpacing","compact");
ax1=nexttile; stairs(time,double(state),"LineWidth",1.8); grid on; ylim([-0.2 5.5]);
yticks(0:5); yticklabels(["Startup","Normal","Degraded","Emergency","Min-risk stop","Hold"]);
xlabel("time (s)"); ylabel("safety state"); title("Stateflow safety decision","Color","k");
set(ax1,"XColor","k","YColor","k","Color","w");
ax2=nexttile; stairs(time,bitget(reason,3),"LineWidth",1.5); hold on;
stairs(time,bitget(reason,2)+1.2,"LineWidth",1.5);
stairs(time,double(latched)+2.4,"LineWidth",1.5); grid on; ylim([-0.2 3.7]);
yticks([0.5 1.7 2.9]); yticklabels(["collision","stale","emergency latch"]);
xlabel("time (s)"); ylabel("trigger active"); title("Independent evidence and latch","Color","k");
set(ax2,"XColor","k","YColor","k","Color","w");
ax3=nexttile; plot(time,acceleration,"LineWidth",1.8); hold on;
plot(time,brake,"LineWidth",1.5); stairs(time,double(override),"LineWidth",1.3); grid on;
xlabel("time (s)"); ylabel("safe command");
legend("acceleration (m/s^2)","brake","override","Location","best");
title(sprintf("override %.1f%%, steady maximum %.2f ms", ...
    100*mean(override),1000*max(execution(11:end))),"Color","k");
set(ax3,"XColor","k","YColor","k","Color","w");
end
