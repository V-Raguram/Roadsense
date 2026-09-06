function result=runRoadsenseVehicleDynamicsDemo(scenario,showPlot)
%RUNROADSENSEVEHICLEDYNAMICSDEMO Simulate and visualize one plant case.
arguments
    scenario (1,1) string {mustBeMember(scenario,["straight","turn","brake", ...
        "low_friction","uphill","invalid"])}="turn"
    showPlot (1,1) logical=true
end
[control,initial,road]=createSyntheticRoadsenseVehicleDynamicsInputs(scenario);
plant=RoadsenseVehicleDynamicsSystem; dt=0.02; duration=5; numberSteps=round(duration/dt)+1;
time=(0:numberSteps-1).'*dt; position=zeros(numberSteps,3); speed=zeros(numberSteps,1);
lateralSpeed=zeros(numberSteps,1); yaw=zeros(numberSteps,1); yawRate=zeros(numberSteps,1);
sideslip=zeros(numberSteps,1); frontSlip=zeros(numberSteps,1); rearSlip=zeros(numberSteps,1);
longitudinalAcceleration=zeros(numberSteps,1); lateralAcceleration=zeros(numberSteps,1);
frictionUtilization=zeros(numberSteps,1); steering=zeros(numberSteps,1);
saturated=false(numberSteps,1);
for step=1:numberSteps
    control.Timestamp=time(step); road.Timestamp=time(step);
    [out{1:24}]=plant(control,initial,road,step==1);
    position(step,:)=out{2}.'; speed(step)=out{12}; lateralSpeed(step)=out{13};
    yaw(step)=out{5}; yawRate(step)=out{8}; sideslip(step)=out{14};
    frontSlip(step)=out{15}; rearSlip(step)=out{16};
    longitudinalAcceleration(step)=out{18}; lateralAcceleration(step)=out{19};
    frictionUtilization(step)=out{20}; steering(step)=out{21}; saturated(step)=out{23};
end
result=struct("Scenario",scenario,"Time",time,"Position",position,"Speed",speed, ...
    "LateralSpeed",lateralSpeed,"Yaw",yaw,"YawRate",yawRate,"Sideslip",sideslip, ...
    "FrontSlip",frontSlip,"RearSlip",rearSlip, ...
    "LongitudinalAcceleration",longitudinalAcceleration, ...
    "LateralAcceleration",lateralAcceleration,"FrictionUtilization",frictionUtilization, ...
    "Steering",steering,"Saturated",saturated);
if ~showPlot; return; end

figure("Name","Roadsense Vehicle Dynamics - "+scenario,"Color","w", ...
    "Position",[100 100 1200 850]); tiledlayout(3,1,"Padding","compact","TileSpacing","compact");
ax1=nexttile; plot(position(:,1),position(:,2),"r-","LineWidth",2); hold on;
scatter(position(1,1),position(1,2),40,[0.1 0.4 0.9],"filled"); grid on;
if max(position(:,2))-min(position(:,2))>1
    axis equal;
else
    axis padded; ylim([-2 2]);
end
xlabel("local-world x (m)"); ylabel("local-world y (m)");
title("Dynamic bicycle vehicle path","Color","k"); legend("vehicle path","start");
set(ax1,"XColor","k","YColor","k","Color","w");
ax2=nexttile; plot(time,speed,"LineWidth",1.7); hold on;
plot(time,lateralSpeed,"LineWidth",1.4); plot(time,rad2deg(yawRate),"LineWidth",1.4); grid on;
xlabel("time (s)"); ylabel("state"); legend("longitudinal speed (m/s)", ...
    "lateral speed (m/s)","yaw rate (deg/s)","Location","best");
title("Body-frame motion states","Color","k"); set(ax2,"XColor","k","YColor","k","Color","w");
ax3=nexttile; plot(time,longitudinalAcceleration,"LineWidth",1.6); hold on;
plot(time,lateralAcceleration,"LineWidth",1.5); plot(time,frictionUtilization,"LineWidth",1.5); grid on;
xlabel("time (s)"); ylabel("dynamics"); legend("longitudinal acceleration (m/s^2)", ...
    "lateral acceleration (m/s^2)","friction utilization","Location","best");
title(sprintf("mu=%.2f, saturated samples=%d",road.FrictionCoefficient,nnz(saturated)),"Color","k");
set(ax3,"XColor","k","YColor","k","Color","w");
end
