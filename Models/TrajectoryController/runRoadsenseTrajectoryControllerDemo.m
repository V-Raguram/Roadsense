function result=runRoadsenseTrajectoryControllerDemo(scenario,showPlot)
%RUNROADSENSETRAJECTORYCONTROLLERDEMO Closed-loop kinematic tracking demo.
arguments
    scenario (1,1) string {mustBeMember(scenario,["offset","curved"])}="offset"
    showPlot (1,1) logical=true
end
[ego,plan,status]=createSyntheticRoadsenseControllerInputs(scenario);
controller=RoadsenseTrajectoryControllerSystem; dt=0.02; numberSteps=201;
time=(0:numberSteps-1).'*dt; position=zeros(numberSteps,2); yaw=zeros(numberSteps,1);
speed=zeros(numberSteps,1); crossTrack=zeros(numberSteps,1); heading=zeros(numberSteps,1);
speedError=zeros(numberSteps,1); steering=zeros(numberSteps,1); acceleration=zeros(numberSteps,1);
throttle=zeros(numberSteps,1); brake=zeros(numberSteps,1);
for step=1:numberSteps
    ego.Timestamp=plan.Timestamp+time(step);
    [out{1:22}]=controller(ego,plan,status);
    position(step,:)=ego.Position(1:2).'; yaw(step)=ego.Yaw;
    speed(step)=norm(ego.Velocity(1:2)); crossTrack(step)=double(out{14});
    heading(step)=double(out{15}); speedError(step)=double(out{16});
    steering(step)=double(out{3}); acceleration(step)=double(out{5});
    throttle(step)=double(out{6}); brake(step)=double(out{7});
    if step<numberSteps
        nextSpeed=max(0,speed(step)+acceleration(step)*dt);
        ego.Yaw=wrapLocal(ego.Yaw+nextSpeed/2.8*tan(steering(step))*dt);
        ego.Position(1:2)=ego.Position(1:2)+ ...
            nextSpeed*[cos(ego.Yaw);sin(ego.Yaw)]*dt;
        ego.Velocity(1:2)=nextSpeed*[cos(ego.Yaw);sin(ego.Yaw)];
        ego.Acceleration(1:2)=acceleration(step)*[cos(ego.Yaw);sin(ego.Yaw)];
        ego.SteeringAngle=steering(step);
    end
end
result=struct("Scenario",scenario,"Time",time,"Position",position,"Yaw",yaw, ...
    "Speed",speed,"CrossTrackError",crossTrack,"HeadingError",heading, ...
    "SpeedError",speedError,"SteeringAngle",steering,"Acceleration",acceleration, ...
    "Throttle",throttle,"Brake",brake,"Plan",plan, ...
    "RmsCrossTrackError",sqrt(mean(crossTrack.^2)), ...
    "FinalCrossTrackError",abs(crossTrack(end)), ...
    "RmsSpeedError",sqrt(mean(speedError.^2)));
if ~showPlot; return; end

n=double(plan.Count); figure("Name","Roadsense Trajectory Controller - "+scenario,"Color","w");
tiledlayout(3,1);
ax1=nexttile; hold on; grid on; axis equal;
plot(plan.Positions(1:n,1),plan.Positions(1:n,2),"k--","LineWidth",1.4);
plot(position(:,1),position(:,2),"r-","LineWidth",2);
scatter(position(1,1),position(1,2),40,[0.1 0.4 0.9],"filled");
xlabel("local-world x (m)"); ylabel("local-world y (m)");
title("Planned and controlled vehicle paths","Color","k");
legend("local plan","controlled ego","start","Location","best");
set(ax1,"XColor","k","YColor","k","Color","w");

ax2=nexttile; hold on; grid on;
plot(time,crossTrack,"LineWidth",1.7); plot(time,heading,"LineWidth",1.5);
plot(time,speedError,"LineWidth",1.4);
xlabel("time (s)"); ylabel("tracking error");
legend("cross-track (m)","heading (rad)","speed error (m/s)","Location","best");
title(sprintf("RMS cross-track %.3f m, final %.3f m", ...
    result.RmsCrossTrackError,result.FinalCrossTrackError),"Color","k");
set(ax2,"XColor","k","YColor","k","Color","w");

ax3=nexttile; hold on; grid on;
plot(time,rad2deg(steering),"LineWidth",1.7); plot(time,acceleration,"LineWidth",1.5);
xlabel("time (s)"); ylabel("command");
legend("steering (deg)","acceleration (m/s^2)","Location","best");
title("Rate-limited control commands","Color","k");
set(ax3,"XColor","k","YColor","k","Color","w");
end

function angle=wrapLocal(angle)
angle=mod(angle+pi,2*pi)-pi;
end
