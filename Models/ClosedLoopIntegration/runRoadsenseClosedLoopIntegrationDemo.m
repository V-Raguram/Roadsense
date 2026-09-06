function result=runRoadsenseClosedLoopIntegrationDemo(showPlot)
%RUNROADSENSECLOSEDLOOPINTEGRATIONDEMO Exercise end-to-end health semantics.
arguments
    showPlot (1,1) logical=true
end
inputs=createSyntheticRoadsenseIntegrationInputs();
monitor=RoadsenseIntegrationMonitorSystem;
time=(0:0.02:6).'; numberSamples=numel(time);
ready=false(numberSamples,1); perception=false(numberSamples,1);
prediction=false(numberSamples,1); override=false(numberSamples,1);
emergency=false(numberSamples,1); goal=false(numberSamples,1);
cycle=zeros(numberSamples,1,"uint32");
for index=1:numberSamples
    current=inputs;
    for busIndex=1:numel(current)
        if isfield(current{busIndex},"Timestamp")
            current{busIndex}.Timestamp=time(index);
        end
    end
    % Startup/warm-up, prediction dropout, emergency intervention, completion.
    if time(index)<0.5; current{2}.Valid=false; end
    if time(index)>=2.0 && time(index)<2.6; current{5}.Valid=false; end
    if time(index)>=3.0 && time(index)<4.0
        current{12}.State=RoadsenseTypes.SafetyState.EmergencyBrake;
        current{12}.OverrideActive=true; current{12}.SafeToDrive=false;
        current{13}.EmergencyStop=true;
        current{13}.AccelerationCommand=single(-6); current{13}.Brake=single(1);
    end
    if time(index)>=5.5; current{15}.GoalReached=true; end
    [~,cycle(index),perception(index),~,prediction(index),~,~,~,~,~,~, ...
        ready(index),override(index),emergency(index),goal(index),~]=monitor(current{:});
end
release(monitor);
result=table(time,cycle,perception,prediction,ready,override,emergency,goal, ...
    'VariableNames',cellstr(["Time","CycleID","PerceptionValid","PredictionValid", ...
    "PipelineReady","SafetyOverride","EmergencyStop","GoalReached"]));
if showPlot
    figure("Name","Roadsense closed-loop integration health", ...
        "Color","white","Position",[100 100 1000 640]);
    tiledlayout(3,1,"TileSpacing","compact");
    nexttile; stairs(time,[perception prediction ready],"LineWidth",1.5); grid on;
    ylim([-0.1 1.1]); legend("Perception","Prediction","Pipeline ready", ...
        "Location","southoutside","Orientation","horizontal"); ylabel("Health");
    title("Roadsense multi-rate integration monitor");
    nexttile; stairs(time,[override emergency],"LineWidth",1.5); grid on;
    ylim([-0.1 1.1]); legend("Safety override","Emergency stop", ...
        "Location","southoutside","Orientation","horizontal"); ylabel("Safety");
    nexttile; stairs(time,goal,"LineWidth",1.5); grid on; ylim([-0.1 1.1]);
    ylabel("Goal reached"); xlabel("Time (s)");
end
end
