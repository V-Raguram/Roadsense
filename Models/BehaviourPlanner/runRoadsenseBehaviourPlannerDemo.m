function result=runRoadsenseBehaviourPlannerDemo(saveFigure)
%RUNROADSENSEBEHAVIOURPLANNERDEMO Exercise supervisory modes and hysteresis.

arguments
    saveFigure (1,1) logical=false
end
sampleTime=0.1; times=(0:sampleTime:10).'; numberSamples=numel(times);
modes=zeros(numberSamples,1,"uint8"); targetSpeeds=zeros(numberSamples,1,"single");
egoSpeeds=zeros(numberSamples,1,"single"); risks=zeros(numberSamples,1,"single");
unknown=zeros(numberSamples,1,"single"); minimumTTC=zeros(numberSamples,1,"single");
replans=false(numberSamples,1); emergencies=false(numberSamples,1);
scenarios=strings(numberSamples,1);
assessment=RoadsenseBehaviourAssessmentSystem;
command=RoadsenseBehaviourCommandSystem;
previousMode=uint8(0); emergencyCounter=uint16(0); yieldCounter=uint16(8);

for index=1:numberSamples
    scenario=scenarioAt(times(index)); scenarios(index)=scenario;
    [ego,map,tracks,route]=createSyntheticRoadsenseBehaviourInputs(scenario);
    ego.Timestamp=times(index); map.Timestamp=times(index);
    tracks.Timestamp=times(index); route.Timestamp=times(index);
    [a{1:22}]=assessment(ego,map,tracks,route);
    [mode,emergencyCounter,yieldCounter]=RoadsenseBehaviourDecisionCore( ...
        a{2},a{3},a{4},a{5},a{6},a{7},a{8},a{9},previousMode, ...
        emergencyCounter,yieldCounter,uint16(5),uint16(8));
    previousMode=mode;
    [c{1:16}]=command(a{1},mode,a{2},a{10},a{11},a{12},a{13},a{14},a{15}, ...
        a{16},a{17},a{18},a{19},a{20},a{21},a{22});
    modes(index)=mode; targetSpeeds(index)=c{4};
    egoSpeeds(index)=single(norm(ego.Velocity(1:2)));
    risks(index)=a{20}; unknown(index)=a{21}; minimumTTC(index)=a{19};
    replans(index)=c{14}; emergencies(index)=c{15};
end

figure(Name="Roadsense Behaviour Planner",Color="white");
tiledlayout(3,1,Padding="compact",TileSpacing="compact");
nexttile;
stairs(times,double(modes),LineWidth=1.8); grid on;
yticks([1 2 3 4 5 7 8 9 10]);
yticklabels(["Cruise","Cautious","Follow","Yield","Creep","Stop", ...
    "Emergency","Minimal risk","Avoid"]);
ylabel("State"); title("Stateflow supervisory mode with hold/clear hysteresis");
nexttile; hold on; grid on;
plot(times,egoSpeeds,"--",LineWidth=1.2,DisplayName="Ego speed");
plot(times,targetSpeeds,LineWidth=1.8,DisplayName="Commanded target");
ylabel("Speed (m/s)"); legend(Location="best");
nexttile; hold on; grid on;
plot(times,risks,LineWidth=1.4,DisplayName="Forward risk");
plot(times,unknown,LineWidth=1.4,DisplayName="Unknown fraction");
ttcPlot=min(minimumTTC,single(8))/single(8);
plot(times,ttcPlot,LineWidth=1.4,DisplayName="TTC / 8 s");
stem(times(replans),ones(nnz(replans),1),"k.",DisplayName="Mode change");
xlabel("Time (s)"); ylabel("Normalized cue"); ylim([0 1.05]);
legend(Location="bestoutside");

if saveFigure
    componentDir=fileparts(mfilename("fullpath")); root=fileparts(fileparts(componentDir));
    outputDir=fullfile(root,"Results","BehaviourPlanner");
    if ~isfolder(outputDir); mkdir(outputDir); end
    exportgraphics(gcf,fullfile(outputDir,"supervisory_state_demo.png"),Resolution=180);
end
result=struct("Time",times,"Scenario",scenarios,"ModeCodes",modes, ...
    "TargetSpeeds",targetSpeeds,"EgoSpeeds",egoSpeeds,"ForwardRisk",risks, ...
    "UnknownFraction",unknown,"MinimumTTC",minimumTTC, ...
    "ReplanRequested",replans,"EmergencyRequested",emergencies);
end

function scenario=scenarioAt(time)
if time < 1; scenario="cruise";
elseif time < 2; scenario="follow";
elseif time < 3; scenario="yield";
elseif time < 4.2; scenario="cruise";
elseif time < 5.2; scenario="creep";
elseif time < 6.2; scenario="avoid";
elseif time < 7.2; scenario="cautious";
elseif time < 7.4; scenario="emergency";
elseif time < 8.3; scenario="cruise";
elseif time < 9.2; scenario="stop";
else; scenario="invalid";
end
end
