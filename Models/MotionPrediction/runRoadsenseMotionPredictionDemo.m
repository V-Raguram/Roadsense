function result = runRoadsenseMotionPredictionDemo(saveFigure)
%RUNROADSENSEMOTIONPREDICTIONDEMO Visualize class-aware future trajectories.

arguments
    saveFigure (1,1) logical = false
end

predictor = RoadsenseMotionPredictionSystem;
for time = 0:0.1:1.0
    tracks = createSyntheticRoadsenseTrackList(time);
    [count,trackIDs,classIDs,numModes,numSteps,timeOffsets,modeProbabilities, ...
        positions,yaws,covariances,validMask,processingTime,overflow,valid] = predictor(tracks);
end

labels = ["Car","Auto-rickshaw","Pedestrian","Animal","Static obstacle"];
modeLabels = ["Continue","Stop","Left deviation","Right deviation"];
modeStyles = ["-","--","-.",":"];
colors = lines(double(count));
figure(Name="Roadsense Motion Prediction",Color="white");
tiledlayout(2,2,Padding="compact",TileSpacing="compact");

nexttile([2 1]); hold on; grid on; axis equal;
for objectIndex = 1:double(count)
    plot(tracks.Positions(objectIndex,1),tracks.Positions(objectIndex,2),"o", ...
        MarkerFaceColor=colors(objectIndex,:),MarkerEdgeColor=colors(objectIndex,:), ...
        DisplayName=labels(objectIndex));
    for mode = 1:double(numModes(objectIndex))
        x = squeeze(positions(objectIndex,1:double(numSteps),mode,1));
        y = squeeze(positions(objectIndex,1:double(numSteps),mode,2));
        plot(x,y,modeStyles(mode),Color=colors(objectIndex,:),LineWidth=1.6, ...
            HandleVisibility="off");
        drawEllipse([x(end) y(end)],covariances(:,:,double(numSteps),mode,objectIndex), ...
            colors(objectIndex,:));
    end
end
xlabel("Forward x (m)"); ylabel("Left y (m)");
title("Three-second multimodal futures with 1-sigma endpoints");
legend(Location="best");

nexttile;
bar(categorical(labels),modeProbabilities(1:double(count),:),"stacked");
ylabel("Probability"); ylim([0 1]); grid on;
title("Class- and history-conditioned mode probabilities");
legend(modeLabels,Location="southoutside");

nexttile; hold on; grid on;
for objectIndex = 1:double(count)
    activeModes = double(numModes(objectIndex));
    terminalTrace = zeros(activeModes,1);
    for mode = 1:activeModes
        terminalTrace(mode) = trace(double( ...
            covariances(:,:,double(numSteps),mode,objectIndex)));
    end
    scatter(repmat(objectIndex,activeModes,1),terminalTrace,45, ...
        (1:activeModes).',"filled");
end
xticks(1:double(count)); xticklabels(labels); xtickangle(20);
ylabel("Terminal covariance trace (m^2)");
title("Uncertainty expansion by mode");

if saveFigure
    componentDir = fileparts(mfilename("fullpath"));
    root = fileparts(fileparts(componentDir));
    outputDir = fullfile(root,"Results","MotionPrediction");
    if ~isfolder(outputDir); mkdir(outputDir); end
    exportgraphics(gcf,fullfile(outputDir,"class_aware_multimodal_demo.png"),Resolution=180);
end

result = struct("Timestamp",tracks.Timestamp,"Count",count,"TrackIDs",trackIDs, ...
    "ClassIDs",classIDs,"NumModes",numModes,"NumSteps",numSteps, ...
    "TimeOffsets",timeOffsets,"ModeProbabilities",modeProbabilities, ...
    "Positions",positions,"Yaws",yaws,"PositionCovariances",covariances, ...
    "ValidMask",validMask,"ProcessingTime",processingTime, ...
    "Overflow",overflow,"Valid",valid);
end

function drawEllipse(centre,covariance,color)
[vectors,values] = eig((double(covariance)+double(covariance).')/2);
radii = sqrt(max(diag(values),0));
angles = linspace(0,2*pi,40);
points = vectors*diag(radii)*[cos(angles);sin(angles)];
plot(centre(1)+points(1,:),centre(2)+points(2,:),Color=[color 0.35], ...
    LineWidth=0.8,HandleVisibility="off");
end
