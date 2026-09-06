function result = runRoadsenseSensorFusionDemo(sequence,saveFigure)
%RUNROADSENSESENSORFUSIONDEMO Track mixed traffic and visualize performance.

arguments
    sequence = createSyntheticRoadsenseFusionSequence()
    saveFigure (1,1) logical = false
end

numberSteps = numel(sequence);
numberObjects = 3;
times = zeros(numberSteps,1);
truth = nan(numberSteps,numberObjects,3);
estimates = nan(numberSteps,numberObjects,3);
velocities = nan(numberSteps,numberObjects,3);
existence = nan(numberSteps,numberObjects);
trackIDs = zeros(numberSteps,numberObjects,"uint32");
sensorMasks = zeros(numberSteps,numberObjects,"uint8");
confirmed = false(numberSteps,numberObjects);
trackCounts = zeros(numberSteps,1,"uint16");
processingTimes = zeros(numberSteps,1,"single");
classIDs = uint8([1 4 7]);

fusion = RoadsenseSensorFusionSystem;
for step = 1:numberSteps
    sample = sequence(step);
    [count,ids,classes,~,probabilities,positions,trackVelocities,~,~,~,~,~,~, ...
        masks,isConfirmed,~,processingTime,~,~,valid] = ...
        fusion(sample.Time,sample.Camera,sample.Lidar,sample.Radar);
    if ~valid; error("Roadsense:FusionInvalid","Fusion rejected sample %d.",step); end
    times(step) = sample.Time;
    truth(step,:,:) = sample.TruthPositions;
    trackCounts(step) = count;
    processingTimes(step) = processingTime;
    for objectIndex = 1:numberObjects
        index = find(classes(1:double(count)) == classIDs(objectIndex),1);
        if isempty(index); continue; end
        estimates(step,objectIndex,:) = positions(index,:);
        velocities(step,objectIndex,:) = trackVelocities(index,:);
        existence(step,objectIndex) = probabilities(index);
        trackIDs(step,objectIndex) = ids(index);
        sensorMasks(step,objectIndex) = masks(index);
        confirmed(step,objectIndex) = isConfirmed(index);
    end
end

errors = vecnorm(estimates(:,:,1:2)-truth(:,:,1:2),2,3);
labels = ["Car","Auto-rickshaw","Pedestrian"];
colors = lines(numberObjects);
figure(Name="Roadsense Sensor Fusion",Color="white");
tiledlayout(2,2,Padding="compact",TileSpacing="compact");

nexttile([2 1]); hold on; grid on; axis equal;
for objectIndex = 1:numberObjects
    plot(truth(:,objectIndex,1),truth(:,objectIndex,2),"--", ...
        Color=colors(objectIndex,:),LineWidth=1.2,DisplayName=labels(objectIndex)+" truth");
    plot(estimates(:,objectIndex,1),estimates(:,objectIndex,2),"-", ...
        Color=colors(objectIndex,:),LineWidth=2.0,DisplayName=labels(objectIndex)+" fused");
end
xlabel("Forward x (m)"); ylabel("Left y (m)");
title("Truth and persistent fused tracks"); legend(Location="best");

nexttile; hold on; grid on;
for objectIndex = 1:numberObjects
    plot(times,errors(:,objectIndex),LineWidth=1.5,Color=colors(objectIndex,:), ...
        DisplayName=labels(objectIndex));
end
xlabel("Time (s)"); ylabel("2-D position error (m)"); title("Tracking error");
legend(Location="best");

nexttile; hold on; grid on;
for objectIndex = 1:numberObjects
    plot(times,existence(:,objectIndex),LineWidth=1.5,Color=colors(objectIndex,:), ...
        DisplayName=labels(objectIndex));
end
yline(0.85,":","confirmation threshold");
xlabel("Time (s)"); ylabel("Existence probability"); ylim([0 1.05]);
title("JIPDA confidence through missed detections"); legend(Location="best");

if saveFigure
    componentDir = fileparts(mfilename("fullpath"));
    root = fileparts(fileparts(componentDir));
    outputDir = fullfile(root,"Results","SensorFusion");
    if ~isfolder(outputDir); mkdir(outputDir); end
    exportgraphics(gcf,fullfile(outputDir,"mixed_traffic_tracking_demo.png"),Resolution=180);
end

result = struct("Time",times,"TruthPositions",truth,"EstimatedPositions",estimates, ...
    "EstimatedVelocities",velocities,"PositionErrors",errors,"Existence",existence, ...
    "TrackIDs",trackIDs,"SensorMasks",sensorMasks,"Confirmed",confirmed, ...
    "TrackCounts",trackCounts,"ProcessingTimes",processingTimes);
end
