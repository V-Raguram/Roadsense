function [imageSemantic,lidarGrid,tracks,predictions] = createSyntheticRoadsenseMapInputs()
%CREATESYNTHETICROADSENSEMAPINPUTS Reproducible inputs for map-fusion tests.

timestamp = 1.0;
semanticRows = 120;
semanticCols = 160;
labels = zeros(semanticRows,semanticCols,"uint8");
confidence = zeros(semanticRows,semanticCols,"single");
drivable = zeros(semanticRows,semanticCols,"single");
roadMask = false(semanticRows,semanticCols);
roadMask(62:end,34:127) = true;
labels(roadMask) = uint8(1);
confidence(roadMask) = single(0.92);
drivable(roadMask) = single(0.93);
footpathMask = false(semanticRows,semanticCols);
footpathMask(72:end,1:33) = true;
footpathMask(72:end,128:end) = true;
labels(footpathMask) = uint8(4);
confidence(footpathMask) = single(0.85);
drivable(footpathMask) = single(0.08);
cameraPothole = false(semanticRows,semanticCols);
cameraPothole(91:101,76:86) = true;
labels(cameraPothole) = uint8(5);
confidence(cameraPothole) = single(0.80);
drivable(cameraPothole) = single(0.12);
imageSemantic = struct("Timestamp",timestamp,"FrameID",uint32(11), ...
    "SemanticLabel",labels,"Confidence",confidence, ...
    "DrivableProbability",drivable,"InferenceTime",single(0.02), ...
    "NetworkReady",true,"Valid",true);

resolution = single(0.25);
xLimits = single([-20;60]);
yLimits = single([-25;25]);
x = double(xLimits(1))+(0.5:319.5)*double(resolution);
y = double(yLimits(1))+(0.5:199.5)*double(resolution);
[xGrid,yGrid] = meshgrid(x,y);
observed = xGrid >= -5 & xGrid <= 48 & abs(yGrid) <= 11;
staticOccupancy = zeros(200,320,"single");
staticObstacle = (xGrid-28).^2+(yGrid+5).^2 <= 1.2^2;
staticOccupancy(staticObstacle) = single(0.92);
pothole = exp(-((xGrid-20).^2/1.4^2+(yGrid+7).^2/0.8^2));
potholeProbability = single(0.80*pothole);
roughness = single(0.015*observed+0.06*pothole);
surfaceCost = max(potholeProbability,min(roughness/single(0.05),single(1)));
lidarGrid = struct("Timestamp",timestamp,"FrameID",uint32(11), ...
    "Resolution",resolution,"XLimits",xLimits,"YLimits",yLimits, ...
    "Occupancy",staticOccupancy,"GroundHeight",zeros(200,320,"single"), ...
    "Roughness",roughness,"PotholeProbability",potholeProbability, ...
    "SurfaceCost",surfaceCost,"ObservedMask",observed, ...
    "ProcessingTime",single(0.01),"Valid",true);

predictor = RoadsenseMotionPredictionSystem;
for time = 0:0.1:timestamp
    tracks = createSyntheticRoadsenseTrackList(time);
    [count,trackIDs,classIDs,numModes,numSteps,timeOffsets,modeProbabilities, ...
        positions,yaws,positionCovariances,validMask,processingTime,overflow,valid] = ...
        predictor(tracks);
end
predictions = struct("Timestamp",timestamp,"Count",count,"TrackIDs",trackIDs, ...
    "ClassIDs",classIDs,"NumModes",numModes,"NumSteps",numSteps, ...
    "TimeOffsets",timeOffsets,"ModeProbabilities",modeProbabilities, ...
    "Positions",positions,"Yaws",yaws, ...
    "PositionCovariances",positionCovariances,"ValidMask",validMask, ...
    "ProcessingTime",processingTime,"Overflow",overflow,"Valid",valid);
end
