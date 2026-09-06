function sequence = createSyntheticRoadsenseFusionSequence(numberSteps,sampleTime,seed)
%CREATESYNTHETICROADSENSEFUSIONSEQUENCE Deterministic mixed-traffic inputs.
%   The sequence contains a car, an auto-rickshaw with lateral wander, and
%   a crossing pedestrian. Sensor noise, missed detections, and a four-frame
%   complete pedestrian dropout exercise spatial fusion and track coasting.

arguments
    numberSteps (1,1) double {mustBeInteger,mustBePositive} = 40
    sampleTime (1,1) double {mustBePositive} = 0.1
    seed (1,1) double {mustBeInteger} = 37
end

stream = RandStream("mt19937ar","Seed",seed);
truthClassIDs = uint8([1;4;7]);
cameraScores = single([0.91;0.84;0.88]);
truthDimensions = single([4.4 1.8 1.6;2.8 1.4 1.8;0.6 0.6 1.75]);
lidarPointCounts = uint32([140;95;35]);
sequence = repmat(struct("Time",0,"Camera",[],"Lidar",[],"Radar",[], ...
    "TruthPositions",zeros(3,3),"TruthVelocities",zeros(3,3), ...
    "TruthClassIDs",uint8([1;4;7])),numberSteps,1);

for step = 1:numberSteps
    time = (step-1)*sampleTime;
    [truthPositions,truthVelocities] = truthAt(time);
    camera = emptyCamera(time,step);
    lidar = emptyLidar(time,step);
    radar = emptyRadar(time,step);

    for objectIndex = 1:3
        % Camera loses the pedestrian in visual occlusion. All three sensors
        % miss it for steps 18:21, testing temporal persistence explicitly.
        cameraVisible = ~(objectIndex == 3 && step >= 14 && step <= 23);
        lidarVisible = ~(objectIndex == 3 && step >= 18 && step <= 21);
        radarVisible = ~(objectIndex == 3 && step >= 10 && step <= 23);

        if cameraVisible
            index = double(camera.Count)+1;
            camera.Count = uint16(index);
            camera.ValidMask(index) = true;
            camera.Positions(index,:) = single(truthPositions(objectIndex,:) + ...
                randn(stream,1,3).*[0.75 0.55 0.25]);
            camera.PositionCovariances(:,:,index) = single(diag([0.75 0.55 0.25].^2));
            camera.ClassIDs(index) = truthClassIDs(objectIndex);
            camera.Scores(index) = cameraScores(objectIndex);
            camera.BoundingBoxes(index,:) = single([100+60*objectIndex 180 55 65]);
        end

        if lidarVisible
            index = double(lidar.Count)+1;
            lidar.Count = uint16(index);
            lidar.ValidMask(index) = true;
            lidar.Positions(index,:) = single(truthPositions(objectIndex,:) + ...
                randn(stream,1,3).*[0.12 0.10 0.08]);
            lidar.PositionCovariances(:,:,index) = single(diag([0.12 0.10 0.08].^2));
            lidar.Dimensions(index,:) = truthDimensions(objectIndex,:);
            lidar.Yaws(index) = single(atan2(truthVelocities(objectIndex,2),truthVelocities(objectIndex,1)));
            lidar.Scores(index) = single(0.93);
            lidar.PointCounts(index) = lidarPointCounts(objectIndex);
        end

        if radarVisible
            index = double(radar.Count)+1;
            radar.Count = uint16(index);
            radar.ValidMask(index) = true;
            radar.Positions(index,:) = single(truthPositions(objectIndex,:) + ...
                randn(stream,1,3).*[0.35 0.30 0.40]);
            radar.Velocities(index,:) = single(truthVelocities(objectIndex,:) + ...
                randn(stream,1,3).*[0.25 0.25 0.15]);
            radar.PositionCovariances(:,:,index) = single(diag([0.35 0.30 0.40].^2));
            radar.VelocityCovariances(:,:,index) = single(diag([0.25 0.25 0.15].^2));
            radar.Scores(index) = single(0.90);
        end
    end

    sequence(step).Time = time;
    sequence(step).Camera = camera;
    sequence(step).Lidar = lidar;
    sequence(step).Radar = radar;
    sequence(step).TruthPositions = truthPositions;
    sequence(step).TruthVelocities = truthVelocities;
end
end

function [positions,velocities] = truthAt(time)
positions = [10+3.2*time, -1.5, 0.8; ...
    15+2.0*time, 2.6+0.55*sin(0.8*time), 0.9; ...
    23.0, -4.0+1.35*time, 0.9];
velocities = [3.2,0,0; ...
    2.0,0.44*cos(0.8*time),0; ...
    0,1.35,0];
end

function bus = emptyCamera(time,frameID)
maximum = 256;
bus = struct("Timestamp",time,"FrameID",uint32(frameID),"Count",uint16(0), ...
    "BoundingBoxes",zeros(maximum,4,"single"),"ClassIDs",zeros(maximum,1,"uint8"), ...
    "Scores",zeros(maximum,1,"single"),"Positions",zeros(maximum,3,"single"), ...
    "PositionCovariances",zeros(3,3,maximum,"single"), ...
    "ValidMask",false(maximum,1),"InputImageSize",uint16([480;640]), ...
    "InferenceTime",single(0.02),"NetworkReady",true,"Overflow",false,"Valid",true);
end

function bus = emptyLidar(time,frameID)
maximum = 256;
bus = struct("Timestamp",time,"FrameID",uint32(frameID),"Count",uint16(0), ...
    "Positions",zeros(maximum,3,"single"),"Dimensions",zeros(maximum,3,"single"), ...
    "Yaws",zeros(maximum,1,"single"),"ClassIDs",zeros(maximum,1,"uint8"), ...
    "Scores",zeros(maximum,1,"single"), ...
    "PositionCovariances",zeros(3,3,maximum,"single"), ...
    "PointCounts",zeros(maximum,1,"uint32"),"ValidMask",false(maximum,1), ...
    "ProcessingTime",single(0.01),"Overflow",false,"Valid",true);
end

function bus = emptyRadar(time,frameID)
maximum = 256;
bus = struct("Timestamp",time,"FrameID",uint32(frameID),"Count",uint16(0), ...
    "Positions",zeros(maximum,3,"single"),"Velocities",zeros(maximum,3,"single"), ...
    "PositionCovariances",zeros(3,3,maximum,"single"), ...
    "VelocityCovariances",zeros(3,3,maximum,"single"), ...
    "Scores",zeros(maximum,1,"single"),"ValidMask",false(maximum,1), ...
    "Overflow",false,"Valid",true);
end
