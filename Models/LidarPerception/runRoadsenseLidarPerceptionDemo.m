function result = runRoadsenseLidarPerceptionDemo(frame)
%RUNROADSENSELIDARPERCEPTIONDEMO Process and visualize one normalized scan.

arguments
    frame = createSyntheticRoadsenseLidarFrame()
end

perception = RoadsenseLidarPerceptionSystem;
[occupancy,groundHeight,roughness,potholeProbability,surfaceCost,observedMask, ...
    count,positions,dimensions,yaws,classIDs,scores,covariances,pointCounts, ...
    validMask,processingTime,overflow,valid] = ...
    perception(frame.Points,frame.Count,frame.Overflow,frame.Valid);

xLimits = [-20 60];
yLimits = [-25 25];
figure(Name="Roadsense LiDAR Perception",Color="white");
tiledlayout(2,2,Padding="compact",TileSpacing="compact");
nexttile;
points = frame.Points(1:double(frame.Count),:);
scatter3(points(:,1),points(:,2),points(:,3),2,points(:,3),"filled");
axis equal; xlim(xLimits); ylim(yLimits); view(2); colorbar;
xlabel("Forward (m)"); ylabel("Left (m)"); title("Normalized LiDAR scan");

nexttile;
imagesc(xLimits,yLimits,occupancy); axis xy equal tight; colorbar;
xlabel("Forward (m)"); ylabel("Left (m)"); title("Non-ground occupancy");
hold on; drawObjects(positions,dimensions,yaws,count);

nexttile;
imagesc(xLimits,yLimits,surfaceCost); axis xy equal tight; colorbar;
xlabel("Forward (m)"); ylabel("Left (m)"); title("Combined road-surface cost");

nexttile;
imagesc(xLimits,yLimits,potholeProbability); axis xy equal tight; colorbar;
xlabel("Forward (m)"); ylabel("Left (m)"); title("Pothole probability");

result = struct("Occupancy",occupancy,"GroundHeight",groundHeight, ...
    "Roughness",roughness,"PotholeProbability",potholeProbability, ...
    "SurfaceCost",surfaceCost,"ObservedMask",observedMask,"Count",count, ...
    "Positions",positions,"Dimensions",dimensions,"Yaws",yaws, ...
    "ClassIDs",classIDs,"Scores",scores,"PositionCovariances",covariances, ...
    "PointCounts",pointCounts,"ValidMask",validMask, ...
    "ProcessingTime",processingTime,"Overflow",overflow,"Valid",valid);
end

function drawObjects(positions,dimensions,yaws,count)
for index = 1:double(count)
    lengthValue = dimensions(index,1);
    widthValue = dimensions(index,2);
    corners = 0.5*[-lengthValue -widthValue; lengthValue -widthValue; ...
        lengthValue widthValue; -lengthValue widthValue; -lengthValue -widthValue];
    rotation = [cos(yaws(index)) -sin(yaws(index)); ...
        sin(yaws(index)) cos(yaws(index))];
    corners = corners*rotation' + positions(index,1:2);
    plot(corners(:,1),corners(:,2),"r-",LineWidth=1.5);
end
end
