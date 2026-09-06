function frame = createSyntheticRoadsenseLidarFrame()
%CREATESYNTHETICROADSENSELIDARFRAME Uneven road, pothole, car, and pedestrian.

maxPoints = 120000;
[x,y] = meshgrid(single(0:0.25:45),single(-10:0.25:10));
z = single(-1.5 + 0.004*x + 0.015*sin(y/3));
pothole = x >= 13 & x <= 15 & abs(y) <= 1.25;
z(pothole) = z(pothole)-single(0.16*(1-(abs(y(pothole))/1.5).^2));
ground = [x(:) y(:) z(:)];

rng(7,"twister");
car = boxReturns(single([16 2 -0.75]),single([4.2 1.8 1.5]),900);
pedestrian = boxReturns(single([9 -2 -0.65]),single([0.55 0.55 1.7]),220);
cart = boxReturns(single([25 -3 -0.95]),single([1.8 0.9 1.1]),350);
points = [ground;car;pedestrian;cart];
points = points + single(0.008*randn(size(points)));

count = min(size(points,1),maxPoints);
fixedPoints = zeros(maxPoints,3,"single");
fixedIntensity = zeros(maxPoints,1,"single");
fixedPoints(1:count,:) = points(1:count,:);
fixedIntensity(1:count) = single(0.35 + 0.6*rand(count,1));

frame = struct( ...
    "Timestamp",0, ...
    "FrameID",uint32(1), ...
    "Count",uint32(count), ...
    "Points",fixedPoints, ...
    "Intensity",fixedIntensity, ...
    "Overflow",size(points,1)>maxPoints, ...
    "Valid",true);
end

function points = boxReturns(centre,dimensions,count)
half = dimensions/2;
points = zeros(count,3,"single");
for index = 1:count
    face = randi(5);
    coordinate = single(2*rand(1,3)-1).*half;
    if face == 1
        coordinate(1) = -half(1);
    elseif face == 2
        coordinate(1) = half(1);
    elseif face == 3
        coordinate(2) = -half(2);
    elseif face == 4
        coordinate(2) = half(2);
    else
        coordinate(3) = half(3);
    end
    points(index,:) = centre+coordinate;
end
end

