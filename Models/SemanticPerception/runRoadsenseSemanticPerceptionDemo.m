function result = runRoadsenseSemanticPerceptionDemo(imageInput)
%RUNROADSENSESEMANTICPERCEPTIONDEMO Run and visualize one camera frame.

arguments
    imageInput = zeros(480,640,3,"uint8")
end
if ischar(imageInput) || isstring(imageInput)
    image = imread(imageInput);
else
    image = imageInput;
end
image = imresize(image,[480 640]);
if size(image,3) == 1
    image = repmat(image,1,1,3);
end
image = im2uint8(image);

system = RoadsenseSemanticPerceptionSystem;
[labels,confidence,drivable,count,boxes,classIDs,scores,positions,covariances, ...
    validMask,inferenceTime,networkReady,overflow,valid] = system(image,true);

displayImage = image;
if count > 0
    detectionText = strings(double(count),1);
    for index = 1:double(count)
        detectionText(index) = objectText(classIDs(index),scores(index));
    end
    displayImage = insertObjectAnnotation(displayImage,"rectangle", ...
        boxes(1:double(count),:),cellstr(detectionText),LineWidth=2);
end
overlay = labeloverlay(imresize(image,[120 160]),labels,Transparency=0.45);
figure(Name="Roadsense Semantic Perception",Color="white");
tiledlayout(2,2,Padding="compact",TileSpacing="compact");
nexttile; imshow(displayImage); title("Camera-derived object regions");
nexttile; imshow(overlay); title("Roadsense semantic classes");
nexttile; imagesc(drivable,[0 1]); axis image off; colorbar; title("Drivable probability");
nexttile; imagesc(confidence,[0 1]); axis image off; colorbar; title("Winning-class confidence");

result = struct("SemanticLabel",labels,"Confidence",confidence, ...
    "DrivableProbability",drivable,"Count",count,"BoundingBoxes",boxes, ...
    "ClassIDs",classIDs,"Scores",scores,"Positions",positions, ...
    "PositionCovariances",covariances,"ValidMask",validMask, ...
    "InferenceTime",inferenceTime,"NetworkReady",networkReady, ...
    "Overflow",overflow,"Valid",valid);
end

function text = objectText(classID,score)
names = ["Unknown","Car","Truck","Bus","Auto-rickshaw","Motorcycle", ...
    "Bicycle","Pedestrian","Pushcart","Animal","Static obstacle"];
index = min(double(classID)+1,numel(names));
text = names(index) + " " + compose("%.2f",score);
end
