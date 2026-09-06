classdef RoadsenseSemanticPerceptionSystem < matlab.System
    %ROADSENSESEMANTICPERCEPTIONSYSTEM Neural semantic inference and postprocessing.
    % The baseline network is a CamVid SegNet. Class-name mapping is written
    % to also accept the Roadsense/IDD class names used by the final network.

    properties (Nontunable)
        NetworkFile (1,1) string = ""
        ExecutionEnvironment (1,1) string {mustBeMember(ExecutionEnvironment,["cpu","gpu","auto"])} = "cpu"
        InferenceMode (1,1) string {mustBeMember(InferenceMode,["network","syntheticColor"])} = "network"
        MinimumConfidence (1,1) single = single(0.45)
        MinimumBlobArea (1,1) uint32 = uint32(60)
        FocalLength (2,1) single = single([400;400])
        PrincipalPoint (2,1) single = single([319.5;239.5])
        CameraHeight (1,1) single = single(1.4)
        CameraPitchDown (1,1) single = single(deg2rad(5))
        MaximumGroundRange (1,1) single = single(80)
    end

    properties (Access = private)
        Network
        ClassNames string
        IsNetworkReady (1,1) logical = false
    end

    properties (Constant, Access = private)
        OutputHeight = 120
        OutputWidth = 160
        MaxDetections = 256
    end

    methods (Access = protected)
        function setupImpl(object)
            if object.InferenceMode=="syntheticColor"
                object.IsNetworkReady=true;
                object.ClassNames=strings(0,1);
                return
            end
            modelPath = object.resolveNetworkPath();
            assert(isfile(modelPath), ...
                "Roadsense:SemanticPerception:NetworkMissing", ...
                "Semantic network not found. Run downloadRoadsenseSemanticBaseline.");
            contents = load(modelPath);
            assert(isfield(contents,"net"), ...
                "Roadsense:SemanticPerception:NetworkVariableMissing", ...
                "Network MAT-file must contain a variable named 'net'.");
            object.Network = contents.net;
            if isfield(contents,"classNames")
                object.ClassNames = string(contents.classNames(:));
            elseif isprop(object.Network.Layers(end),"Classes")
                object.ClassNames = string(object.Network.Layers(end).Classes(:));
            else
                error("Roadsense:SemanticPerception:ClassNamesMissing", ...
                    "Network MAT-file must contain classNames for this network type.");
            end
            object.IsNetworkReady = true;
        end

        function [semanticLabel,confidence,drivableProbability,count, ...
                boxes,classIDs,scores,positions,positionCovariances,validMask, ...
                inferenceTime,networkReady,overflow,outputValid] = stepImpl(object,image,imageValid)

            semanticLabel = zeros(object.OutputHeight,object.OutputWidth,"uint8");
            confidence = zeros(object.OutputHeight,object.OutputWidth,"single");
            drivableProbability = zeros(object.OutputHeight,object.OutputWidth,"single");
            count = uint16(0);
            boxes = zeros(object.MaxDetections,4,"single");
            classIDs = zeros(object.MaxDetections,1,"uint8");
            scores = zeros(object.MaxDetections,1,"single");
            positions = zeros(object.MaxDetections,3,"single");
            positionCovariances = zeros(3,3,object.MaxDetections,"single");
            validMask = false(object.MaxDetections,1);
            inferenceTime = single(0);
            networkReady = object.IsNetworkReady;
            overflow = false;
            outputValid = false;

            if ~imageValid || ~object.IsNetworkReady
                return
            end

            startTime = tic;
            if object.InferenceMode=="syntheticColor"
                [semanticLabel,confidence,drivableProbability,count,boxes, ...
                    classIDs,scores,positions,positionCovariances,validMask,overflow]= ...
                    object.syntheticColorInference(image);
                inferenceTime=single(toc(startTime)); outputValid=true;
                return
            end
            inputSize = object.Network.Layers(1).InputSize(1:2);
            networkImage = imresize(image,inputSize,"bilinear");
            [rawLabels,winningConfidence,rawScores] = semanticseg(networkImage,object.Network, ...
                ExecutionEnvironment=object.ExecutionEnvironment);

            mappedLabels = object.mapSemanticLabels(rawLabels);
            semanticLabel = imresize(mappedLabels, ...
                [object.OutputHeight object.OutputWidth],"nearest");
            confidence = imresize(single(winningConfidence), ...
                [object.OutputHeight object.OutputWidth],"bilinear");

            roadIndex = find(strcmpi(object.ClassNames,"Road"),1);
            if isempty(roadIndex)
                roadIndex = find(contains(lower(object.ClassNames),"drivable"),1);
            end
            if ~isempty(roadIndex)
                drivableProbability = imresize(single(rawScores(:,:,roadIndex)), ...
                    [object.OutputHeight object.OutputWidth],"bilinear");
            end

            [candidateBoxes,candidateIDs,candidateScores] = ...
                object.extractObjectRegions(rawLabels,rawScores,size(image));
            totalCandidates = size(candidateBoxes,1);
            accepted = min(totalCandidates,object.MaxDetections);
            overflow = totalCandidates > object.MaxDetections;

            if accepted > 0
                boxes(1:accepted,:) = candidateBoxes(1:accepted,:);
                classIDs(1:accepted) = candidateIDs(1:accepted);
                scores(1:accepted) = candidateScores(1:accepted);
                for index = 1:accepted
                    [groundPosition,covariance,isProjectable] = ...
                        object.projectBoxToGround(boxes(index,:),classIDs(index));
                    positions(index,:) = groundPosition;
                    positionCovariances(:,:,index) = covariance;
                    validMask(index) = isProjectable;
                end
                count = uint16(accepted);
            end

            inferenceTime = single(toc(startTime));
            outputValid = true;
        end

        function resetImpl(~)
            % The baseline inference path has no temporal state.
        end

        function number = getNumInputsImpl(~)
            number = 2;
        end

        function number = getNumOutputsImpl(~)
            number = 14;
        end

        function names = getInputNamesImpl(~)
            names = ["Image","ImageValid"];
        end

        function [name1,name2,name3,name4,name5,name6,name7,name8,name9, ...
                name10,name11,name12,name13,name14] = getOutputNamesImpl(~)
            name1 = 'SemanticLabel';
            name2 = 'Confidence';
            name3 = 'DrivableProbability';
            name4 = 'Count';
            name5 = 'BoundingBoxes';
            name6 = 'ClassIDs';
            name7 = 'Scores';
            name8 = 'Positions';
            name9 = 'PositionCovariances';
            name10 = 'ValidMask';
            name11 = 'InferenceTime';
            name12 = 'NetworkReady';
            name13 = 'Overflow';
            name14 = 'OutputValid';
        end

        function [size1,size2,size3,size4,size5,size6,size7,size8,size9, ...
                size10,size11,size12,size13,size14] = getOutputSizeImpl(object)
            size1 = [object.OutputHeight object.OutputWidth];
            size2 = size1;
            size3 = size1;
            size4 = [1 1];
            size5 = [object.MaxDetections 4];
            size6 = [object.MaxDetections 1];
            size7 = size6;
            size8 = [object.MaxDetections 3];
            size9 = [3 3 object.MaxDetections];
            size10 = size6;
            size11 = [1 1];
            size12 = [1 1];
            size13 = [1 1];
            size14 = [1 1];
        end

        function [type1,type2,type3,type4,type5,type6,type7,type8,type9, ...
                type10,type11,type12,type13,type14] = getOutputDataTypeImpl(~)
            type1 = "uint8";
            type2 = "single";
            type3 = "single";
            type4 = "uint16";
            type5 = "single";
            type6 = "uint8";
            type7 = "single";
            type8 = "single";
            type9 = "single";
            type10 = "logical";
            type11 = "single";
            type12 = "logical";
            type13 = "logical";
            type14 = "logical";
        end

        function [fixed1,fixed2,fixed3,fixed4,fixed5,fixed6,fixed7,fixed8, ...
                fixed9,fixed10,fixed11,fixed12,fixed13,fixed14] = isOutputFixedSizeImpl(~)
            [fixed1,fixed2,fixed3,fixed4,fixed5,fixed6,fixed7,fixed8, ...
                fixed9,fixed10,fixed11,fixed12,fixed13,fixed14] = deal(true);
        end

        function [complex1,complex2,complex3,complex4,complex5,complex6, ...
                complex7,complex8,complex9,complex10,complex11,complex12, ...
                complex13,complex14] = isOutputComplexImpl(~)
            [complex1,complex2,complex3,complex4,complex5,complex6, ...
                complex7,complex8,complex9,complex10,complex11,complex12, ...
                complex13,complex14] = deal(false);
        end

        function icon = getIconImpl(~)
            icon = "Roadsense\nSemantic AI";
        end
    end

    methods (Access = private)
        function [labels,confidence,drivable,count,boxes,classIDs,scores, ...
                positions,covariances,validMask,overflow]=syntheticColorInference(object,image)
            labels=zeros(object.OutputHeight,object.OutputWidth,"uint8");
            confidence=single(0.70)*ones(object.OutputHeight,object.OutputWidth,"single");
            drivable=zeros(object.OutputHeight,object.OutputWidth,"single");
            red=single(image(:,:,1)); green=single(image(:,:,2)); blue=single(image(:,:,3));
            road=red>=75 & red<=86 & green>=73 & green<=84 & blue>=69 & blue<=80;
            pothole=red>=34 & red<=44 & green>=39 & green<=49 & blue>=44 & blue<=54;
            roadSmall=imresize(road,[object.OutputHeight object.OutputWidth],"nearest");
            potholeSmall=imresize(pothole,[object.OutputHeight object.OutputWidth],"nearest");
            labels(roadSmall)=uint8(1); labels(potholeSmall)=uint8(5);
            confidence(roadSmall)=single(0.95); confidence(potholeSmall)=single(0.98);
            drivable(roadSmall)=single(0.95); drivable(potholeSmall)=single(0.05);

            boxes=zeros(object.MaxDetections,4,"single");
            classIDs=zeros(object.MaxDetections,1,"uint8");
            scores=zeros(object.MaxDetections,1,"single");
            positions=zeros(object.MaxDetections,3,"single");
            covariances=zeros(3,3,object.MaxDetections,"single");
            validMask=false(object.MaxDetections,1); total=0;
            palette=uint8([40 100 220;145 75 35;225 70 40;245 155 20; ...
                135 55 185;25 165 140;30 205 65;190 115 25;150 95 45]);
            semanticCodes=uint8([6 6 6 6 6 7 7 7 8]);
            for classIndex=1:size(palette,1)
                color=single(palette(classIndex,:));
                mask=abs(red-color(1))<=2 & abs(green-color(2))<=2 & abs(blue-color(3))<=2;
                small=imresize(mask,[object.OutputHeight object.OutputWidth],"nearest");
                labels(small)=semanticCodes(classIndex); confidence(small)=single(0.98);
                components=bwconncomp(mask,8);
                regions=regionprops(components,"Area","BoundingBox");
                for regionIndex=1:numel(regions)
                    if regions(regionIndex).Area<12; continue; end
                    total=total+1; if total>object.MaxDetections; continue; end
                    boxes(total,:)=single(regions(regionIndex).BoundingBox);
                    classIDs(total)=uint8(classIndex); scores(total)=single(0.98);
                    [positions(total,:),covariances(:,:,total),validMask(total)]= ...
                        object.projectBoxToGround(boxes(total,:),classIDs(total));
                end
            end
            accepted=min(total,object.MaxDetections); count=uint16(accepted);
            overflow=total>object.MaxDetections;
        end

        function modelPath = resolveNetworkPath(object)
            if strlength(object.NetworkFile) > 0
                modelPath = object.NetworkFile;
                return
            end
            componentDir = fileparts(mfilename("fullpath"));
            root = fileparts(fileparts(componentDir));
            modelPath = fullfile(root,"Data","Networks","segnetVGG16CamVid.mat");
        end

        function mapped = mapSemanticLabels(object,labels)
            mapped = zeros(size(labels),"uint8");
            for index = 1:numel(object.ClassNames)
                className = object.ClassNames(index);
                code = semanticCode(className);
                if code > 0
                    mapped(labels == categorical(className)) = code;
                end
            end
        end

        function [boxes,classIDs,scores] = extractObjectRegions(object,labels,scoreTensor,originalImageSize)
            boxes = zeros(0,4,"single");
            classIDs = zeros(0,1,"uint8");
            scores = zeros(0,1,"single");
            scaleX = single(originalImageSize(2)/size(labels,2));
            scaleY = single(originalImageSize(1)/size(labels,1));

            for classIndex = 1:numel(object.ClassNames)
                objectID = objectClassCode(object.ClassNames(classIndex));
                if objectID == 0
                    continue
                end
                mask = labels == categorical(object.ClassNames(classIndex));
                components = bwconncomp(mask,8);
                regions = regionprops(components,"Area","BoundingBox","PixelIdxList");
                classScore = scoreTensor(:,:,classIndex);
                for regionIndex = 1:numel(regions)
                    regionScore = single(mean(classScore(regions(regionIndex).PixelIdxList),"all"));
                    if regions(regionIndex).Area < double(object.MinimumBlobArea) ...
                            || regionScore < object.MinimumConfidence
                        continue
                    end
                    box = single(regions(regionIndex).BoundingBox);
                    box([1 3]) = box([1 3])*scaleX;
                    box([2 4]) = box([2 4])*scaleY;
                    boxes(end+1,:) = box; %#ok<AGROW>
                    classIDs(end+1,1) = objectID; %#ok<AGROW>
                    scores(end+1,1) = regionScore; %#ok<AGROW>
                end
            end

            if ~isempty(scores)
                [scores,order] = sort(scores,"descend");
                boxes = boxes(order,:);
                classIDs = classIDs(order);
            end
        end

        function [position,covariance,valid] = projectBoxToGround(object,box,classID)
            bottomU = box(1) + box(3)/2;
            bottomV = box(2) + box(4);
            rayRight = (bottomU-object.PrincipalPoint(1))/object.FocalLength(1);
            rayDown = (bottomV-object.PrincipalPoint(2))/object.FocalLength(2);
            pitch = object.CameraPitchDown;
            downComponent = sin(pitch) + cos(pitch)*rayDown;
            forwardComponent = cos(pitch) - sin(pitch)*rayDown;

            valid = downComponent > single(0.02);
            if valid
                scale = object.CameraHeight/downComponent;
                forward = scale*forwardComponent;
                left = -scale*rayRight;
                valid = forward > 0 && forward <= object.MaximumGroundRange;
            else
                forward = single(0);
                left = single(0);
            end

            position = single([forward left 0]);
            rangeSigma = single(0.5 + 0.04*max(forward,0));
            lateralSigma = single(0.25 + 0.02*max(forward,0));
            heightSigma = nominalHeightSigma(classID);
            covariance = zeros(3,3,"single");
            covariance(1,1) = rangeSigma^2;
            covariance(2,2) = lateralSigma^2;
            covariance(3,3) = heightSigma^2;
        end
    end
end

function code = semanticCode(className)
name = normalized(className);
if any(name == ["road","drivablesurface","roadfallback","drivablefallback"])
    code = uint8(1);
elseif any(name == ["nondrivable","terrain"])
    code = uint8(2);
elseif any(name == ["roadedge","curb","curbmedian"])
    code = uint8(3);
elseif any(name == ["pavement","sidewalk","footpath","parking"])
    code = uint8(4);
elseif contains(name,"pothole")
    code = uint8(5);
elseif objectClassCode(className) > 0 && objectClassCode(className) < 6
    code = uint8(6);
elseif any(objectClassCode(className) == uint8([6 7 8]))
    code = uint8(7);
elseif objectClassCode(className) == 9
    code = uint8(8);
elseif any(name == ["building","pole","tree","signsymbol","trafficsign","fence","staticobstacle"])
    code = uint8(9);
else
    code = uint8(0);
end
end

function code = objectClassCode(className)
name = normalized(className);
if name == "car"
    code = uint8(1);
elseif any(name == ["truck","caravan","trailer"])
    code = uint8(2);
elseif name == "bus"
    code = uint8(3);
elseif any(name == ["autorickshaw","autorickshawvehicle"])
    code = uint8(4);
elseif any(name == ["motorcycle","motorbike","rider"])
    code = uint8(5);
elseif any(name == ["bicycle","bicyclist"])
    code = uint8(6);
elseif any(name == ["pedestrian","person"])
    code = uint8(7);
elseif any(name == ["pushcart","cart"])
    code = uint8(8);
elseif any(name == ["animal","cow","cattle","dog"])
    code = uint8(9);
elseif name == "staticobstacle"
    code = uint8(10);
else
    code = uint8(0);
end
end

function name = normalized(className)
name = lower(regexprep(string(className),"[^a-zA-Z0-9]",""));
end

function sigma = nominalHeightSigma(classID)
if classID == 7 || classID == 9
    sigma = single(0.35);
else
    sigma = single(0.5);
end
end
