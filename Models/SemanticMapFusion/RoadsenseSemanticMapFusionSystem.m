classdef RoadsenseSemanticMapFusionSystem < matlab.System
    %ROADSENSESEMANTICMAPFUSIONSYSTEM Fuse perception into a planner cost grid.

    properties (Nontunable)
        GridRows (1,1) double = 200
        GridCols (1,1) double = 320
        SemanticRows (1,1) double = 120
        SemanticCols (1,1) double = 160
        MaxTracks (1,1) double = 128
        MaxModes (1,1) double = 4
        PredictionSteps (1,1) double = 16
        GridResolution (1,1) single = single(0.25)
        GridXLimits (2,1) single = single([-20;60])
        GridYLimits (2,1) single = single([-25;25])
        CameraFocalLength (2,1) single = single([400;400])
        CameraPrincipalPoint (2,1) single = single([319.5;239.5])
        CameraHeight (1,1) single = single(1.4)
        CameraPitchDown (1,1) single = single(deg2rad(5))
    end

    properties
        UnknownDrivability (1,1) single = single(0.35)
        CameraMinConfidence (1,1) single = single(0.25)
        CurrentSafetyMargin (1,1) single = single(0.50)
        MaximumCurrentInflation (1,1) single = single(2.0)
        PredictionSigma (1,1) single = single(2.5)
        MaximumPredictionRadius (1,1) single = single(8.0)
        PredictionTimeDecay (1,1) single = single(3.0)
        PotholeThreshold (1,1) single = single(0.30)
    end

    properties (Access = private)
        PixelGridRows
        PixelGridCols
        PixelProjectsToGrid
    end

    methods (Access = protected)
        function setupImpl(object)
            object.precomputeCameraProjection();
        end

        function [timestamp,resolution,xLimits,yLimits,staticOccupancy, ...
                dynamicOccupancy,predictedRisk,occupancy,drivability,surfaceCost, ...
                combinedCost,semanticLabel,observedMask,processingTime, ...
                sourceOverflow,outputValid] = stepImpl(object,imageSemantic, ...
                lidarGrid,tracks,predictions)
            startTime = tic;
            rows = object.GridRows;
            cols = object.GridCols;
            resolution = object.GridResolution;
            xLimits = object.GridXLimits;
            yLimits = object.GridYLimits;
            staticOccupancy = zeros(rows,cols,"single");
            dynamicOccupancy = zeros(rows,cols,"single");
            predictedRisk = zeros(rows,cols,"single");
            occupancy = zeros(rows,cols,"single");
            drivability = repmat(object.UnknownDrivability,rows,cols);
            surfaceCost = zeros(rows,cols,"single");
            combinedCost = zeros(rows,cols,"single");
            semanticLabel = zeros(rows,cols,"uint8");
            observedMask = false(rows,cols);
            sourceOverflow = logical(tracks.SourceOverflow || tracks.Overflow || predictions.Overflow);

            timestamp = object.latestTimestamp(imageSemantic.Timestamp,lidarGrid.Timestamp, ...
                tracks.Timestamp,predictions.Timestamp);
            lidarGeometryValid = ~lidarGrid.Valid || ...
                (abs(double(lidarGrid.Resolution-resolution)) < 1e-5 && ...
                max(abs(double(lidarGrid.XLimits(:)-xLimits(:)))) < 1e-4 && ...
                max(abs(double(lidarGrid.YLimits(:)-yLimits(:)))) < 1e-4);
            outputValid = logical(isfinite(timestamp) && lidarGeometryValid && ...
                (imageSemantic.Valid || lidarGrid.Valid || tracks.Valid || predictions.Valid));
            if ~outputValid
                processingTime = single(toc(startTime));
                return
            end

            if lidarGrid.Valid
                staticOccupancy = object.clamp01(lidarGrid.Occupancy);
                surfaceCost = object.clamp01(lidarGrid.SurfaceCost);
                observedMask = lidarGrid.ObservedMask;
                lidarFree = single(0.75)*(single(1)-staticOccupancy);
                drivability(observedMask) = max(single(0.10),lidarFree(observedMask));
            end

            if imageSemantic.Valid
                [cameraDrivability,cameraLabels,cameraObserved] = ...
                    object.projectCameraEvidence(imageSemantic);
                cameraWeight = single(0.75);
                drivability(cameraObserved) = cameraWeight*cameraDrivability(cameraObserved) + ...
                    (single(1)-cameraWeight)*drivability(cameraObserved);
                semanticLabel(cameraObserved) = cameraLabels(cameraObserved);
                observedMask = observedMask | cameraObserved;
            end

            if lidarGrid.Valid
                pothole = lidarGrid.PotholeProbability >= object.PotholeThreshold;
                semanticLabel(pothole) = uint8(5);
                staticObstacle = staticOccupancy >= single(0.55);
                semanticLabel(staticObstacle) = uint8(9);
            end

            if tracks.Valid
                [dynamicOccupancy,dynamicLabels,dynamicObserved,dynamicCore] = ...
                    object.rasterizeCurrentTracks(tracks);
                % LiDAR sees both infrastructure and road users. A credible
                % tracked footprint is dynamic evidence, so remove its sensor
                % returns from the hard static layer. It remains represented
                % in current occupancy and the time-indexed prediction layers.
                staticOccupancy(dynamicCore) = single(0);
                semanticLabel(dynamicObserved) = dynamicLabels(dynamicObserved);
                observedMask = observedMask | dynamicObserved;
            end

            if predictions.Valid
                predictedRisk = object.rasterizePredictions(predictions,tracks);
            end

            occupancy = max(staticOccupancy,dynamicOccupancy);
            drivability = object.clamp01(drivability.*(single(1)-occupancy));
            nonDrivableCost = single(1)-drivability;
            combinedCost = single(1) - ...
                (single(1)-occupancy).* ...
                (single(1)-surfaceCost).* ...
                (single(1)-predictedRisk).* ...
                (single(1)-single(0.75)*nonDrivableCost);
            combinedCost = object.clamp01(combinedCost);
            processingTime = single(toc(startTime));
        end

        function number = getNumInputsImpl(~)
            number = 4;
        end

        function number = getNumOutputsImpl(~)
            number = 16;
        end

        function names = getInputNamesImpl(~)
            names = ["ImageSemantic","LidarGrid","FusedTracks","Predictions"];
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16] = ...
                getOutputNamesImpl(~)
            n1='Timestamp'; n2='Resolution'; n3='XLimits'; n4='YLimits';
            n5='StaticOccupancy'; n6='DynamicOccupancy'; n7='PredictedRisk';
            n8='Occupancy'; n9='Drivability'; n10='SurfaceCost';
            n11='CombinedCost'; n12='SemanticLabel'; n13='ObservedMask';
            n14='ProcessingTime'; n15='SourceOverflow'; n16='OutputValid';
        end

        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16] = ...
                getOutputSizeImpl(object)
            one=[1 1]; grid=[object.GridRows object.GridCols];
            s1=one; s2=one; s3=[2 1]; s4=[2 1]; s5=grid; s6=grid;
            s7=grid; s8=grid; s9=grid; s10=grid; s11=grid; s12=grid;
            s13=grid; s14=one; s15=one; s16=one;
        end

        function [t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15,t16] = ...
                getOutputDataTypeImpl(~)
            t1='double'; t2='single'; t3='single'; t4='single'; t5='single';
            t6='single'; t7='single'; t8='single'; t9='single'; t10='single';
            t11='single'; t12='uint8'; t13='logical'; t14='single';
            t15='logical'; t16='logical';
        end

        function varargout = isOutputFixedSizeImpl(object)
            varargout = repmat({true},1,getNumOutputsImpl(object));
        end

        function varargout = isOutputComplexImpl(object)
            varargout = repmat({false},1,getNumOutputsImpl(object));
        end

        function icon = getIconImpl(~)
            icon = "Roadsense\nSemantic Traversability\nMap Fusion";
        end
    end

    methods (Access = private)
        function precomputeCameraProjection(object)
            object.PixelGridRows = zeros(object.SemanticRows,object.SemanticCols,"uint16");
            object.PixelGridCols = zeros(object.SemanticRows,object.SemanticCols,"uint16");
            object.PixelProjectsToGrid = false(object.SemanticRows,object.SemanticCols);
            horizontalScale = object.SemanticCols/640;
            verticalScale = object.SemanticRows/480;
            focalX = double(object.CameraFocalLength(1))*horizontalScale;
            focalY = double(object.CameraFocalLength(2))*verticalScale;
            centreX = (double(object.CameraPrincipalPoint(1))+0.5)*horizontalScale-0.5;
            centreY = (double(object.CameraPrincipalPoint(2))+0.5)*verticalScale-0.5;
            pitch = double(object.CameraPitchDown);
            rotation = [cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
            for row = 1:object.SemanticRows
                for column = 1:object.SemanticCols
                    ray = [1;-(double(column)-0.5-centreX)/focalX; ...
                        -(double(row)-0.5-centreY)/focalY];
                    ray = rotation*ray;
                    if ray(3) >= -1e-4; continue; end
                    scale = double(object.CameraHeight)/(-ray(3));
                    x = scale*ray(1);
                    y = scale*ray(2);
                    [gridRow,gridColumn,inside] = object.worldToGrid(x,y);
                    if inside
                        object.PixelGridRows(row,column) = uint16(gridRow);
                        object.PixelGridCols(row,column) = uint16(gridColumn);
                        object.PixelProjectsToGrid(row,column) = true;
                    end
                end
            end
        end

        function [drivable,labels,observed] = projectCameraEvidence(object,semantic)
            drivableSum = zeros(object.GridRows,object.GridCols,"single");
            weightSum = zeros(object.GridRows,object.GridCols,"single");
            bestConfidence = zeros(object.GridRows,object.GridCols,"single");
            labels = zeros(object.GridRows,object.GridCols,"uint8");
            for imageRow = 1:object.SemanticRows
                for imageColumn = 1:object.SemanticCols
                    if ~object.PixelProjectsToGrid(imageRow,imageColumn); continue; end
                    confidence = semantic.Confidence(imageRow,imageColumn);
                    label = semantic.SemanticLabel(imageRow,imageColumn);
                    if confidence < object.CameraMinConfidence || label > uint8(5); continue; end
                    row = double(object.PixelGridRows(imageRow,imageColumn));
                    column = double(object.PixelGridCols(imageRow,imageColumn));
                    weight = max(confidence,single(0.05));
                    drivableSum(row,column) = drivableSum(row,column) + ...
                        weight*semantic.DrivableProbability(imageRow,imageColumn);
                    weightSum(row,column) = weightSum(row,column)+weight;
                    if confidence > bestConfidence(row,column)
                        bestConfidence(row,column) = confidence;
                        labels(row,column) = label;
                    end
                end
            end
            observed = weightSum > 0;
            drivable = zeros(object.GridRows,object.GridCols,"single");
            drivable(observed) = drivableSum(observed)./weightSum(observed);
            drivable = object.clamp01(drivable);
        end

        function [occupancy,labels,observed,coreMask] = rasterizeCurrentTracks(object,tracks)
            occupancy = zeros(object.GridRows,object.GridCols,"single");
            labels = zeros(object.GridRows,object.GridCols,"uint8");
            observed = false(object.GridRows,object.GridCols);
            coreMask = false(object.GridRows,object.GridCols);
            count = min(double(tracks.Count),object.MaxTracks);
            for index = 1:count
                if ~tracks.ValidMask(index); continue; end
                centre = double(tracks.Positions(index,1:2));
                dimensions = max(double(tracks.Dimensions(index,1:2)),[0.5 0.5]);
                covariance = double(tracks.StateCovariances(1:2,1:2,index));
                uncertainty = 0;
                if all(isfinite(covariance),"all")
                    uncertainty = min(double(object.MaximumCurrentInflation), ...
                        2*sqrt(max(max(diag(covariance)),0)));
                end
                margin = double(object.CurrentSafetyMargin)+uncertainty;
                halfLength = dimensions(1)/2+margin;
                halfWidth = dimensions(2)/2+margin;
                coreHalfLength = dimensions(1)/2+0.20;
                coreHalfWidth = dimensions(2)/2+0.20;
                radius = hypot(halfLength,halfWidth);
                yaw = double(tracks.Yaws(index));
                probability = min(max(double(tracks.ExistenceProbabilities(index)),0),1);
                semanticCode = object.objectClassToSemantic(tracks.ClassIDs(index));
                [minimumRow,maximumRow,minimumColumn,maximumColumn] = ...
                    object.gridBounds(centre,radius);
                for row = minimumRow:maximumRow
                    y = double(object.GridYLimits(1))+(row-0.5)*double(object.GridResolution);
                    for column = minimumColumn:maximumColumn
                        x = double(object.GridXLimits(1))+(column-0.5)*double(object.GridResolution);
                        deltaX = x-centre(1); deltaY = y-centre(2);
                        longitudinal = cos(yaw)*deltaX+sin(yaw)*deltaY;
                        lateral = -sin(yaw)*deltaX+cos(yaw)*deltaY;
                        if probability >= 0.35 && ...
                                abs(longitudinal) <= coreHalfLength && ...
                                abs(lateral) <= coreHalfWidth
                            coreMask(row,column) = true;
                        end
                        if abs(longitudinal) <= halfLength && abs(lateral) <= halfWidth
                            occupancy(row,column) = max(occupancy(row,column),single(probability));
                            if probability >= 0.10
                                labels(row,column) = semanticCode;
                                observed(row,column) = true;
                            end
                        end
                    end
                end
            end
        end

        function risk = rasterizePredictions(object,predictions,tracks)
            risk = zeros(object.GridRows,object.GridCols,"single");
            count = min(double(predictions.Count),object.MaxTracks);
            for objectIndex = 1:count
                if ~predictions.ValidMask(objectIndex); continue; end
                trackIndex = object.findTrack(tracks,predictions.TrackIDs(objectIndex));
                if trackIndex > 0
                    dimensions = max(double(tracks.Dimensions(trackIndex,1:2)),[0.5 0.5]);
                    existence = min(max(double(tracks.ExistenceProbabilities(trackIndex)),0),1);
                else
                    dimensions = [1 1]; existence = 0.50;
                end
                modes = min(double(predictions.NumModes(objectIndex)),object.MaxModes);
                steps = min(double(predictions.NumSteps),object.PredictionSteps);
                if steps < 2; continue; end
                for mode = 1:modes
                    modeProbability = double(predictions.ModeProbabilities(objectIndex,mode));
                    if modeProbability <= 0; continue; end
                    % The map is a spatial look-ahead corridor rather than a
                    % time-indexed collision checker. Rasterizing every other
                    % 0.2 s sample preserves corridor coverage at 0.4 s spacing
                    % while the planner still receives every prediction sample.
                    if mod(steps,2) == 0
                        stepIndices = 2:2:steps;
                    else
                        stepIndices = [2:2:steps-1 steps];
                    end
                    for step = stepIndices
                        centre = double(squeeze(predictions.Positions(objectIndex,step,mode,:))).';
                        yaw = double(predictions.Yaws(objectIndex,step,mode));
                        covariance = double(predictions.PositionCovariances(:,:,step,mode,objectIndex));
                        rotation = [cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
                        footprintCovariance = rotation*diag((dimensions/2.5).^2)*rotation.';
                        covariance = object.regularizeCovariance(covariance+footprintCovariance+ ...
                            eye(2)*double(object.CurrentSafetyMargin)^2);
                        radius = min(double(object.MaximumPredictionRadius), ...
                            double(object.PredictionSigma)*sqrt(max(max(diag(covariance)),0.1)));
                        peak = existence*modeProbability*exp( ...
                            -double(predictions.TimeOffsets(step))/double(object.PredictionTimeDecay));
                        [minimumRow,maximumRow,minimumColumn,maximumColumn] = ...
                            object.gridBounds(centre,radius);
                        determinant = covariance(1,1)*covariance(2,2)-covariance(1,2)*covariance(2,1);
                        inverse = [covariance(2,2) -covariance(1,2); ...
                            -covariance(2,1) covariance(1,1)]/determinant;
                        sigmaSquared = double(object.PredictionSigma)^2;
                        rowIndices = minimumRow:maximumRow;
                        columnIndices = minimumColumn:maximumColumn;
                        if isempty(rowIndices) || isempty(columnIndices); continue; end
                        xCoordinates = double(object.GridXLimits(1)) + ...
                            (double(columnIndices)-0.5)*double(object.GridResolution);
                        yCoordinates = double(object.GridYLimits(1)) + ...
                            (double(rowIndices)-0.5)*double(object.GridResolution);
                        [xGrid,yGrid] = meshgrid(xCoordinates,yCoordinates);
                        deltaX = xGrid-centre(1);
                        deltaY = yGrid-centre(2);
                        distanceSquared = inverse(1,1)*deltaX.^2 + ...
                            2*inverse(1,2)*deltaX.*deltaY + inverse(2,2)*deltaY.^2;
                        inside = distanceSquared <= sigmaSquared;
                        contribution = peak*exp(-0.5*distanceSquared);
                        oldRisk = double(risk(rowIndices,columnIndices));
                        oldRisk(inside) = 1-(1-oldRisk(inside)).*(1-contribution(inside));
                        risk(rowIndices,columnIndices) = single(oldRisk);
                    end
                end
            end
            risk = object.clamp01(risk);
        end

        function covariance = regularizeCovariance(~,covariance)
            if any(~isfinite(covariance),"all") || any(diag(covariance) <= 0)
                covariance = eye(2);
            end
            covariance = (covariance+covariance.')/2+eye(2)*1e-3;
            determinant = covariance(1,1)*covariance(2,2)-covariance(1,2)^2;
            if determinant <= 1e-6
                covariance = covariance+eye(2)*0.1;
            end
        end

        function index = findTrack(object,tracks,trackID)
            index = 0;
            if ~tracks.Valid; return; end
            count = min(double(tracks.Count),object.MaxTracks);
            for candidate = 1:count
                if tracks.ValidMask(candidate) && tracks.TrackIDs(candidate) == trackID
                    index = candidate;
                    return
                end
            end
        end

        function code = objectClassToSemantic(~,classID)
            if classID >= uint8(1) && classID <= uint8(5)
                code = uint8(6);
            elseif classID >= uint8(6) && classID <= uint8(8)
                code = uint8(7);
            elseif classID == uint8(9)
                code = uint8(8);
            else
                code = uint8(9);
            end
        end

        function [minimumRow,maximumRow,minimumColumn,maximumColumn] = ...
                gridBounds(object,centre,radius)
            minimumColumn = max(1,floor((centre(1)-radius-double(object.GridXLimits(1))) ...
                /double(object.GridResolution))+1);
            maximumColumn = min(object.GridCols,floor((centre(1)+radius-double(object.GridXLimits(1))) ...
                /double(object.GridResolution))+1);
            minimumRow = max(1,floor((centre(2)-radius-double(object.GridYLimits(1))) ...
                /double(object.GridResolution))+1);
            maximumRow = min(object.GridRows,floor((centre(2)+radius-double(object.GridYLimits(1))) ...
                /double(object.GridResolution))+1);
        end

        function [row,column,inside] = worldToGrid(object,x,y)
            column = floor((x-double(object.GridXLimits(1)))/double(object.GridResolution))+1;
            row = floor((y-double(object.GridYLimits(1)))/double(object.GridResolution))+1;
            inside = row >= 1 && row <= object.GridRows && ...
                column >= 1 && column <= object.GridCols;
        end

        function timestamp = latestTimestamp(~,first,second,third,fourth)
            timestamp = -inf;
            if isfinite(first); timestamp=max(timestamp,first); end
            if isfinite(second); timestamp=max(timestamp,second); end
            if isfinite(third); timestamp=max(timestamp,third); end
            if isfinite(fourth); timestamp=max(timestamp,fourth); end
        end

        function values = clamp01(~,values)
            values = min(max(single(values),single(0)),single(1));
        end
    end
end
