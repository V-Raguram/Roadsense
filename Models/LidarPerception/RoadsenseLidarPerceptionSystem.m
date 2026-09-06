classdef RoadsenseLidarPerceptionSystem < matlab.System
    %ROADSENSELIDARPERCEPTIONSYSTEM Segment ground, surface, and obstacles.

    properties (Nontunable)
        GridResolution (1,1) single = single(0.25)
        GridXLimits (2,1) single = single([-20;60])
        GridYLimits (2,1) single = single([-25;25])
        ZLimits (2,1) single = single([-3;5])
        SMRFResolution (1,1) single = single(0.5)
        SMRFSlopeThreshold (1,1) single = single(0.20)
        SMRFElevationThreshold (1,1) single = single(0.30)
        ClusterDistance (1,1) single = single(0.75)
        MinimumClusterPoints (1,1) uint16 = uint16(8)
        MaximumClusterPoints (1,1) uint32 = uint32(20000)
        PotholeDepthThreshold (1,1) single = single(0.08)
        RoughnessThreshold (1,1) single = single(0.05)
    end

    properties (Constant, Access = private)
        GridRows = 200
        GridCols = 320
        MaxObjects = 256
        MaxInputPoints = 120000
    end

    methods (Access = protected)
        function [occupancy,groundHeight,roughness,potholeProbability,surfaceCost, ...
                observedMask,objectCount,positions,dimensions,yaws,classIDs,scores, ...
                positionCovariances,pointCounts,objectValidMask,processingTime, ...
                overflow,outputValid] = stepImpl(object,points,count,sourceOverflow,scanValid)

            occupancy = zeros(object.GridRows,object.GridCols,"single");
            groundHeight = zeros(object.GridRows,object.GridCols,"single");
            roughness = zeros(object.GridRows,object.GridCols,"single");
            potholeProbability = zeros(object.GridRows,object.GridCols,"single");
            surfaceCost = zeros(object.GridRows,object.GridCols,"single");
            observedMask = false(object.GridRows,object.GridCols);
            objectCount = uint16(0);
            positions = zeros(object.MaxObjects,3,"single");
            dimensions = zeros(object.MaxObjects,3,"single");
            yaws = zeros(object.MaxObjects,1,"single");
            classIDs = zeros(object.MaxObjects,1,"uint8");
            scores = zeros(object.MaxObjects,1,"single");
            positionCovariances = zeros(3,3,object.MaxObjects,"single");
            pointCounts = zeros(object.MaxObjects,1,"uint32");
            objectValidMask = false(object.MaxObjects,1);
            processingTime = single(0);
            overflow = sourceOverflow;
            outputValid = false;

            usableCount = min(double(count),object.MaxInputPoints);
            if ~scanValid || usableCount < 3
                return
            end

            startTime = tic;
            cloud = points(1:usableCount,:);
            finiteMask = all(isfinite(cloud),2);
            roiMask = finiteMask ...
                & cloud(:,1) >= object.GridXLimits(1) ...
                & cloud(:,1) < object.GridXLimits(2) ...
                & cloud(:,2) >= object.GridYLimits(1) ...
                & cloud(:,2) < object.GridYLimits(2) ...
                & cloud(:,3) >= object.ZLimits(1) ...
                & cloud(:,3) <= object.ZLimits(2);
            cloud = cloud(roiMask,:);
            if size(cloud,1) < 3
                processingTime = single(toc(startTime));
                outputValid = true;
                return
            end

            pointCloudObject = pointCloud(cloud);
            if size(cloud,1) >= 30
                groundMask = segmentGroundSMRF(pointCloudObject, ...
                    double(object.SMRFResolution), ...
                    MaxWindowRadius=8, ...
                    SlopeThreshold=double(object.SMRFSlopeThreshold), ...
                    ElevationThreshold=double(object.SMRFElevationThreshold), ...
                    ElevationScale=1.0);
            else
                heightCutoff = prctile(cloud(:,3),30) + object.SMRFElevationThreshold;
                groundMask = cloud(:,3) <= heightCutoff;
            end

            [occupancy,groundHeight,roughness,potholeProbability,surfaceCost,observedMask] = ...
                object.buildGridEvidence(cloud,groundMask);
            [candidatePositions,candidateDimensions,candidateYaws,candidateScores, ...
                candidateCovariances,candidatePointCounts] = ...
                object.clusterObstacles(cloud(~groundMask,:));

            numberCandidates = size(candidatePositions,1);
            accepted = min(numberCandidates,object.MaxObjects);
            overflow = sourceOverflow || numberCandidates > object.MaxObjects;
            if accepted > 0
                positions(1:accepted,:) = candidatePositions(1:accepted,:);
                dimensions(1:accepted,:) = candidateDimensions(1:accepted,:);
                yaws(1:accepted) = candidateYaws(1:accepted);
                scores(1:accepted) = candidateScores(1:accepted);
                positionCovariances(:,:,1:accepted) = candidateCovariances(:,:,1:accepted);
                pointCounts(1:accepted) = candidatePointCounts(1:accepted);
                objectValidMask(1:accepted) = true;
                objectCount = uint16(accepted);
            end

            processingTime = single(toc(startTime));
            outputValid = true;
        end

        function resetImpl(~)
            % This perception stage has no temporal state.
        end

        function number = getNumInputsImpl(~)
            number = 4;
        end

        function number = getNumOutputsImpl(~)
            number = 18;
        end

        function names = getInputNamesImpl(~)
            names = ["Points","Count","SourceOverflow","ScanValid"];
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18] = ...
                getOutputNamesImpl(~)
            n1='Occupancy'; n2='GroundHeight'; n3='Roughness';
            n4='PotholeProbability'; n5='SurfaceCost'; n6='ObservedMask';
            n7='ObjectCount'; n8='Positions'; n9='Dimensions'; n10='Yaws';
            n11='ClassIDs'; n12='Scores'; n13='PositionCovariances';
            n14='PointCounts'; n15='ObjectValidMask'; n16='ProcessingTime';
            n17='Overflow'; n18='OutputValid';
        end

        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18] = ...
                getOutputSizeImpl(object)
            gridSize = [object.GridRows object.GridCols];
            s1=gridSize; s2=gridSize; s3=gridSize; s4=gridSize;
            s5=gridSize; s6=gridSize; s7=[1 1];
            s8=[object.MaxObjects 3]; s9=[object.MaxObjects 3];
            s10=[object.MaxObjects 1]; s11=s10; s12=s10;
            s13=[3 3 object.MaxObjects]; s14=s10; s15=s10;
            s16=[1 1]; s17=[1 1]; s18=[1 1];
        end

        function [t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18] = ...
                getOutputDataTypeImpl(~)
            t1='single'; t2='single'; t3='single'; t4='single'; t5='single';
            t6='logical'; t7='uint16'; t8='single'; t9='single'; t10='single';
            t11='uint8'; t12='single'; t13='single'; t14='uint32';
            t15='logical'; t16='single'; t17='logical'; t18='logical';
        end

        function [f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,f16,f17,f18] = ...
                isOutputFixedSizeImpl(~)
            [f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,f16,f17,f18] = deal(true);
        end

        function [c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18] = ...
                isOutputComplexImpl(~)
            [c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18] = deal(false);
        end

        function icon = getIconImpl(~)
            icon = "Roadsense\nLiDAR Perception";
        end
    end

    methods (Access = private)
        function [occupancy,groundHeight,roughness,potholeProbability,surfaceCost,observedMask] = ...
                buildGridEvidence(object,cloud,groundMask)
            rows = object.GridRows;
            cols = object.GridCols;
            cellCount = rows*cols;
            column = floor((cloud(:,1)-object.GridXLimits(1))/object.GridResolution) + 1;
            row = floor((cloud(:,2)-object.GridYLimits(1))/object.GridResolution) + 1;
            column = min(max(column,1),cols);
            row = min(max(row,1),rows);
            linearIndex = sub2ind([rows cols],row,column);

            observedCounts = accumarray(linearIndex,1,[cellCount 1],@sum,0);
            observedMask = reshape(observedCounts > 0,[rows cols]);

            nonGroundIndex = linearIndex(~groundMask);
            nonGroundCounts = accumarray(nonGroundIndex,1,[cellCount 1],@sum,0);
            occupancy = reshape(single(1-exp(-nonGroundCounts/3)),[rows cols]);

            groundPoints = cloud(groundMask,:);
            groundIndex = linearIndex(groundMask);
            groundHeightVector = accumarray(groundIndex,double(groundPoints(:,3)), ...
                [cellCount 1],@mean,0);
            roughnessVector = accumarray(groundIndex,double(groundPoints(:,3)), ...
                [cellCount 1],@(values) std(values,0),0);
            groundHeight = reshape(single(groundHeightVector),[rows cols]);
            roughness = reshape(single(roughnessVector),[rows cols]);

            depressionVector = zeros(cellCount,1);
            if size(groundPoints,1) >= 3
                design = [double(groundPoints(:,1:2)) ones(size(groundPoints,1),1)];
                coefficients = design\double(groundPoints(:,3));
                referenceHeight = design*coefficients;
                depression = max(referenceHeight-double(groundPoints(:,3)),0);
                depressionVector = accumarray(groundIndex,depression, ...
                    [cellCount 1],@mean,0);
            end
            normalizedDepression = max( ...
                (depressionVector-double(object.PotholeDepthThreshold)) ...
                /(2*double(object.PotholeDepthThreshold)),0);
            potholeProbability = reshape(single(min(normalizedDepression,1)),[rows cols]);
            roughnessCost = min(roughness/max(object.RoughnessThreshold,eps("single")),1);
            surfaceCost = max(potholeProbability,roughnessCost);
        end

        function [positions,dimensions,yaws,scores,covariances,pointCounts] = ...
                clusterObstacles(object,nonGroundPoints)
            positions = zeros(0,3,"single");
            dimensions = zeros(0,3,"single");
            yaws = zeros(0,1,"single");
            scores = zeros(0,1,"single");
            covariances = zeros(3,3,0,"single");
            pointCounts = zeros(0,1,"uint32");
            if size(nonGroundPoints,1) < double(object.MinimumClusterPoints)
                return
            end

            cloud = pointCloud(nonGroundPoints);
            [labels,numberClusters] = pcsegdist(cloud,double(object.ClusterDistance), ...
                NumClusterPoints=[double(object.MinimumClusterPoints) double(object.MaximumClusterPoints)], ...
                Method="approximate",ParallelNeighborSearch=false);

            for clusterIndex = 1:numberClusters
                cluster = nonGroundPoints(labels == clusterIndex,:);
                numberPoints = size(cluster,1);
                if numberPoints < double(object.MinimumClusterPoints)
                    continue
                end
                centre = mean(cluster,1);
                xy = double(cluster(:,1:2)-centre(1:2));
                if numberPoints >= 3 && any(std(xy,0,1) > 1e-4)
                    [vectors,values] = eig(cov(xy,1));
                    [~,principalIndex] = max(diag(values));
                    principal = vectors(:,principalIndex);
                    lateral = [-principal(2);principal(1)];
                    localCoordinates = xy*[principal lateral];
                    planarDimensions = max(localCoordinates,[],1)-min(localCoordinates,[],1);
                    yaw = atan2(principal(2),principal(1));
                else
                    planarDimensions = max(xy,[],1)-min(xy,[],1);
                    yaw = 0;
                end
                height = max(cluster(:,3))-min(cluster(:,3));
                clusterDimensions = single([max(planarDimensions) min(planarDimensions) height]);

                if clusterDimensions(1) < 0.15 || clusterDimensions(1) > 15 ...
                        || clusterDimensions(2) < 0.05 || clusterDimensions(2) > 6 ...
                        || clusterDimensions(3) < 0.10 || clusterDimensions(3) > 5
                    continue
                end

                spread = std(double(cluster),0,1)/sqrt(numberPoints);
                sigma = max(spread,[0.05 0.05 0.08]);
                covariance = diag(single(sigma.^2));
                confidence = single(min(1,1-exp(-numberPoints/20)));

                positions(end+1,:) = single(centre); %#ok<AGROW>
                dimensions(end+1,:) = clusterDimensions; %#ok<AGROW>
                yaws(end+1,1) = single(wrapToPi(yaw)); %#ok<AGROW>
                scores(end+1,1) = confidence; %#ok<AGROW>
                covariances(:,:,end+1) = covariance; %#ok<AGROW>
                pointCounts(end+1,1) = uint32(numberPoints); %#ok<AGROW>
            end

            if ~isempty(positions)
                [~,order] = sort(vecnorm(positions(:,1:2),2,2),"ascend");
                positions = positions(order,:);
                dimensions = dimensions(order,:);
                yaws = yaws(order);
                scores = scores(order);
                covariances = covariances(:,:,order);
                pointCounts = pointCounts(order);
            end
        end
    end
end
