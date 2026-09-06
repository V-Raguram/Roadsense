classdef RoadsenseSensorFusionSystem < matlab.System
    %ROADSENSESENSORFUSIONSYSTEM Uncertainty-aware camera/LiDAR/radar tracker.
    %   The component first creates one spatial detection per physical object
    %   by gated information fusion. A JPDA tracker with an IMM initializer
    %   then performs temporal association, filtering, track confirmation,
    %   and controlled coasting through short sensor dropouts.

    properties (Nontunable)
        MaxDetections (1,1) double = 256
        MaxTracks (1,1) double = 128
        NumberClasses (1,1) double = 10
    end

    properties
        CameraLidarGate (1,1) single = single(4.0)
        RadarLidarGate (1,1) single = single(2.5)
        CameraRadarGate (1,1) single = single(4.5)
        AssignmentThreshold (2,1) single = single([12;30])
        ConfirmationProbability (1,1) single = single(0.85)
        DeletionProbability (1,1) single = single(0.005)
        DetectionProbability (1,1) single = single(0.80)
    end

    properties (Access = private)
        Tracker
        MetaTrackIDs
        MetaDimensions
        MetaYaws
        MetaClassIDs
        MetaClassObserved
        LastCameraTime
        LastLidarTime
        LastRadarTime
        LastTrackerTime
        CachedOutput
        TrackerInitialized logical = false
    end

    methods (Access = protected)
        function setupImpl(object)
            object.Tracker = trackerJPDA( ...
                FilterInitializationFcn=@initekfimm, ...
                MaxNumTracks=object.MaxTracks, ...
                MaxNumSensors=3, ...
                TrackLogic="Integrated", ...
                ConfirmationThreshold=double(object.ConfirmationProbability), ...
                DeletionThreshold=double(object.DeletionProbability), ...
                AssignmentThreshold=double(object.AssignmentThreshold(:).'), ...
                DetectionProbability=double(object.DetectionProbability), ...
                ClutterDensity=1e-5, ...
                NewTargetDensity=1e-4, ...
                ClassFusionMethod="Bayes", ...
                InitialClassProbabilities=ones(1,object.NumberClasses)/object.NumberClasses);
            object.MetaTrackIDs = zeros(object.MaxTracks,1,"uint32");
            object.MetaDimensions = zeros(object.MaxTracks,3,"single");
            object.MetaYaws = zeros(object.MaxTracks,1,"single");
            object.MetaClassIDs = zeros(object.MaxTracks,1,"uint8");
            object.MetaClassObserved = false(object.MaxTracks,1);
            object.LastCameraTime = -inf;
            object.LastLidarTime = -inf;
            object.LastRadarTime = -inf;
            object.LastTrackerTime = -inf;
            object.CachedOutput = struct;
            object.TrackerInitialized = false;
        end

        function [trackCount,trackIDs,classIDs,classConfidences,existenceProbabilities, ...
                positions,velocities,accelerations,yaws,yawRates,dimensions, ...
                stateCovariances,ages,sensorMasks,isConfirmed,validMask, ...
                processingTime,sourceOverflow,overflow,outputValid] = ...
                stepImpl(object,currentTime,camera,lidar,radar)

            startTime = tic;
            [trackIDs,classIDs,classConfidences,existenceProbabilities,positions, ...
                velocities,accelerations,yaws,yawRates,dimensions,stateCovariances, ...
                ages,sensorMasks,isConfirmed,validMask] = object.emptyOutputs();
            trackCount = uint16(0);
            sourceOverflow = logical(camera.Overflow || lidar.Overflow || radar.Overflow);
            overflow = false;
            outputValid = isfinite(currentTime) && currentTime >= 0;
            if ~outputValid
                processingTime = single(toc(startTime));
                return
            end

            % A referenced model can be evaluated at the 100 Hz parent
            % interface while CurrentTime is held at the 20 Hz fusion rate.
            % JPDA requires strictly increasing time, so repeated evaluations
            % of one fusion tick must return the prior deterministic result.
            if object.TrackerInitialized && ...
                    double(currentTime) <= object.LastTrackerTime+1e-9
                cached=object.CachedOutput;
                trackCount=cached.TrackCount; trackIDs=cached.TrackIDs;
                classIDs=cached.ClassIDs; classConfidences=cached.ClassConfidences;
                existenceProbabilities=cached.ExistenceProbabilities;
                positions=cached.Positions; velocities=cached.Velocities;
                accelerations=cached.Accelerations; yaws=cached.Yaws;
                yawRates=cached.YawRates; dimensions=cached.Dimensions;
                stateCovariances=cached.StateCovariances; ages=cached.Ages;
                sensorMasks=cached.SensorMasks; isConfirmed=cached.IsConfirmed;
                validMask=cached.ValidMask; sourceOverflow=cached.SourceOverflow;
                overflow=cached.Overflow; processingTime=single(toc(startTime));
                return
            end

            [detections,detectionMeta] = object.makeFusedDetections( ...
                double(currentTime),camera,lidar,radar);

            % trackerJPDA requires at least one detection on its first call.
            % A real system commonly starts before any sensor has produced a
            % valid frame, so emit a valid empty list until initialization.
            if isempty(detections) && ~object.TrackerInitialized
                processingTime = single(toc(startTime));
                return
            end

            [~,~,allTracks] = object.Tracker(detections,double(currentTime));
            object.TrackerInitialized = true;
            object.LastTrackerTime = double(currentTime);
            numberTracks = numel(allTracks);
            accepted = min(numberTracks,object.MaxTracks);
            overflow = numberTracks > object.MaxTracks;
            currentTrackIDs = zeros(object.MaxTracks,1,"uint32");
            detectionUsed = false(numel(detections),1);

            for index = 1:accepted
                track = allTracks(index);
                trackID = uint32(track.TrackID);
                currentTrackIDs(index) = trackID;
                trackIDs(index) = trackID;
                existenceProbabilities(index) = single(track.TrackLogicState);

                state = double(track.State(:));
                positions(index,:) = single([state(1) state(3) state(5)]);
                velocities(index,:) = single([state(2) state(4) state(6)]);
                speed = hypot(state(2),state(4));
                if speed > 0.25
                    yaws(index) = single(atan2(state(4),state(2)));
                end
                covariance = double(track.StateCovariance);
                permutation = [1 3 5 2 4 6];
                stateCovariances(:,:,index) = single(covariance(permutation,permutation));
                ages(index) = uint16(min(double(track.Age),double(intmax("uint16"))));
                isConfirmed(index) = logical(track.IsConfirmed);
                validMask(index) = true;

                metadataSlot = object.findMetadataSlot(trackID);
                [matchedDetection,distance] = object.nearestDetection( ...
                    positions(index,:),detectionMeta.Positions,detectionUsed);
                if matchedDetection > 0 && distance <= 5.0
                    detectionUsed(matchedDetection) = true;
                    object.MetaTrackIDs(metadataSlot) = trackID;
                    if all(detectionMeta.Dimensions(matchedDetection,:) > 0)
                        object.MetaDimensions(metadataSlot,:) = ...
                            detectionMeta.Dimensions(matchedDetection,:);
                    end
                    if isfinite(detectionMeta.Yaws(matchedDetection))
                        object.MetaYaws(metadataSlot) = detectionMeta.Yaws(matchedDetection);
                    end
                    if detectionMeta.ClassIDs(matchedDetection) > 0
                        object.MetaClassIDs(metadataSlot) = detectionMeta.ClassIDs(matchedDetection);
                        object.MetaClassObserved(metadataSlot) = true;
                    end
                    sensorMasks(index) = detectionMeta.SensorMasks(matchedDetection);
                end
                dimensions(index,:) = object.MetaDimensions(metadataSlot,:);
                if object.MetaClassObserved(metadataSlot)
                    classIDs(index) = uint8(track.ObjectClassID);
                    if classIDs(index) == 0
                        classIDs(index) = object.MetaClassIDs(metadataSlot);
                    end
                    probabilities = double(track.ObjectClassProbabilities);
                    if ~isempty(probabilities)
                        classConfidences(index) = single(max(probabilities));
                    end
                end
                if speed <= 0.25 && isfinite(object.MetaYaws(metadataSlot))
                    yaws(index) = object.MetaYaws(metadataSlot);
                end
            end

            object.removeStaleMetadata(currentTrackIDs(1:accepted));
            trackCount = uint16(accepted);
            object.CachedOutput=struct("TrackCount",trackCount,"TrackIDs",trackIDs, ...
                "ClassIDs",classIDs,"ClassConfidences",classConfidences, ...
                "ExistenceProbabilities",existenceProbabilities,"Positions",positions, ...
                "Velocities",velocities,"Accelerations",accelerations,"Yaws",yaws, ...
                "YawRates",yawRates,"Dimensions",dimensions, ...
                "StateCovariances",stateCovariances,"Ages",ages, ...
                "SensorMasks",sensorMasks,"IsConfirmed",isConfirmed, ...
                "ValidMask",validMask,"SourceOverflow",sourceOverflow,"Overflow",overflow);
            processingTime = single(toc(startTime));
        end

        function resetImpl(object)
            if ~isempty(object.Tracker)
                reset(object.Tracker);
            end
            object.MetaTrackIDs(:) = 0;
            object.MetaDimensions(:) = 0;
            object.MetaYaws(:) = 0;
            object.MetaClassIDs(:) = 0;
            object.MetaClassObserved(:) = false;
            object.LastCameraTime = -inf;
            object.LastLidarTime = -inf;
            object.LastRadarTime = -inf;
            object.LastTrackerTime = -inf;
            object.CachedOutput = struct;
            object.TrackerInitialized = false;
        end

        function releaseImpl(object)
            if ~isempty(object.Tracker)
                release(object.Tracker);
            end
        end

        function number = getNumInputsImpl(~)
            number = 4;
        end

        function number = getNumOutputsImpl(~)
            number = 20;
        end

        function names = getInputNamesImpl(~)
            names = ["CurrentTime","CameraObjects","LidarObjects","RadarObjects"];
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17,n18,n19,n20] = ...
                getOutputNamesImpl(~)
            n1='Count'; n2='TrackIDs'; n3='ClassIDs'; n4='ClassConfidences';
            n5='ExistenceProbabilities'; n6='Positions'; n7='Velocities';
            n8='Accelerations'; n9='Yaws'; n10='YawRates'; n11='Dimensions';
            n12='StateCovariances'; n13='Ages'; n14='SensorMasks';
            n15='IsConfirmed'; n16='ValidMask'; n17='ProcessingTime';
            n18='SourceOverflow'; n19='Overflow'; n20='OutputValid';
        end

        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18,s19,s20] = ...
                getOutputSizeImpl(object)
            one = [1 1]; vector = [object.MaxTracks 1]; matrix = [object.MaxTracks 3];
            s1=one; s2=vector; s3=vector; s4=vector; s5=vector;
            s6=matrix; s7=matrix; s8=matrix; s9=vector; s10=vector;
            s11=matrix; s12=[6 6 object.MaxTracks]; s13=vector; s14=vector;
            s15=vector; s16=vector; s17=one; s18=one; s19=one; s20=one;
        end

        function [t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18,t19,t20] = ...
                getOutputDataTypeImpl(~)
            t1='uint16'; t2='uint32'; t3='uint8'; t4='single'; t5='single';
            t6='single'; t7='single'; t8='single'; t9='single'; t10='single';
            t11='single'; t12='single'; t13='uint16'; t14='uint8';
            t15='logical'; t16='logical'; t17='single'; t18='logical';
            t19='logical'; t20='logical';
        end

        function varargout = isOutputFixedSizeImpl(object)
            varargout = repmat({true},1,getNumOutputsImpl(object));
        end

        function varargout = isOutputComplexImpl(object)
            varargout = repmat({false},1,getNumOutputsImpl(object));
        end

        function icon = getIconImpl(~)
            icon = "Roadsense\nJPDA + IMM\nSensor Fusion";
        end
    end

    methods (Access = private)
        function [detections,meta] = makeFusedDetections(object,time,camera,lidar,radar)
            cameraFresh = camera.Valid && camera.Timestamp > object.LastCameraTime && camera.Timestamp <= time;
            lidarFresh = lidar.Valid && lidar.Timestamp > object.LastLidarTime && lidar.Timestamp <= time;
            radarFresh = radar.Valid && radar.Timestamp > object.LastRadarTime && radar.Timestamp <= time;
            cameraIndices = object.validIndices(camera.Count,camera.ValidMask,cameraFresh);
            lidarIndices = object.validIndices(lidar.Count,lidar.ValidMask,lidarFresh);
            radarIndices = object.validIndices(radar.Count,radar.ValidMask,radarFresh);
            if cameraFresh; object.LastCameraTime = camera.Timestamp; end
            if lidarFresh; object.LastLidarTime = lidar.Timestamp; end
            if radarFresh; object.LastRadarTime = radar.Timestamp; end
            cameraUsed = false(numel(cameraIndices),1);
            radarUsed = false(numel(radarIndices),1);
            detections = cell(0,1);
            meta = object.emptyMetadata();

            % LiDAR anchors provide the most reliable position and extent.
            for localIndex = 1:numel(lidarIndices)
                lidarIndex = lidarIndices(localIndex);
                cameraLocal = object.nearestUnused(camera.Positions, cameraIndices, ...
                    cameraUsed,lidar.Positions(lidarIndex,:),object.CameraLidarGate);
                radarLocal = object.nearestUnused(radar.Positions, radarIndices, ...
                    radarUsed,lidar.Positions(lidarIndex,:),object.RadarLidarGate);
                if cameraLocal > 0; cameraUsed(cameraLocal) = true; end
                if radarLocal > 0; radarUsed(radarLocal) = true; end
                [detection,entry] = object.fuseOne(time,camera,lidar,radar, ...
                    object.indexOrZero(cameraIndices,cameraLocal),lidarIndex, ...
                    object.indexOrZero(radarIndices,radarLocal));
                [detections,meta] = object.appendDetection(detections,meta,detection,entry);
            end

            % Unmatched radar returns can still be paired with camera classes.
            for radarLocal = 1:numel(radarIndices)
                if radarUsed(radarLocal); continue; end
                radarIndex = radarIndices(radarLocal);
                cameraLocal = object.nearestUnused(camera.Positions,cameraIndices, ...
                    cameraUsed,radar.Positions(radarIndex,:),object.CameraRadarGate);
                if cameraLocal > 0; cameraUsed(cameraLocal) = true; end
                [detection,entry] = object.fuseOne(time,camera,lidar,radar, ...
                    object.indexOrZero(cameraIndices,cameraLocal),0,radarIndex);
                [detections,meta] = object.appendDetection(detections,meta,detection,entry);
            end

            % Camera-only ground projections are retained with their broad covariance.
            for cameraLocal = 1:numel(cameraIndices)
                if cameraUsed(cameraLocal); continue; end
                cameraIndex = cameraIndices(cameraLocal);
                [detection,entry] = object.fuseOne(time,camera,lidar,radar,cameraIndex,0,0);
                [detections,meta] = object.appendDetection(detections,meta,detection,entry);
            end
        end

        function indices = validIndices(object,count,validMask,listValid)
            if ~listValid
                indices = zeros(0,1);
                return
            end
            boundedCount = min(double(count),object.MaxDetections);
            indices = find(validMask(1:boundedCount));
        end

        function localIndex = nearestUnused(~,positions,indices,used,query,gate)
            localIndex = 0;
            if isempty(indices); return; end
            candidates = double(positions(indices,1:2));
            distances = vecnorm(candidates-double(query(1:2)),2,2);
            distances(used) = inf;
            [minimum,index] = min(distances);
            if ~isempty(minimum) && minimum <= double(gate)
                localIndex = index;
            end
        end

        function index = indexOrZero(~,indices,localIndex)
            if localIndex > 0; index = indices(localIndex); else; index = 0; end
        end

        function [detection,entry] = fuseOne(object,time,camera,lidar,radar, ...
                cameraIndex,lidarIndex,radarIndex)
            information = zeros(3,3);
            weightedPosition = zeros(3,1);
            sensorMask = uint8(0);
            combinedScore = 0;
            classID = uint8(0);
            classScore = single(0);
            dimensions = zeros(1,3,"single");
            yaw = single(0);

            if cameraIndex > 0
                covariance = object.sanitizeCovariance( ...
                    camera.PositionCovariances(:,:,cameraIndex),[4 4 2]);
                weight = covariance\eye(3);
                information = information+weight;
                weightedPosition = weightedPosition+weight*double(camera.Positions(cameraIndex,:).');
                sensorMask = bitor(sensorMask,uint8(1));
                combinedScore = max(combinedScore,double(camera.Scores(cameraIndex)));
                classID = uint8(camera.ClassIDs(cameraIndex));
                classScore = single(camera.Scores(cameraIndex));
            end
            if lidarIndex > 0
                covariance = object.sanitizeCovariance( ...
                    lidar.PositionCovariances(:,:,lidarIndex),[0.3 0.3 0.5]);
                weight = covariance\eye(3);
                information = information+weight;
                weightedPosition = weightedPosition+weight*double(lidar.Positions(lidarIndex,:).');
                sensorMask = bitor(sensorMask,uint8(2));
                combinedScore = max(combinedScore,double(lidar.Scores(lidarIndex)));
                dimensions = single(lidar.Dimensions(lidarIndex,:));
                yaw = single(lidar.Yaws(lidarIndex));
            end
            if radarIndex > 0
                covariance = object.sanitizeCovariance( ...
                    radar.PositionCovariances(:,:,radarIndex),[0.8 0.8 1.5]);
                weight = covariance\eye(3);
                information = information+weight;
                weightedPosition = weightedPosition+weight*double(radar.Positions(radarIndex,:).');
                sensorMask = bitor(sensorMask,uint8(4));
                combinedScore = max(combinedScore,double(radar.Scores(radarIndex)));
            end

            positionCovariance = information\eye(3);
            position = positionCovariance*weightedPosition;
            reliability = max(combinedScore,0.20);
            positionCovariance = positionCovariance/reliability;
            measurementParameters = struct( ...
                "Frame","rectangular", ...
                "OriginPosition",[0 0 0], ...
                "Orientation",eye(3), ...
                "HasVelocity",true, ...
                "HasElevation",true);

            if radarIndex > 0
                velocityCovariance = object.sanitizeCovariance( ...
                    radar.VelocityCovariances(:,:,radarIndex),[1.5 1.5 2.0]);
                velocity = double(radar.Velocities(radarIndex,:).');
                velocityCovariance = velocityCovariance/reliability;
            else
                % JPDA corrects detections in one association cluster as a
                % common measurement matrix. Keep a fixed 6-D layout through
                % radar dropouts. A 2 m/s standard deviation is deliberately
                % weak but bounded, preventing a newly born track from casting
                % near-uniform risk over the complete prediction grid.
                velocity = zeros(3,1);
                velocityCovariance = diag([4 4 4]);
            end
            measurement = [position;velocity];
            measurementNoise = blkdiag(positionCovariance,velocityCovariance);
            arguments = {"MeasurementNoise",measurementNoise, ...
                "SensorIndex",1, ...
                "MeasurementParameters",measurementParameters, ...
                "ObjectClassID",double(classID)};
            if classID > 0
                classParameters = struct("ConfusionMatrix", ...
                    object.classConfusionMatrix(double(classScore)));
            else
                % JPDA class fusion requires the declared matrix on every
                % detection. Class ID zero tells the tracker not to update
                % semantic evidence, so identity is neutral here.
                classParameters = struct("ConfusionMatrix",eye(object.NumberClasses));
            end
            arguments = [arguments {"ObjectClassParameters",classParameters}];
            detection = objectDetection(time,measurement,arguments{:});

            if ~all(dimensions > 0)
                dimensions = object.nominalDimensions(classID);
            end
            entry = struct("Position",single(position.'),"Dimensions",dimensions, ...
                "Yaw",yaw,"SensorMask",sensorMask,"ClassID",classID);
        end

        function covariance = sanitizeCovariance(~,raw,defaultSigma)
            covariance = double(raw);
            if any(~isfinite(covariance),"all") || any(diag(covariance) <= 0)
                covariance = diag(double(defaultSigma).^2);
            end
            covariance = (covariance+covariance.')/2 + eye(3)*1e-4;
        end

        function matrix = classConfusionMatrix(object,score)
            reliability = min(max(score,0.51),0.98);
            offDiagonal = (1-reliability)/(object.NumberClasses-1);
            matrix = ones(object.NumberClasses)*offDiagonal;
            matrix(1:object.NumberClasses+1:end) = reliability;
        end

        function dimensions = nominalDimensions(~,classID)
            table = single([4.4 1.8 1.6; 8.0 2.5 3.2; 10.5 2.6 3.2; ...
                2.8 1.4 1.8; 2.1 0.8 1.3; 1.8 0.6 1.2; 0.6 0.6 1.75; ...
                2.2 1.1 1.6; 1.8 0.7 1.4; 1.0 1.0 1.0]);
            if classID >= 1 && classID <= size(table,1)
                dimensions = table(double(classID),:);
            else
                dimensions = single([1 1 1]);
            end
        end

        function [detections,meta] = appendDetection(object,detections,meta,detection,entry)
            if numel(detections) >= object.MaxTracks; return; end
            detections{end+1,1} = detection;
            meta.Positions(end+1,:) = entry.Position;
            meta.Dimensions(end+1,:) = entry.Dimensions;
            meta.Yaws(end+1,1) = entry.Yaw;
            meta.SensorMasks(end+1,1) = entry.SensorMask;
            meta.ClassIDs(end+1,1) = entry.ClassID;
        end

        function meta = emptyMetadata(~)
            meta = struct("Positions",zeros(0,3,"single"), ...
                "Dimensions",zeros(0,3,"single"),"Yaws",zeros(0,1,"single"), ...
                "SensorMasks",zeros(0,1,"uint8"),"ClassIDs",zeros(0,1,"uint8"));
        end

        function slot = findMetadataSlot(object,trackID)
            slot = find(object.MetaTrackIDs == trackID,1);
            if isempty(slot)
                slot = find(object.MetaTrackIDs == 0,1);
            end
            if isempty(slot); slot = 1; end
        end

        function [index,distance] = nearestDetection(~,position,detectionPositions,used)
            index = 0; distance = inf;
            if isempty(detectionPositions); return; end
            distances = vecnorm(double(detectionPositions(:,1:2))-double(position(1:2)),2,2);
            distances(used) = inf;
            [distance,index] = min(distances);
            if isempty(distance) || ~isfinite(distance); index = 0; distance = inf; end
        end

        function removeStaleMetadata(object,currentTrackIDs)
            for slot = 1:object.MaxTracks
                if object.MetaTrackIDs(slot) ~= 0 && ...
                        ~any(currentTrackIDs == object.MetaTrackIDs(slot))
                    object.MetaTrackIDs(slot) = 0;
                    object.MetaDimensions(slot,:) = 0;
                    object.MetaYaws(slot) = 0;
                    object.MetaClassIDs(slot) = 0;
                    object.MetaClassObserved(slot) = false;
                end
            end
        end

        function [trackIDs,classIDs,classConfidences,existenceProbabilities, ...
                positions,velocities,accelerations,yaws,yawRates,dimensions, ...
                stateCovariances,ages,sensorMasks,isConfirmed,validMask] = emptyOutputs(object)
            vector = [object.MaxTracks 1]; matrix = [object.MaxTracks 3];
            trackIDs = zeros(vector,"uint32"); classIDs = zeros(vector,"uint8");
            classConfidences = zeros(vector,"single");
            existenceProbabilities = zeros(vector,"single");
            positions = zeros(matrix,"single"); velocities = zeros(matrix,"single");
            accelerations = zeros(matrix,"single"); yaws = zeros(vector,"single");
            yawRates = zeros(vector,"single"); dimensions = zeros(matrix,"single");
            stateCovariances = zeros(6,6,object.MaxTracks,"single");
            ages = zeros(vector,"uint16"); sensorMasks = zeros(vector,"uint8");
            isConfirmed = false(vector); validMask = false(vector);
        end
    end
end
