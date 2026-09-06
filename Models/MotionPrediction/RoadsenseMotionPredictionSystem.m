classdef RoadsenseMotionPredictionSystem < matlab.System
    %ROADSENSEMOTIONPREDICTIONSYSTEM Class-aware multimodal trajectory predictor.
    %   Track history estimates acceleration and yaw-rate trends. Four
    %   bounded kinematic modes represent continuation, stopping, leftward
    %   deviation, and rightward deviation, with class-dependent priors and
    %   uncertainty growth. Static obstacles use one stationary mode.

    properties (Nontunable)
        MaxTracks (1,1) double = 128
        MaxModes (1,1) double = 4
        PredictionSteps (1,1) double = 16
    end

    properties
        PredictionStep (1,1) single = single(0.2)
        HistoryGain (1,1) single = single(0.35)
        VehicleTurnRate (1,1) single = single(0.25)
        VulnerableTurnRate (1,1) single = single(0.65)
        VehicleBrake (1,1) single = single(-3.0)
        VulnerableBrake (1,1) single = single(-1.5)
        MinimumModeProbability (1,1) single = single(0.03)
    end

    properties (Access = private)
        HistoryTrackIDs
        PreviousVelocities
        PreviousYaws
        PreviousTimes
        FilteredAccelerations
        FilteredYawRates
    end

    methods (Access = protected)
        function setupImpl(object)
            object.HistoryTrackIDs = zeros(object.MaxTracks,1,"uint32");
            object.PreviousVelocities = zeros(object.MaxTracks,3,"single");
            object.PreviousYaws = zeros(object.MaxTracks,1,"single");
            object.PreviousTimes = -inf(object.MaxTracks,1);
            object.FilteredAccelerations = zeros(object.MaxTracks,1,"single");
            object.FilteredYawRates = zeros(object.MaxTracks,1,"single");
        end

        function [count,trackIDs,classIDs,numModes,numSteps,timeOffsets, ...
                modeProbabilities,positions,yaws,positionCovariances,validMask, ...
                processingTime,overflow,outputValid] = stepImpl(object,tracks)
            startTime = tic;
            [trackIDs,classIDs,numModes,modeProbabilities,positions,yaws, ...
                positionCovariances,validMask] = object.emptyOutputs();
            timeOffsets = single((0:object.PredictionSteps-1).')*object.PredictionStep;
            numSteps = uint8(object.PredictionSteps);
            count = uint16(0);
            overflow = logical(tracks.Overflow || double(tracks.Count) > object.MaxTracks);
            outputValid = logical(tracks.Valid && isfinite(tracks.Timestamp));
            if ~outputValid
                processingTime = single(toc(startTime));
                return
            end

            accepted = min(double(tracks.Count),object.MaxTracks);
            activeTrackIDs = zeros(object.MaxTracks,1,"uint32");
            outputIndex = 0;
            for inputIndex = 1:accepted
                if ~tracks.ValidMask(inputIndex); continue; end
                outputIndex = outputIndex+1;
                trackID = uint32(tracks.TrackIDs(inputIndex));
                classID = uint8(tracks.ClassIDs(inputIndex));
                activeTrackIDs(outputIndex) = trackID;
                historySlot = object.updateHistory(tracks,inputIndex);

                trackIDs(outputIndex) = trackID;
                classIDs(outputIndex) = classID;
                if classID == uint8(10)
                    numberModes = 1;
                else
                    numberModes = object.MaxModes;
                end
                numModes(outputIndex) = uint8(numberModes);
                probabilities = object.calculateModeProbabilities(classID, ...
                    object.FilteredAccelerations(historySlot), ...
                    object.FilteredYawRates(historySlot),numberModes);
                modeProbabilities(outputIndex,1:numberModes) = probabilities;

                [objectPositions,objectYaws,objectCovariances] = object.predictOne( ...
                    tracks,inputIndex,classID,numberModes, ...
                    object.FilteredAccelerations(historySlot), ...
                    object.FilteredYawRates(historySlot),timeOffsets);
                positions(outputIndex,:,1:numberModes,:) = objectPositions;
                yaws(outputIndex,:,1:numberModes) = objectYaws;
                positionCovariances(:,:,:,1:numberModes,outputIndex) = objectCovariances;
                validMask(outputIndex) = true;
            end

            object.removeStaleHistory(activeTrackIDs(1:outputIndex));
            count = uint16(outputIndex);
            processingTime = single(toc(startTime));
        end

        function resetImpl(object)
            object.HistoryTrackIDs(:) = 0;
            object.PreviousVelocities(:) = 0;
            object.PreviousYaws(:) = 0;
            object.PreviousTimes(:) = -inf;
            object.FilteredAccelerations(:) = 0;
            object.FilteredYawRates(:) = 0;
        end

        function number = getNumInputsImpl(~)
            number = 1;
        end

        function number = getNumOutputsImpl(~)
            number = 14;
        end

        function name = getInputNamesImpl(~)
            name = "FusedTracks";
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14] = ...
                getOutputNamesImpl(~)
            n1='Count'; n2='TrackIDs'; n3='ClassIDs'; n4='NumModes';
            n5='NumSteps'; n6='TimeOffsets'; n7='ModeProbabilities';
            n8='Positions'; n9='Yaws'; n10='PositionCovariances';
            n11='ValidMask'; n12='ProcessingTime'; n13='Overflow';
            n14='OutputValid';
        end

        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14] = ...
                getOutputSizeImpl(object)
            one=[1 1]; vector=[object.MaxTracks 1];
            s1=one; s2=vector; s3=vector; s4=vector; s5=one;
            s6=[object.PredictionSteps 1];
            s7=[object.MaxTracks object.MaxModes];
            s8=[object.MaxTracks object.PredictionSteps object.MaxModes 2];
            s9=[object.MaxTracks object.PredictionSteps object.MaxModes];
            s10=[2 2 object.PredictionSteps object.MaxModes object.MaxTracks];
            s11=vector; s12=one; s13=one; s14=one;
        end

        function [t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14] = ...
                getOutputDataTypeImpl(~)
            t1='uint16'; t2='uint32'; t3='uint8'; t4='uint8'; t5='uint8';
            t6='single'; t7='single'; t8='single'; t9='single'; t10='single';
            t11='logical'; t12='single'; t13='logical'; t14='logical';
        end

        function varargout = isOutputFixedSizeImpl(object)
            varargout = repmat({true},1,getNumOutputsImpl(object));
        end

        function varargout = isOutputComplexImpl(object)
            varargout = repmat({false},1,getNumOutputsImpl(object));
        end

        function icon = getIconImpl(~)
            icon = "Roadsense\nClass-aware Multimodal\nMotion Prediction";
        end
    end

    methods (Access = private)
        function slot = updateHistory(object,tracks,index)
            trackID = uint32(tracks.TrackIDs(index));
            slot = find(object.HistoryTrackIDs == trackID,1);
            if isempty(slot)
                slot = find(object.HistoryTrackIDs == 0,1);
                if isempty(slot); slot = 1; end
                object.HistoryTrackIDs(slot) = trackID;
                object.PreviousVelocities(slot,:) = tracks.Velocities(index,:);
                object.PreviousYaws(slot) = tracks.Yaws(index);
                object.PreviousTimes(slot) = tracks.Timestamp;
                return
            end

            deltaTime = tracks.Timestamp-object.PreviousTimes(slot);
            hasFreshSupport = tracks.SensorMasks(index) > 0;
            if deltaTime > 1e-4 && hasFreshSupport
                previousVelocity = double(object.PreviousVelocities(slot,1:2));
                currentVelocity = double(tracks.Velocities(index,1:2));
                previousSpeed = norm(previousVelocity);
                currentSpeed = norm(currentVelocity);
                measuredAcceleration = (currentSpeed-previousSpeed)/deltaTime;
                measuredAcceleration = min(max(measuredAcceleration,-5),4);
                measuredYawRate = object.angleDifference( ...
                    double(tracks.Yaws(index)),double(object.PreviousYaws(slot)))/deltaTime;
                measuredYawRate = min(max(measuredYawRate,-1.5),1.5);
                gain = double(object.HistoryGain);
                object.FilteredAccelerations(slot) = single((1-gain)* ...
                    double(object.FilteredAccelerations(slot))+gain*measuredAcceleration);
                object.FilteredYawRates(slot) = single((1-gain)* ...
                    double(object.FilteredYawRates(slot))+gain*measuredYawRate);
                object.PreviousVelocities(slot,:) = tracks.Velocities(index,:);
                object.PreviousYaws(slot) = tracks.Yaws(index);
                object.PreviousTimes(slot) = tracks.Timestamp;
            end
        end

        function probabilities = calculateModeProbabilities(object,classID,acceleration,yawRate,numberModes)
            probabilities = zeros(1,numberModes,"single");
            if numberModes == 1
                probabilities(1) = 1;
                return
            end
            if any(classID == uint8([6 7 8 9]))
                raw = [0.40 0.20 0.20 0.20];
            elseif classID == 0
                raw = [0.35 0.20 0.225 0.225];
            else
                raw = [0.55 0.20 0.125 0.125];
            end
            brakeEvidence = min(max(-double(acceleration)/3,0),1);
            turnEvidence = min(abs(double(yawRate))/0.6,1);
            raw(2) = raw(2)+0.25*brakeEvidence;
            raw(1) = max(raw(1)-0.15*brakeEvidence-0.10*turnEvidence,0);
            if yawRate >= 0
                raw(3) = raw(3)+0.20*turnEvidence;
            else
                raw(4) = raw(4)+0.20*turnEvidence;
            end
            raw = max(raw,double(object.MinimumModeProbability));
            probabilities = single(raw/sum(raw));
        end

        function [positions,yaws,covariances] = predictOne(object,tracks,index,classID, ...
                numberModes,filteredAcceleration,filteredYawRate,timeOffsets)
            positions = zeros(1,object.PredictionSteps,numberModes,2,"single");
            yaws = zeros(1,object.PredictionSteps,numberModes,"single");
            covariances = zeros(2,2,object.PredictionSteps,numberModes,"single");
            initialPosition = double(tracks.Positions(index,1:2));
            initialVelocity = double(tracks.Velocities(index,1:2));
            speed = norm(initialVelocity);
            initialYaw = double(tracks.Yaws(index));
            if speed > 0.20
                initialYaw = atan2(initialVelocity(2),initialVelocity(1));
            end

            [turnMagnitude,brakeAcceleration,qLongitudinal,qLateral] = ...
                object.classParameters(classID);
            modeAccelerations = [double(filteredAcceleration) brakeAcceleration 0 0];
            modeYawRates = [double(filteredYawRate) 0 turnMagnitude -turnMagnitude];
            if numberModes == 1
                modeAccelerations(1) = 0;
                modeYawRates(1) = 0;
                speed = 0;
            end

            rawCovariance = double(tracks.StateCovariances(:,:,index));
            initialPositionCovariance = object.validCovariance(rawCovariance(1:2,1:2),0.25);
            initialVelocityCovariance = object.validCovariance(rawCovariance(4:5,4:5),1.0);
            existenceInflation = 1+2*max(0,1-double(tracks.ExistenceProbabilities(index)));

            for mode = 1:numberModes
                currentPosition = initialPosition;
                currentSpeed = speed;
                currentYaw = initialYaw;
                positions(1,1,mode,:) = single(currentPosition);
                yaws(1,1,mode) = single(currentYaw);
                covariances(:,:,1,mode) = single(initialPositionCovariance*existenceInflation);
                for step = 2:object.PredictionSteps
                    deltaTime = double(timeOffsets(step)-timeOffsets(step-1));
                    newSpeed = max(0,currentSpeed+modeAccelerations(mode)*deltaTime);
                    middleYaw = currentYaw+0.5*modeYawRates(mode)*deltaTime;
                    distance = 0.5*(currentSpeed+newSpeed)*deltaTime;
                    currentPosition = currentPosition+distance*[cos(middleYaw) sin(middleYaw)];
                    currentYaw = object.wrapAngle(currentYaw+modeYawRates(mode)*deltaTime);
                    currentSpeed = newSpeed;
                    positions(1,step,mode,:) = single(currentPosition);
                    yaws(1,step,mode) = single(currentYaw);

                    futureTime = double(timeOffsets(step));
                    rotation = [cos(currentYaw) -sin(currentYaw); ...
                        sin(currentYaw) cos(currentYaw)];
                    modeInflation = [1.0 1.15 1.30 1.30];
                    processCovariance = rotation*diag([qLongitudinal qLateral])*rotation.' ...
                        *(futureTime^3/3)*modeInflation(mode);
                    covariance = initialPositionCovariance+ ...
                        futureTime^2*initialVelocityCovariance+processCovariance;
                    covariances(:,:,step,mode) = single( ...
                        (covariance+covariance.')/2*existenceInflation);
                end
            end
        end

        function [turnRate,brake,qLongitudinal,qLateral] = classParameters(object,classID)
            turnRate = double(object.VehicleTurnRate);
            brake = double(object.VehicleBrake);
            qLongitudinal = 0.25;
            qLateral = 0.12;
            switch double(classID)
                case {2,3}
                    turnRate = 0.18; brake = -2.2;
                    qLongitudinal = 0.18; qLateral = 0.08;
                case 4
                    turnRate = 0.35;
                    qLongitudinal = 0.35; qLateral = 0.25;
                case 5
                    turnRate = 0.45;
                    qLongitudinal = 0.50; qLateral = 0.45;
                case 6
                    turnRate = 0.55; brake = double(object.VulnerableBrake);
                    qLongitudinal = 0.60; qLateral = 0.80;
                case {7,8}
                    turnRate = double(object.VulnerableTurnRate);
                    brake = double(object.VulnerableBrake);
                    qLongitudinal = 0.70; qLateral = 1.00;
                case 9
                    turnRate = 0.85; brake = -2.0;
                    qLongitudinal = 1.20; qLateral = 1.50;
                case 10
                    turnRate = 0; brake = 0;
                    qLongitudinal = 0.01; qLateral = 0.01;
                case 0
                    turnRate = 0.75; brake = -2.5;
                    qLongitudinal = 1.00; qLateral = 1.20;
            end
        end

        function covariance = validCovariance(~,raw,defaultVariance)
            covariance = double(raw);
            if any(~isfinite(covariance),"all") || any(diag(covariance) <= 0)
                covariance = eye(2)*defaultVariance;
            end
            covariance = (covariance+covariance.')/2+eye(2)*1e-5;
        end

        function removeStaleHistory(object,activeTrackIDs)
            for slot = 1:object.MaxTracks
                if object.HistoryTrackIDs(slot) ~= 0 && ...
                        ~any(activeTrackIDs == object.HistoryTrackIDs(slot))
                    object.HistoryTrackIDs(slot) = 0;
                    object.PreviousVelocities(slot,:) = 0;
                    object.PreviousYaws(slot) = 0;
                    object.PreviousTimes(slot) = -inf;
                    object.FilteredAccelerations(slot) = 0;
                    object.FilteredYawRates(slot) = 0;
                end
            end
        end

        function angle = angleDifference(object,first,second)
            angle = object.wrapAngle(first-second);
        end

        function angle = wrapAngle(~,angle)
            angle = mod(angle+pi,2*pi)-pi;
        end

        function [trackIDs,classIDs,numModes,modeProbabilities,positions,yaws, ...
                positionCovariances,validMask] = emptyOutputs(object)
            vector=[object.MaxTracks 1];
            trackIDs=zeros(vector,"uint32"); classIDs=zeros(vector,"uint8");
            numModes=zeros(vector,"uint8");
            modeProbabilities=zeros(object.MaxTracks,object.MaxModes,"single");
            positions=zeros(object.MaxTracks,object.PredictionSteps,object.MaxModes,2,"single");
            yaws=zeros(object.MaxTracks,object.PredictionSteps,object.MaxModes,"single");
            positionCovariances=zeros(2,2,object.PredictionSteps,object.MaxModes,object.MaxTracks,"single");
            validMask=false(vector);
        end
    end
end
