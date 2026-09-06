classdef RoadsenseBehaviourAssessmentSystem < matlab.System
    %ROADSENSEBEHAVIOURASSESSMENTSYSTEM Convert maps/tracks into decisions cues.

    properties (Nontunable)
        MaxTracks (1,1) double = 128
        GridRows (1,1) double = 200
        GridCols (1,1) double = 320
        GridResolution (1,1) single = single(0.25)
        GridXLimits (2,1) single = single([-20;60])
        GridYLimits (2,1) single = single([-25;25])
    end

    properties
        ReactionTime (1,1) single = single(0.60)
        EmergencyDeceleration (1,1) single = single(6.0)
        ComfortDeceleration (1,1) single = single(3.0)
        EmergencyTTC (1,1) single = single(1.2)
        FollowTimeGap (1,1) single = single(1.8)
        CreepSpeed (1,1) single = single(1.2)
        CautiousSpeed (1,1) single = single(3.0)
        AvoidSpeed (1,1) single = single(4.0)
    end

    properties (Access = private)
        XGrid
        YGrid
    end

    methods (Access = protected)
        function setupImpl(object)
            x = double(object.GridXLimits(1)) + ...
                (0.5:object.GridCols-0.5)*double(object.GridResolution);
            y = double(object.GridYLimits(1)) + ...
                (0.5:object.GridRows-0.5)*double(object.GridResolution);
            [object.XGrid,object.YGrid] = meshgrid(x,y);
        end

        function [timestamp,inputsValid,emergencyHazard,goalStop,yieldRequired, ...
                followRequired,creepRecommended,avoidRecommended,cautiousRecommended, ...
                cruiseTarget,followTarget,yieldTarget,creepTarget,cautiousTarget, ...
                avoidTarget,desiredClearance,stopDistance,leadTrackID,minimumTTC, ...
                forwardRisk,unknownFraction,reasonMask] = stepImpl(object,ego,map,tracks,route)
            timestamp = max([ego.Timestamp map.Timestamp tracks.Timestamp route.Timestamp]);
            inputsValid = logical(ego.Valid && map.Valid && route.Valid && isfinite(timestamp));
            emergencyHazard=false; goalStop=false; yieldRequired=false;
            followRequired=false; creepRecommended=false; avoidRecommended=false;
            cautiousRecommended=false;
            cruiseTarget=single(0); followTarget=single(0); yieldTarget=single(0);
            creepTarget=object.CreepSpeed; cautiousTarget=single(0); avoidTarget=single(0);
            desiredClearance=single(1.0); stopDistance=single(0);
            leadTrackID=uint32(0); minimumTTC=single(inf);
            forwardRisk=single(0); unknownFraction=single(1); reasonMask=uint16(0);
            if ~inputsValid
                reasonMask=uint16(1);
                return
            end

            egoSpeed = norm(double(ego.Velocity(1:2)));
            cruiseTarget = single(max(0,min(double(route.DesiredSpeed),double(route.SpeedLimit))));
            cautiousTarget = min(cruiseTarget,object.CautiousSpeed);
            avoidTarget = min(cruiseTarget,object.AvoidSpeed);
            stopDistance = single(egoSpeed*double(object.ReactionTime) + ...
                egoSpeed^2/(2*double(object.ComfortDeceleration)) + 2.0);
            emergencyDistance = egoSpeed*0.30 + ...
                egoSpeed^2/(2*double(object.EmergencyDeceleration)) + 1.0;

            lookAhead = min(max(double(stopDistance)+8,12),30);
            forwardMask = object.XGrid >= 0 & object.XGrid <= lookAhead & abs(object.YGrid) <= 1.6;
            emergencyMask = object.XGrid >= 0 & object.XGrid <= emergencyDistance & abs(object.YGrid) <= 1.4;
            visibilityMask = object.XGrid >= 0 & object.XGrid <= 25 & abs(object.YGrid) <= 4.0;
            blockageMask = object.XGrid >= 1 & object.XGrid <= min(max(double(stopDistance)+4,8),18) ...
                & abs(object.YGrid) <= 1.5;
            leftMask = object.XGrid >= 1 & object.XGrid <= 16 & object.YGrid >= 2 & object.YGrid <= 5;
            rightMask = object.XGrid >= 1 & object.XGrid <= 16 & object.YGrid <= -2 & object.YGrid >= -5;

            forwardRisk = single(max(max(map.Occupancy(forwardMask)), ...
                max(map.PredictedRisk(forwardMask))));
            unknownFraction = single(1-nnz(map.ObservedMask(visibilityMask))/nnz(visibilityMask));
            maximumSurface = max(map.SurfaceCost(forwardMask));
            % The prediction grid is a union over multiple future times and
            % modes, so it is suitable for caution and replanning but not for
            % declaring a present collision. Immediate emergency occupancy
            % comes from current static/dynamic evidence; time-indexed future
            % collisions remain enforced independently by the local planner.
            emergencyMapRisk = max(map.Occupancy(emergencyMask));
            blocked = max(map.Occupancy(blockageMask)) >= single(0.60);
            % Determine whether an avoidance shoulder exists from persistent
            % infrastructure/surface evidence only. Dynamic and predicted
            % risk must not veto the very manoeuvre intended to avoid that
            % actor; the local planner checks those trajectories in time.
            leftBlockedFraction = mean(map.StaticOccupancy(leftMask)>=single(0.65) | ...
                map.SurfaceCost(leftMask)>=single(0.80));
            rightBlockedFraction = mean(map.StaticOccupancy(rightMask)>=single(0.65) | ...
                map.SurfaceCost(rightMask)>=single(0.80));
            lateralPathFree = min(leftBlockedFraction,rightBlockedFraction) < 0.35;

            nearestLeadDistance = inf;
            nearestLeadRelativeSpeed = 0;
            vulnerableConflict = false;
            oncomingConflict = false;
            if tracks.Valid
                count = min(double(tracks.Count),object.MaxTracks);
                for index = 1:count
                    if ~tracks.ValidMask(index); continue; end
                    x = double(tracks.Positions(index,1));
                    y = double(tracks.Positions(index,2));
                    relativeForwardSpeed = double(tracks.Velocities(index,1));
                    classID = tracks.ClassIDs(index);
                    dimensions = max(double(tracks.Dimensions(index,1:2)),[0.5 0.5]);
                    lateralConflict = abs(y) <= 1.2+dimensions(2)/2;
                    if x > 0 && lateralConflict
                        if relativeForwardSpeed < -0.10
                            ttc = max(0,(x-dimensions(1)/2-2.2)/(-relativeForwardSpeed));
                            minimumTTC = min(minimumTTC,single(ttc));
                        end
                        if x < nearestLeadDistance
                            nearestLeadDistance=x;
                            nearestLeadRelativeSpeed=relativeForwardSpeed;
                            leadTrackID=tracks.TrackIDs(index);
                        end
                        isMotorVehicle = classID >= uint8(1) && classID <= uint8(5);
                        if isMotorVehicle && x <= 50 && relativeForwardSpeed < -1.0
                            oncomingConflict = true;
                        end
                    end
                    isVulnerable = classID >= uint8(6) && classID <= uint8(9);
                    if isVulnerable && x >= -2 && x <= 14 && abs(y) <= 5
                        vulnerableConflict=true;
                    end
                end
            end

            emergencyHazard = logical(minimumTTC < object.EmergencyTTC || emergencyMapRisk >= 0.92);
            goalStop = logical(route.GoalReached || route.DistanceToGoal <= single(1.0));
            junctionRisk = logical((route.UncontrolledIntersection || route.MergeRequired) ...
                && forwardRisk >= single(0.38));
            yieldRequired = logical(vulnerableConflict || junctionRisk);
            desiredGap = max(6,double(object.FollowTimeGap)*egoSpeed+3);
            followRequired = logical(isfinite(nearestLeadDistance) && ...
                nearestLeadDistance <= desiredGap+4 && ~yieldRequired);
            if followRequired
                estimatedLeadSpeed = max(0,egoSpeed+nearestLeadRelativeSpeed);
                gapCorrection = 0.35*(nearestLeadDistance-desiredGap);
                followTarget = single(max(0,min(double(cruiseTarget),estimatedLeadSpeed+gapCorrection)));
            else
                followTarget=cruiseTarget;
                leadTrackID=uint32(0);
            end
            yieldTarget=single(0);
            creepRecommended = logical((route.OccludedArea || route.UncontrolledIntersection) ...
                && ~yieldRequired && unknownFraction >= single(0.25) && ...
                egoSpeed < 0.80*double(object.CreepSpeed) && ego.Timestamp < 2.0);
            avoidRecommended = logical((blocked || oncomingConflict) && ...
                route.AllowObstacleAvoidance && lateralPathFree && ~vulnerableConflict);
            cautiousRecommended = logical(forwardRisk >= single(0.45) ...
                || (route.OccludedArea && unknownFraction >= single(0.40)) ...
                || maximumSurface >= single(0.40));
            if vulnerableConflict; desiredClearance=single(2.0); end

            if emergencyHazard; reasonMask=bitor(reasonMask,uint16(2)); end
            if goalStop; reasonMask=bitor(reasonMask,uint16(4)); end
            if vulnerableConflict; reasonMask=bitor(reasonMask,uint16(8)); end
            if followRequired; reasonMask=bitor(reasonMask,uint16(16)); end
            if blocked; reasonMask=bitor(reasonMask,uint16(32)); end
            if unknownFraction >= single(0.40); reasonMask=bitor(reasonMask,uint16(64)); end
            if maximumSurface >= single(0.40); reasonMask=bitor(reasonMask,uint16(128)); end
            if route.UncontrolledIntersection || route.MergeRequired
                reasonMask=bitor(reasonMask,uint16(256));
            end
            if oncomingConflict; reasonMask=bitor(reasonMask,uint16(512)); end
        end

        function number = getNumInputsImpl(~); number=4; end
        function number = getNumOutputsImpl(~); number=22; end
        function names = getInputNamesImpl(~)
            names=["EgoState","SemanticGrid","FusedTracks","RouteContext"];
        end
        function varargout = getOutputNamesImpl(~)
            varargout={'Timestamp','InputsValid','EmergencyHazard','GoalStop', ...
                'YieldRequired','FollowRequired','CreepRecommended','AvoidRecommended', ...
                'CautiousRecommended','CruiseTarget','FollowTarget','YieldTarget', ...
                'CreepTarget','CautiousTarget','AvoidTarget','DesiredClearance', ...
                'StopDistance','LeadTrackID','MinimumTTC','ForwardRisk', ...
                'UnknownFraction','ReasonMask'};
        end
        function varargout = getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout = getOutputDataTypeImpl(~)
            varargout={'double','logical','logical','logical','logical','logical', ...
                'logical','logical','logical','single','single','single','single', ...
                'single','single','single','single','uint32','single','single', ...
                'single','uint16'};
        end
        function varargout = isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout = isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon = getIconImpl(~)
            icon="Roadsense\nHazard and Context\nAssessment";
        end
    end
end
