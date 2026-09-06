classdef RoadsenseLocalPlannerSystem < matlab.System
    %ROADSENSELOCALPLANNERSYSTEM Lane-independent, uncertainty-aware planner.

    properties (Nontunable)
        MaxPlanPoints (1,1) double = 61
        MaxReferencePoints (1,1) double = 256
        MaxTracks (1,1) double = 128
        MaxModes (1,1) double = 4
        PredictionSteps (1,1) double = 16
        GridRows (1,1) double = 200
        GridCols (1,1) double = 320
    end

    properties
        PlanStep (1,1) single = single(0.10)
        PlanHorizon (1,1) single = single(6.0)
        LateralOffsets (:,1) single = single([-4;-2;-1;0;1;2;4])
        SpeedScales (:,1) single = single([0.70;1.00;1.15])
        Wheelbase (1,1) single = single(2.8)
        VehicleLength (1,1) single = single(4.6)
        VehicleWidth (1,1) single = single(1.9)
        MaximumCurvature (1,1) single = single(0.22)
        MaximumJerk (1,1) single = single(2.5)
        HardOccupancy (1,1) single = single(0.65)
        CollisionProbability (1,1) single = single(0.05)
        PredictionSigma (1,1) single = single(2.0)
        MaximumLongitudinalInflation (1,1) single = single(3.0)
        MaximumLateralInflation (1,1) single = single(2.5)
        LateralClearanceRatio (1,1) single = single(0.35)
        MaximumLateralClearance (1,1) single = single(0.80)
        Deadline (1,1) single = single(0.10)
    end

    properties (Access=private)
        NextPlanID (1,1) uint32 = uint32(0)
        AvoidanceSide (1,1) int8 = int8(0)
        AvoidanceDirected (1,1) logical = false
    end

    methods (Access=protected)
        function [timestamp,planID,count,timeFromStart,positions,yaws,speeds, ...
                accelerations,curvatures,steeringAngles,totalCost,minimumClearance, ...
                candidateCount,feasibleCount,emergencyFallback,planValid, ...
                statusTimestamp,statusPlanID,plannerCode,behaviourCode, ...
                statusCandidateCount,statusFeasibleCount,executionTime,deadline, ...
                replanRequested,emergencyRequested] = stepImpl(object,ego,map, ...
                tracks,predictions,behaviour,reference)
            startTime=tic;
            timestamp=max([ego.Timestamp map.Timestamp tracks.Timestamp ...
                predictions.Timestamp behaviour.Timestamp reference.Timestamp]);
            planID=uint32(0); count=uint16(0);
            timeFromStart=zeros(object.MaxPlanPoints,1,"single");
            positions=zeros(object.MaxPlanPoints,2,"single");
            yaws=zeros(object.MaxPlanPoints,1,"single");
            speeds=zeros(object.MaxPlanPoints,1,"single");
            accelerations=zeros(object.MaxPlanPoints,1,"single");
            curvatures=zeros(object.MaxPlanPoints,1,"single");
            steeringAngles=zeros(object.MaxPlanPoints,1,"single");
            totalCost=single(inf); minimumClearance=single(0);
            candidateCount=uint16(0); feasibleCount=uint16(0);
            emergencyFallback=false; planValid=false;
            behaviourCode=uint8(behaviour.Mode);
            plannerCode=uint8(4);
            criticalValid=logical(ego.Valid && map.Valid && tracks.Valid && ...
                predictions.Valid && behaviour.Valid && reference.Valid && ...
                isfinite(timestamp) && reference.Count >= uint16(2));
            if criticalValid
                referenceCount=min(double(reference.Count),object.MaxReferencePoints);
                criticalValid=object.referenceIsUsable(reference,referenceCount) && ...
                    object.mapGeometryIsUsable(map);
            else
                referenceCount=0;
            end

            if criticalValid
                object.NextPlanID=object.NextPlanID+uint32(1);
                planID=object.NextPlanID;
                if behaviourCode==uint8(10)
                    closingSide=object.sideAwayFromClosingTrack(tracks);
                    if closingSide~=int8(0) && ~object.AvoidanceDirected
                        object.AvoidanceSide=closingSide;
                        object.AvoidanceDirected=true;
                    end
                else
                    object.AvoidanceSide=int8(0);
                    object.AvoidanceDirected=false;
                end
                [best,bestCost,bestClearance,candidateCount,feasibleCount]= ...
                    object.searchCandidates(ego,map,tracks,predictions,behaviour, ...
                    reference,referenceCount);
                forceFallback=behaviourCode==uint8(8) || behaviour.EmergencyRequested;
                if best.Valid && ~forceFallback
                    if behaviourCode==uint8(10) && abs(best.TargetOffset)>single(0.5)
                        object.AvoidanceSide=int8(sign(best.TargetOffset));
                    elseif behaviourCode~=uint8(10)
                        object.AvoidanceSide=int8(0);
                    end
                    [timeFromStart,positions,yaws,speeds,accelerations,curvatures, ...
                        steeringAngles]=object.toWorldPlan(best,ego);
                    count=uint16(object.MaxPlanPoints);
                    totalCost=single(bestCost);
                    minimumClearance=single(bestClearance);
                    planValid=true;
                    plannerCode=uint8(1);
                else
                    fallback=object.buildEmergencyCandidate(ego,behaviour,reference,referenceCount);
                    [~,~,fallbackClearance]=object.evaluateCandidate( ...
                        fallback,map,tracks,predictions,behaviour,false);
                    [timeFromStart,positions,yaws,speeds,accelerations,curvatures, ...
                        steeringAngles]=object.toWorldPlan(fallback,ego);
                    count=uint16(object.MaxPlanPoints);
                    totalCost=single(bestCost);
                    if ~isfinite(totalCost); totalCost=single(1e6); end
                    minimumClearance=single(fallbackClearance);
                    emergencyFallback=true;
                    planValid=fallback.Valid;
                    plannerCode=uint8(6);
                end
            end

            executionTime=single(toc(startTime));
            deadline=object.Deadline;
            if planValid && executionTime > deadline && plannerCode==uint8(1)
                plannerCode=uint8(5);
            end
            statusTimestamp=timestamp; statusPlanID=planID;
            statusCandidateCount=candidateCount; statusFeasibleCount=feasibleCount;
            replanRequested=logical(~planValid || emergencyFallback || ...
                behaviour.ReplanRequested || executionTime>deadline);
            emergencyRequested=logical(~planValid || emergencyFallback || ...
                behaviour.EmergencyRequested);
        end

        function resetImpl(object)
            object.NextPlanID=uint32(0);
            object.AvoidanceSide=int8(0);
            object.AvoidanceDirected=false;
        end

        function number=getNumInputsImpl(~); number=6; end
        function number=getNumOutputsImpl(~); number=26; end
        function names=getInputNamesImpl(~)
            names=["EgoState","SemanticGrid","FusedTracks","Predictions", ...
                "BehaviourCommand","ReferencePath"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','PlanID','Count','TimeFromStart','Positions', ...
                'Yaws','Speeds','Accelerations','Curvatures','SteeringAngles', ...
                'TotalCost','MinimumClearance','CandidateCount','FeasibleCount', ...
                'EmergencyFallback','PlanValid','StatusTimestamp','StatusPlanID', ...
                'PlannerCode','BehaviourCode','StatusCandidateCount', ...
                'StatusFeasibleCount','ExecutionTime','Deadline', ...
                'ReplanRequested','EmergencyRequested'};
        end
        function varargout=getOutputSizeImpl(object)
            one=[1 1]; n=[object.MaxPlanPoints 1];
            varargout={one,one,one,n,[object.MaxPlanPoints 2],n,n,n,n,n, ...
                one,one,one,one,one,one,one,one,one,one,one,one,one,one,one,one};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','uint16','single','single','single', ...
                'single','single','single','single','single','single','uint16', ...
                'uint16','logical','logical','double','uint32','uint8','uint8', ...
                'uint16','uint16','single','single','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nAdaptive Local Planner\nSampling + Safety Filter";
        end
    end

    methods (Access=private)
        function [best,bestCost,bestClearance,candidateCount,feasibleCount]= ...
                searchCandidates(object,ego,map,tracks,predictions,behaviour,reference,nref)
            best=object.emptyCandidate(); bestCost=inf; bestClearance=0;
            candidateCount=uint16(0); feasibleCount=uint16(0);
            for lateralIndex=1:numel(object.LateralOffsets)
                for speedIndex=1:numel(object.SpeedScales)
                    candidateCount=candidateCount+uint16(1);
                    candidate=object.buildCandidate(ego,behaviour,reference,nref, ...
                        object.LateralOffsets(lateralIndex),object.SpeedScales(speedIndex));
                    [feasible,cost,clearance]=object.evaluateCandidate( ...
                        candidate,map,tracks,predictions,behaviour,true);
                    if feasible
                        feasibleCount=feasibleCount+uint16(1);
                        if cost<bestCost
                            best=candidate; bestCost=cost; bestClearance=clearance;
                        end
                    end
                end
            end
        end

        function candidate=buildCandidate(object,ego,behaviour,reference,nref,targetOffset,speedScale)
            candidate=object.emptyCandidate();
            candidate.TargetOffset=single(targetOffset);
            dt=double(object.PlanStep);
            candidate.Time=single((0:object.MaxPlanPoints-1).')*object.PlanStep;
            initialSpeed=max(0,object.longitudinalSpeed(ego));
            initialAcceleration=object.longitudinalAcceleration(ego);
            minimumAcceleration=min(double(behaviour.MinimumAcceleration),0);
            maximumAcceleration=max(double(behaviour.MaximumAcceleration),0);
            initialAcceleration=min(max(initialAcceleration,minimumAcceleration),maximumAcceleration);
            % A braking command can remain applied while the plant is held at
            % zero speed.  That command is not a physical negative-velocity
            % acceleration state and must not seed a spurious first-sample
            % jerk in a newly planned standstill trajectory.
            if initialSpeed<=0.05 && initialAcceleration<0
                initialAcceleration=0;
            end
            speed=initialSpeed; acceleration=initialAcceleration; progress=0;
            transitionDistance=max(10,2.5*max(initialSpeed,double(behaviour.TargetSpeed)));
            segmentIndex=2; cumulativeStart=0;
            routeSpeed=double(reference.RecommendedSpeeds(1));
            for point=1:object.MaxPlanPoints
                if point>1
                    targetSpeed=min(max(0,double(behaviour.TargetSpeed)*double(speedScale)), ...
                        max(0,routeSpeed));
                    desiredAcceleration=(targetSpeed-speed)/1.5;
                    desiredAcceleration=min(max(desiredAcceleration,minimumAcceleration),maximumAcceleration);
                    maximumDelta=double(object.MaximumJerk)*dt;
                    acceleration=min(max(desiredAcceleration,acceleration-maximumDelta), ...
                        acceleration+maximumDelta);
                    newSpeed=max(0,speed+acceleration*dt);
                    if targetSpeed<=0.01 && newSpeed<0.15
                        newSpeed=0; acceleration=0;
                    end
                    progress=progress+0.5*(speed+newSpeed)*dt;
                    speed=newSpeed;
                end
                [basePosition,baseYaw,routeSpeed,segmentIndex,cumulativeStart]= ...
                    object.referenceAtForward(reference,nref,progress,segmentIndex,cumulativeStart);
                u=min(max(progress/transitionDistance,0),1);
                blend=10*u^3-15*u^4+6*u^5;
                lateral=double(targetOffset)*blend;
                candidate.Position(point,:)=single(basePosition+ ...
                    lateral*[-sin(baseYaw) cos(baseYaw)]);
                candidate.Speed(point)=single(speed);
                candidate.Acceleration(point)=single(acceleration);
            end
            candidate=object.finishGeometry(candidate);
        end

        function candidate=buildEmergencyCandidate(object,ego,behaviour,reference,nref)
            candidate=object.emptyCandidate();
            candidate.Time=single((0:object.MaxPlanPoints-1).')*object.PlanStep;
            dt=double(object.PlanStep); speed=max(0,object.longitudinalSpeed(ego));
            acceleration=min(0,object.longitudinalAcceleration(ego)); progress=0;
            if speed<=0.05 && acceleration<0; acceleration=0; end
            minimumAcceleration=min(double(behaviour.MinimumAcceleration),-6.0);
            segmentIndex=2; cumulativeStart=0;
            for point=1:object.MaxPlanPoints
                if point>1
                    acceleration=max(minimumAcceleration, ...
                        acceleration-double(object.MaximumJerk)*dt);
                    newSpeed=max(0,speed+acceleration*dt);
                    progress=progress+0.5*(speed+newSpeed)*dt;
                    speed=newSpeed;
                    if speed==0; acceleration=0; end
                end
                [basePosition,~,~,segmentIndex,cumulativeStart]= ...
                    object.referenceAtForward(reference,nref,progress,segmentIndex,cumulativeStart);
                candidate.Position(point,:)=single(basePosition);
                candidate.Speed(point)=single(speed);
                candidate.Acceleration(point)=single(acceleration);
            end
            candidate=object.finishGeometry(candidate);
        end

        function candidate=finishGeometry(object,candidate)
            n=object.MaxPlanPoints;
            for point=1:n-1
                delta=double(candidate.Position(point+1,:)-candidate.Position(point,:));
                if norm(delta)>1e-5
                    candidate.Yaw(point)=single(atan2(delta(2),delta(1)));
                elseif point>1
                    candidate.Yaw(point)=candidate.Yaw(point-1);
                end
            end
            candidate.Yaw(n)=candidate.Yaw(max(1,n-1));
            for point=2:n-1
                distance=norm(double(candidate.Position(point+1,:)-candidate.Position(point-1,:)));
                if distance>1e-4
                    deltaYaw=object.wrapAngle(double(candidate.Yaw(point+1))-double(candidate.Yaw(point-1)));
                    candidate.Curvature(point)=single(deltaYaw/distance);
                end
            end
            candidate.Curvature(1)=candidate.Curvature(2);
            candidate.Curvature(n)=candidate.Curvature(n-1);
            candidate.Steering=single(atan(double(object.Wheelbase)*double(candidate.Curvature)));
            candidate.Valid=all(isfinite(candidate.Position(:))) && ...
                all(isfinite(candidate.Speed)) && all(isfinite(candidate.Curvature));
        end

        function [feasible,cost,minimumClearance]=evaluateCandidate(object,candidate, ...
                map,tracks,predictions,behaviour,enforceCollision)
            feasible=candidate.Valid; minimumClearance=inf;
            if max(abs(candidate.Curvature))>object.MaximumCurvature+single(1e-4)
                feasible=false;
            end
            jerk=diff(double(candidate.Acceleration))/double(object.PlanStep);
            if ~isempty(jerk) && max(abs(jerk))>double(object.MaximumJerk)+1e-3
                feasible=false;
            end
            [mapSafe,mapCost]=object.evaluateMapCandidate(candidate,map);
            if ~mapSafe
                if enforceCollision; feasible=false; end
                minimumClearance=min(minimumClearance,0);
            end
            dynamicRisk=0;
            if predictions.Valid && predictions.Count>0
                for point=1:2:object.MaxPlanPoints
                    [collision,clearance,risk]=object.dynamicSafety(candidate.Position(point,:), ...
                        candidate.Yaw(point),candidate.Time(point),tracks,predictions,behaviour);
                    minimumClearance=min(minimumClearance,clearance);
                    dynamicRisk=dynamicRisk+risk;
                    if collision && enforceCollision; feasible=false; end
                end
            end
            if ~isfinite(minimumClearance); minimumClearance=1000; end
            lateralEnd=abs(double(candidate.Position(end,2)));
            speedError=double(candidate.Speed(end)-behaviour.TargetSpeed);
            curvatureEffort=mean(double(candidate.Curvature).^2);
            jerkEffort=mean(jerk.^2);
            progress=double(candidate.Position(end,1));
            evaluationCount=ceil(object.MaxPlanPoints/2);
            lateralWeight=1.3; avoidanceBias=0;
            if uint8(behaviour.Mode)==uint8(10)
                lateralWeight=0.15;
                maximumOffset=max(abs(double(object.LateralOffsets)));
                avoidanceBias=0.45*(maximumOffset-abs(double(candidate.TargetOffset)))^2;
                candidateSide=sign(double(candidate.TargetOffset));
                if object.AvoidanceSide~=int8(0) && candidateSide~=0 && ...
                        candidateSide~=double(object.AvoidanceSide)
                    avoidanceBias=avoidanceBias+20;
                end
            end
            cost=mapCost/evaluationCount+dynamicRisk/evaluationCount+ ...
                lateralWeight*lateralEnd^2+0.12*speedError^2+ ...
                16*curvatureEffort+0.015*jerkEffort+avoidanceBias-0.035*progress;
        end

        function [safe,totalCost]=evaluateMapCandidate(object,candidate,map)
            halfLength=double(object.VehicleLength)/2+0.20;
            halfWidth=double(object.VehicleWidth)/2+0.20;
            offsets=[0 0; halfLength halfWidth; halfLength -halfWidth; ...
                -halfLength halfWidth; -halfLength -halfWidth; halfLength 0; ...
                -halfLength 0; 0 halfWidth; 0 -halfWidth];
            safe=true; totalCost=0;
            for point=1:2:object.MaxPlanPoints
                yaw=double(candidate.Yaw(point));
                rotation=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
                samples=offsets*rotation.'+double(candidate.Position(point,:));
                staticMaximum=single(0); combined=single(0); surface=single(0);
                predicted=single(0); drivable=single(1);
                for index=1:9
                    column=floor((samples(index,1)-double(map.XLimits(1)))/double(map.Resolution))+1;
                    row=floor((samples(index,2)-double(map.YLimits(1)))/double(map.Resolution))+1;
                    if row<1 || row>object.GridRows || column<1 || column>object.GridCols
                        safe=false; continue
                    end
                    staticMaximum=max(staticMaximum,map.StaticOccupancy(row,column));
                    combined=max(combined,map.CombinedCost(row,column));
                    surface=max(surface,map.SurfaceCost(row,column));
                    predicted=max(predicted,map.PredictedRisk(row,column));
                    drivable=min(drivable,map.Drivability(row,column));
                end
                if staticMaximum>=object.HardOccupancy; safe=false; end
                totalCost=totalCost+double(combined)+0.45*double(surface)+ ...
                    0.65*double(predicted)+0.35*(1-double(drivable));
            end
        end

        function [collision,minimumClearance,risk]=dynamicSafety(object,position,yaw,time, ...
                tracks,predictions,behaviour)
            collision=false; minimumClearance=inf; risk=0;
            if ~predictions.Valid || predictions.Count==0; return; end
            steps=min(double(predictions.NumSteps),object.PredictionSteps);
            if steps<1 || double(time)>double(predictions.TimeOffsets(steps))+1e-5; return; end
            [~,step]=min(abs(double(predictions.TimeOffsets(1:steps))-double(time)));
            objects=min(double(predictions.Count),object.MaxTracks);
            for objectIndex=1:objects
                if ~predictions.ValidMask(objectIndex); continue; end
                trackIndex=object.findTrack(tracks,predictions.TrackIDs(objectIndex));
                existence=1; dimensions=[1.0 0.8];
                if trackIndex>0
                    existence=max(0,double(tracks.ExistenceProbabilities(trackIndex)));
                    dimensions=max(double(tracks.Dimensions(trackIndex,1:2)),[0.5 0.5]);
                end
                modes=min(double(predictions.NumModes(objectIndex)),object.MaxModes);
                for mode=1:modes
                    probability=double(predictions.ModeProbabilities(objectIndex,mode))*existence;
                    if probability<double(object.CollisionProbability); continue; end
                    objectPosition=[double(predictions.Positions(objectIndex,step,mode,1)) ...
                        double(predictions.Positions(objectIndex,step,mode,2))];
                    objectYaw=double(predictions.Yaws(objectIndex,step,mode));
                    covariance=double(predictions.PositionCovariances(:,:,step,mode,objectIndex));
                    objectRotation=[cos(objectYaw) sin(objectYaw); ...
                        -sin(objectYaw) cos(objectYaw)];
                    localCovariance=objectRotation*covariance*objectRotation.';
                    sigmaLong=min(double(object.MaximumLongitudinalInflation), ...
                        double(object.PredictionSigma)*sqrt(max(localCovariance(1,1),0)));
                    sigmaLat=min(double(object.MaximumLateralInflation), ...
                        double(object.PredictionSigma)*sqrt(max(localCovariance(2,2),0)));
                    longitudinalClearance=max(0,double(behaviour.DesiredClearance));
                    % The behaviour clearance is primarily a following/stopping
                    % distance. Applying all of it laterally, in addition to the
                    % covariance envelope, can make a mixed-traffic road wider
                    % than the complete candidate lattice. Retain a bounded
                    % shoulder for side-by-side passing while preserving the
                    % complete requested margin in the longitudinal direction.
                    lateralClearance=min(double(object.MaximumLateralClearance), ...
                        double(object.LateralClearanceRatio)*longitudinalClearance);
                    relativeYaw=double(yaw)-objectYaw;
                    egoLong=double(object.VehicleLength)/2*abs(cos(relativeYaw))+ ...
                        double(object.VehicleWidth)/2*abs(sin(relativeYaw));
                    egoLat=double(object.VehicleLength)/2*abs(sin(relativeYaw))+ ...
                        double(object.VehicleWidth)/2*abs(cos(relativeYaw));
                    halfLong=egoLong+dimensions(1)/2+longitudinalClearance+sigmaLong;
                    halfLat=egoLat+dimensions(2)/2+lateralClearance+sigmaLat;
                    delta=double(position)-objectPosition;
                    local=objectRotation*delta.';
                    separatedLong=abs(local(1))-halfLong;
                    separatedLat=abs(local(2))-halfLat;
                    signedClearance=max(separatedLong,separatedLat);
                    minimumClearance=min(minimumClearance,signedClearance);
                    risk=risk+probability*exp(-0.5*((local(1)/max(halfLong,0.1))^2+ ...
                        (local(2)/max(halfLat,0.1))^2));
                    if separatedLong<=0 && separatedLat<=0; collision=true; end
                end
            end
        end

        function index=findTrack(object,tracks,trackID)
            index=0;
            if ~tracks.Valid; return; end
            count=min(double(tracks.Count),object.MaxTracks);
            for candidate=1:count
                if tracks.ValidMask(candidate) && tracks.TrackIDs(candidate)==trackID
                    index=candidate; return
                end
            end
        end

        function side=sideAwayFromClosingTrack(object,tracks)
            side=int8(0); nearest=inf; lateral=0;
            if ~tracks.Valid; return; end
            count=min(double(tracks.Count),object.MaxTracks);
            for index=1:count
                if ~tracks.ValidMask(index); continue; end
                x=double(tracks.Positions(index,1));
                if x>0 && x<nearest && double(tracks.Velocities(index,1))<-1.0
                    nearest=x; lateral=double(tracks.Positions(index,2));
                end
            end
            if isfinite(nearest)
                if lateral>=0; side=int8(-1); else; side=int8(1); end
            end
        end

        function [time,positions,yaws,speeds,accelerations,curvatures,steering]= ...
                toWorldPlan(object,candidate,ego)
            time=candidate.Time; speeds=candidate.Speed;
            accelerations=candidate.Acceleration; curvatures=candidate.Curvature;
            steering=candidate.Steering;
            rotation=[cos(ego.Yaw) -sin(ego.Yaw);sin(ego.Yaw) cos(ego.Yaw)];
            world=double(candidate.Position)*rotation.'+double(ego.Position(1:2)).';
            positions=single(world);
            yaws=single(object.wrapAngle(double(candidate.Yaw)+ego.Yaw));
        end

        function [position,yaw,recommendedSpeed,segmentIndex,cumulativeStart]= ...
                referenceAtForward(~,reference,nref,distance,segmentIndex,cumulativeStart)
            if segmentIndex<2; segmentIndex=2; end
            while segmentIndex<=nref
                segment=double(reference.Positions(segmentIndex,:)- ...
                    reference.Positions(segmentIndex-1,:));
                segmentLength=norm(segment);
                if cumulativeStart+segmentLength>=distance || segmentLength<=1e-6
                    break
                end
                cumulativeStart=cumulativeStart+segmentLength;
                segmentIndex=segmentIndex+1;
            end
            if segmentIndex>nref
                segmentIndex=nref;
                position=double(reference.Positions(nref,:));
                yaw=double(reference.Yaws(nref));
                recommendedSpeed=double(reference.RecommendedSpeeds(nref));
                return
            end
            segment=double(reference.Positions(segmentIndex,:)- ...
                reference.Positions(segmentIndex-1,:));
            segmentLength=norm(segment);
            fraction=0;
            if segmentLength>1e-6
                fraction=min(max((distance-cumulativeStart)/segmentLength,0),1);
            end
            position=double(reference.Positions(segmentIndex-1,:))+fraction*segment;
            if segmentLength>1e-6
                yaw=atan2(segment(2),segment(1));
            else
                yaw=double(reference.Yaws(segmentIndex));
            end
            recommendedSpeed=(1-fraction)*double(reference.RecommendedSpeeds(segmentIndex-1))+ ...
                fraction*double(reference.RecommendedSpeeds(segmentIndex));
        end

        function valid=referenceIsUsable(~,reference,nref)
            valid=nref>=2 && all(reference.ValidMask(1:nref)) && ...
                all(isfinite(reference.Positions(1:nref,:)),"all") && ...
                all(isfinite(reference.RecommendedSpeeds(1:nref)));
            if valid
                differences=diff(double(reference.Positions(1:nref,:)),1,1);
                valid=sum(vecnorm(differences,2,2))>1.0;
            end
        end

        function valid=mapGeometryIsUsable(object,map)
            valid=isfinite(map.Resolution) && map.Resolution>0 && ...
                isequal(size(map.StaticOccupancy),[object.GridRows object.GridCols]) && ...
                map.XLimits(2)>map.XLimits(1) && map.YLimits(2)>map.YLimits(1);
        end

        function speed=longitudinalSpeed(~,ego)
            forward=[cos(ego.Yaw);sin(ego.Yaw)];
            speed=dot(double(ego.Velocity(1:2)),forward);
        end
        function acceleration=longitudinalAcceleration(~,ego)
            forward=[cos(ego.Yaw);sin(ego.Yaw)];
            acceleration=dot(double(ego.Acceleration(1:2)),forward);
        end
        function angle=wrapAngle(~,angle)
            angle=mod(angle+pi,2*pi)-pi;
        end
        function candidate=emptyCandidate(object)
            candidate=struct("Time",zeros(object.MaxPlanPoints,1,"single"), ...
                "Position",zeros(object.MaxPlanPoints,2,"single"), ...
                "Yaw",zeros(object.MaxPlanPoints,1,"single"), ...
                "Speed",zeros(object.MaxPlanPoints,1,"single"), ...
                "Acceleration",zeros(object.MaxPlanPoints,1,"single"), ...
                "Curvature",zeros(object.MaxPlanPoints,1,"single"), ...
                "Steering",zeros(object.MaxPlanPoints,1,"single"), ...
                "TargetOffset",single(0),"Valid",false);
        end
    end
end
