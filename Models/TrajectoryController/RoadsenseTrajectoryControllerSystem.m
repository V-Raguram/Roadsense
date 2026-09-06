classdef RoadsenseTrajectoryControllerSystem < matlab.System
    %ROADSENSETRAJECTORYCONTROLLERSYSTEM Track a local-world trajectory.

    properties (Nontunable)
        MaxPlanPoints (1,1) double = 61
        SampleTime (1,1) double = 0.02
    end

    properties
        Wheelbase (1,1) single = single(2.8)
        LateralGain (1,1) single = single(2.2)
        HeadingGain (1,1) single = single(1.2)
        SofteningSpeed (1,1) single = single(2.0)
        SpeedKp (1,1) single = single(0.80)
        SpeedKi (1,1) single = single(0.15)
        IntegralLimit (1,1) single = single(3.0)
        MaximumSteering (1,1) single = single(0.55)
        MaximumSteeringRate (1,1) single = single(0.70)
        MaximumAcceleration (1,1) single = single(3.0)
        MaximumDeceleration (1,1) single = single(6.0)
        AccelerationSlew (1,1) single = single(5.0)
        PreviewTime (1,1) single = single(1.0)
        PlanGrace (1,1) single = single(0.25)
    end

    properties (Access=private)
        PreviousSteering (1,1) single = single(0)
        PreviousAcceleration (1,1) single = single(0)
        SpeedErrorIntegral (1,1) single = single(0)
        PreviousTimestamp (1,1) double = -inf
        PreviousPlanID (1,1) uint32 = uint32(0)
        Initialized (1,1) logical = false
    end

    methods (Access=protected)
        function [timestamp,planID,steeringAngle,steeringRate,accelerationCommand, ...
                throttle,brake,targetSpeed,emergencyStop,commandValid, ...
                statusTimestamp,statusPlanID,referenceIndex,crossTrackError, ...
                headingError,speedError,referenceCurvature,referenceAge, ...
                steeringSaturated,accelerationSaturated,planExpired,statusValid] = ...
                stepImpl(object,ego,plan,plannerStatus)
            timestamp=ego.Timestamp; planID=plan.PlanID;
            steeringAngle=single(0); steeringRate=single(0); %#ok<NASGU>
            accelerationCommand=-object.MaximumDeceleration; %#ok<NASGU>
            throttle=single(0); brake=single(1); targetSpeed=single(0);
            emergencyStop=true; commandValid=false;
            statusTimestamp=timestamp; statusPlanID=planID; referenceIndex=uint16(0);
            crossTrackError=single(0); headingError=single(0); speedError=single(0);
            referenceCurvature=single(0); referenceAge=single(0);
            steeringSaturated=false; accelerationSaturated=false; %#ok<NASGU>
            planExpired=false; statusValid=false; %#ok<NASGU>

            deltaTime=object.controlDeltaTime(timestamp);
            if ~object.Initialized
                if isfinite(ego.SteeringAngle)
                    object.PreviousSteering=single(ego.SteeringAngle);
                end
                object.PreviousAcceleration=single(0);
                object.Initialized=true;
            end

            count=min(double(plan.Count),object.MaxPlanPoints);
            age=timestamp-plan.Timestamp;
            if isfinite(age); referenceAge=single(max(age,0)); end
            if count>=1
                finalTime=double(plan.TimeFromStart(count));
            else
                finalTime=0;
            end
            planExpired=logical(isfinite(age) && age>finalTime+double(object.PlanGrace));
            plannerCode=uint8(plannerStatus.Code);
            idsMatch=plannerStatus.PlanID==plan.PlanID;
            criticalValid=logical(ego.Valid && plan.Valid && count>=2 && ...
                isfinite(timestamp) && isfinite(plan.Timestamp) && age>=-0.10 && ...
                ~planExpired && idsMatch && plannerCode~=uint8(0) && plannerCode~=uint8(4));

            if criticalValid
                if planID~=object.PreviousPlanID
                    object.SpeedErrorIntegral=single(0);
                    object.PreviousPlanID=planID;
                end
                [nearestIndex,targetIndex]=object.selectReferences(ego,plan,count,max(age,0));
                referenceIndex=uint16(targetIndex);
                [crossTrackError,headingError]=object.lateralErrors(ego,plan,nearestIndex,targetIndex);
                referenceCurvature=plan.Curvatures(targetIndex);
                targetSpeed=max(single(0),plan.Speeds(targetIndex));
                egoSpeed=object.longitudinalSpeed(ego);
                speedError=targetSpeed-single(egoSpeed);

                feedforward=atan(double(object.Wheelbase)*double(referenceCurvature));
                feedback=double(object.HeadingGain)*double(headingError)- ...
                    atan(double(object.LateralGain)*double(crossTrackError)/ ...
                    (max(egoSpeed,0)+double(object.SofteningSpeed)));
                rawSteering=feedforward+feedback;
                boundedSteering=min(max(rawSteering,-double(object.MaximumSteering)), ...
                    double(object.MaximumSteering));
                maximumSteeringDelta=double(object.MaximumSteeringRate)*deltaTime;
                rateLimitedSteering=min(max(boundedSteering, ...
                    double(object.PreviousSteering)-maximumSteeringDelta), ...
                    double(object.PreviousSteering)+maximumSteeringDelta);
                steeringAngle=single(rateLimitedSteering);
                steeringRate=single((rateLimitedSteering-double(object.PreviousSteering))/deltaTime);
                steeringSaturated=logical(abs(rawSteering-boundedSteering)>1e-6 || ...
                    abs(boundedSteering-rateLimitedSteering)>1e-6);

                candidateIntegral=min(max(double(object.SpeedErrorIntegral)+ ...
                    double(speedError)*deltaTime,-double(object.IntegralLimit)), ...
                    double(object.IntegralLimit));
                rawAcceleration=double(plan.Accelerations(targetIndex))+ ...
                    double(object.SpeedKp)*double(speedError)+ ...
                    double(object.SpeedKi)*candidateIntegral;
                boundedAcceleration=min(max(rawAcceleration,-double(object.MaximumDeceleration)), ...
                    double(object.MaximumAcceleration));
                if abs(rawAcceleration-boundedAcceleration)<1e-6
                    object.SpeedErrorIntegral=single(candidateIntegral);
                end

                % Emergency authority belongs to the status record, which
                % carries the plan identifier and the local planner decision.
                % The plan and status traverse independent rate transitions;
                % an old plan's EmergencyFallback flag may coexist briefly
                % with a newer safe status.  Using that stale field caused
                % unnecessary full-brake pulses in the closed-loop harness.
                emergencyStop=logical(plannerStatus.EmergencyRequested || ...
                    plannerCode==uint8(6));
                if emergencyStop
                    accelerationCommand=-object.MaximumDeceleration;
                    accelerationSaturated=true;
                else
                    maximumAccelerationDelta=double(object.AccelerationSlew)*deltaTime;
                    limitedAcceleration=min(max(boundedAcceleration, ...
                        double(object.PreviousAcceleration)-maximumAccelerationDelta), ...
                        double(object.PreviousAcceleration)+maximumAccelerationDelta);
                    accelerationCommand=single(limitedAcceleration);
                    accelerationSaturated=logical(abs(rawAcceleration-boundedAcceleration)>1e-6 || ...
                        abs(boundedAcceleration-limitedAcceleration)>1e-6);
                end
                throttle=single(max(0,double(accelerationCommand))/ ...
                    max(double(object.MaximumAcceleration),eps));
                brake=single(max(0,-double(accelerationCommand))/ ...
                    max(double(object.MaximumDeceleration),eps));
                throttle=min(throttle,single(1)); brake=min(brake,single(1));
                commandValid=true; statusValid=true;
            else
                [steeringAngle,steeringRate]=object.failSafeSteering(deltaTime);
                accelerationCommand=-object.MaximumDeceleration;
                accelerationSaturated=true;
                object.SpeedErrorIntegral=single(0);
            end

            object.PreviousSteering=steeringAngle;
            object.PreviousAcceleration=accelerationCommand;
            object.PreviousTimestamp=timestamp;
        end

        function resetImpl(object)
            object.PreviousSteering=single(0); object.PreviousAcceleration=single(0);
            object.SpeedErrorIntegral=single(0); object.PreviousTimestamp=-inf;
            object.PreviousPlanID=uint32(0); object.Initialized=false;
        end

        function number=getNumInputsImpl(~); number=3; end
        function number=getNumOutputsImpl(~); number=22; end
        function names=getInputNamesImpl(~)
            names=["EgoState","LocalPlan","PlannerStatus"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','PlanID','SteeringAngle','SteeringRate', ...
                'AccelerationCommand','Throttle','Brake','TargetSpeed', ...
                'EmergencyStop','CommandValid','StatusTimestamp','StatusPlanID', ...
                'ReferenceIndex','CrossTrackError','HeadingError','SpeedError', ...
                'ReferenceCurvature','ReferenceAge','SteeringSaturated', ...
                'AccelerationSaturated','PlanExpired','StatusValid'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','single','single','single','single', ...
                'single','single','logical','logical','double','uint32','uint16', ...
                'single','single','single','single','single','logical','logical', ...
                'logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nTrajectory Controller\nStanley + PI";
        end
    end

    methods (Access=private)
        function deltaTime=controlDeltaTime(object,timestamp)
            deltaTime=object.SampleTime;
            if isfinite(object.PreviousTimestamp) && isfinite(timestamp)
                measured=timestamp-object.PreviousTimestamp;
                if measured>0 && measured<=0.2
                    deltaTime=measured;
                end
            end
        end

        function [nearestIndex,targetIndex]=selectReferences(object,ego,plan,count,age)
            nearestIndex=1; minimumDistance=inf;
            for index=1:count
                delta=double(plan.Positions(index,:))-double(ego.Position(1:2)).';
                distanceSquared=delta(1)^2+delta(2)^2;
                if distanceSquared<minimumDistance
                    minimumDistance=distanceSquared; nearestIndex=index;
                end
            end
            previewTime=age+double(object.PreviewTime); timeIndex=count;
            for index=1:count
                if double(plan.TimeFromStart(index))>=previewTime
                    timeIndex=index; break
                end
            end
            targetIndex=max(nearestIndex,timeIndex);
        end

        function [crossTrack,heading]=lateralErrors(object,ego,plan,nearestIndex,targetIndex)
            referencePosition=double(plan.Positions(nearestIndex,:));
            referenceYaw=double(plan.Yaws(nearestIndex));
            displacement=double(ego.Position(1:2)).'-referencePosition;
            crossTrack=single(-sin(referenceYaw)*displacement(1)+ ...
                cos(referenceYaw)*displacement(2));
            heading=single(object.wrapAngle(double(plan.Yaws(targetIndex))-ego.Yaw));
        end

        function speed=longitudinalSpeed(~,ego)
            speed=dot(double(ego.Velocity(1:2)),[cos(ego.Yaw);sin(ego.Yaw)]);
        end

        function [steering,rate]=failSafeSteering(object,deltaTime)
            maximumDelta=double(object.MaximumSteeringRate)*deltaTime;
            previous=double(object.PreviousSteering);
            steering=single(min(max(0,previous-maximumDelta),previous+maximumDelta));
            rate=single((double(steering)-previous)/deltaTime);
        end

        function angle=wrapAngle(~,angle)
            angle=mod(angle+pi,2*pi)-pi;
        end
    end
end
