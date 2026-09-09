classdef RoadsenseSafetyAssessmentSystem < matlab.System
    %ROADSENSESAFETYASSESSMENTSYSTEM Independent cross-subsystem safety checks.
    properties (Nontunable)
        MaxTracks (1,1) double = 128
        GridRows (1,1) double = 200
        GridCols (1,1) double = 320
        GridResolution (1,1) single = single(0.25)
        GridXLimits (2,1) single = single([-20;60])
        GridYLimits (2,1) single = single([-25;25])
    end
    properties
        EmergencyTTC (1,1) single = single(1.0)
        EmergencyDistance (1,1) single = single(2.5)
        ControlStaleTime (1,1) single = single(0.10)
        TrackingStaleTime (1,1) single = single(0.10)
        PerceptionStaleTime (1,1) single = single(0.25)
        PlanningStaleTime (1,1) single = single(0.30)
        CrossTrackWarning (1,1) single = single(0.75)
        CrossTrackFault (1,1) single = single(2.5)
        HeadingWarning (1,1) single = single(0.20)
        HeadingFault (1,1) single = single(0.60)
        SideslipWarning (1,1) single = single(0.10)
        SideslipFault (1,1) single = single(0.18)
        FrictionWarning (1,1) single = single(0.85)
        LateralAccelerationFault (1,1) single = single(7.0)
    end
    properties (Access=private)
        CloseRows
        CloseColumns
    end
    methods (Access=protected)
        function setupImpl(object)
            x=double(object.GridXLimits(1))+(0.5:object.GridCols-0.5)*double(object.GridResolution);
            y=double(object.GridYLimits(1))+(0.5:object.GridRows-0.5)*double(object.GridResolution);
            object.CloseColumns=x>=0 & x<=8;
            object.CloseRows=abs(y)<=1.5;
        end
        function [timestamp,inputsValid,criticalFault,emergencyHazard, ...
                degradedCondition,egoMoving,egoSpeed,minimumTTC,nearestDistance, ...
                maximumInputAge,crossTrackError,sideslipAngle,frictionUtilization, ...
                reasonMask] = stepImpl(object,ego,map,tracks,behaviour,planner, ...
                tracking,dynamics,control)
            timestamp=ego.Timestamp; egoSpeed=single(norm(ego.Velocity(1:2)));
            egoMoving=egoSpeed>single(0.20); minimumTTC=single(inf);
            nearestDistance=single(inf); maximumInputAge=single(inf);
            crossTrackError=tracking.CrossTrackError;
            sideslipAngle=dynamics.SideslipAngle;
            frictionUtilization=dynamics.FrictionUtilization;
            plannerCode=uint8(planner.Code);
            inputsValid=logical(ego.Valid && map.Valid && tracks.Valid && behaviour.Valid && ...
                tracking.Valid && dynamics.Valid && control.Valid && isfinite(timestamp) && ...
                plannerCode~=uint8(0) && plannerCode~=uint8(4));

            ages=[timestamp-map.Timestamp timestamp-tracks.Timestamp ...
                timestamp-behaviour.Timestamp timestamp-planner.Timestamp ...
                timestamp-tracking.Timestamp timestamp-dynamics.Timestamp ...
                timestamp-control.Timestamp];
            if all(isfinite(ages))
                maximumInputAge=single(max(max(ages),0));
            end
            futureData=any(ages < -0.05);
            staleData=logical(futureData || ages(1)>double(object.PerceptionStaleTime) || ...
                ages(2)>double(object.PerceptionStaleTime) || ...
                ages(3)>double(object.PlanningStaleTime) || ...
                ages(4)>double(object.PlanningStaleTime) || ...
                ages(5)>double(object.TrackingStaleTime) || ...
                ages(6)>double(object.TrackingStaleTime) || ...
                ages(7)>double(object.ControlStaleTime));

            closeMapRisk=single(0);
            if map.Valid
                closeMapRisk=max(map.Occupancy(object.CloseRows,object.CloseColumns),[],"all");
            end
            if tracks.Valid
                count=min(double(tracks.Count),object.MaxTracks);
                for index=1:count
                    if ~tracks.ValidMask(index); continue; end
                    xPosition=double(tracks.Positions(index,1));
                    yPosition=double(tracks.Positions(index,2));
                    dimensions=max(double(tracks.Dimensions(index,1:2)),[0.5 0.5]);
                    if xPosition>0 && abs(yPosition)<=1.2+dimensions(2)/2
                        clearance=max(0,xPosition-2.3-dimensions(1)/2);
                        nearestDistance=min(nearestDistance,single(clearance));
                        closingSpeed=-double(tracks.Velocities(index,1));
                        if closingSpeed>0.1
                            minimumTTC=min(minimumTTC,single(clearance/closingSpeed));
                        end
                    end
                end
            end
            emergencyRequested=logical(behaviour.EmergencyRequested || ...
                planner.EmergencyRequested || control.EmergencyStop);
            imminentCollision=logical(minimumTTC<object.EmergencyTTC || ...
                nearestDistance<object.EmergencyDistance || closeMapRisk>=single(0.95));
            emergencyHazard=logical(imminentCollision || emergencyRequested);
            % Cross-track, sideslip and tyre-force measures describe motion
            % stability. At standstill they can retain the last steering or
            % force state and must not prevent a safe restart after a hazard.
            trackingFault=logical(tracking.PlanExpired || (egoMoving && ( ...
                abs(tracking.CrossTrackError)>single(2)*object.CrossTrackFault || ...
                abs(tracking.HeadingError)>single(2)*object.HeadingFault)));
            trackingConcern=logical(trackingFault || (egoMoving && ( ...
                abs(tracking.CrossTrackError)>object.CrossTrackFault || ...
                abs(tracking.HeadingError)>object.HeadingFault || ...
                abs(tracking.CrossTrackError)>object.CrossTrackWarning || ...
                abs(tracking.HeadingError)>object.HeadingWarning)));
            vehicleInstability=logical(egoMoving && ( ...
                abs(dynamics.SideslipAngle)>object.SideslipFault || ...
                abs(dynamics.LateralAcceleration)>object.LateralAccelerationFault || ...
                dynamics.FrictionUtilization>single(1.05)));
            vehicleConcern=logical(egoMoving && (vehicleInstability || ...
                abs(dynamics.SideslipAngle)>object.SideslipWarning || ...
                dynamics.FrictionUtilization>object.FrictionWarning));
            % Reaching the configured longitudinal command limit is normal
            % during launch and braking; the command is already bounded by
            % the controller. Steering or tire-force saturation can erode
            % lateral authority and therefore remains a degraded condition.
            actuatorWarning=logical(egoMoving && (tracking.SteeringSaturated || ...
                dynamics.TireForceSaturated));
            deadlineWarning=plannerCode==uint8(5);
            highRisk=behaviour.ForwardRisk>=single(0.65);
            degradedCondition=logical((egoMoving && ( ...
                abs(tracking.CrossTrackError)>object.CrossTrackWarning || ...
                abs(tracking.HeadingError)>object.HeadingWarning || ...
                abs(dynamics.SideslipAngle)>object.SideslipWarning || ...
                dynamics.FrictionUtilization>object.FrictionWarning)) || actuatorWarning || ...
                deadlineWarning || highRisk);
            criticalFault=logical(~inputsValid || staleData || trackingFault || vehicleInstability);
            reasonMask=uint16(0);
            if ~inputsValid; reasonMask=bitor(reasonMask,uint16(1)); end
            if staleData; reasonMask=bitor(reasonMask,uint16(2)); end
            if imminentCollision; reasonMask=bitor(reasonMask,uint16(4)); end
            if emergencyRequested; reasonMask=bitor(reasonMask,uint16(8)); end
            if trackingConcern; reasonMask=bitor(reasonMask,uint16(16)); end
            if vehicleConcern; reasonMask=bitor(reasonMask,uint16(32)); end
            if actuatorWarning; reasonMask=bitor(reasonMask,uint16(64)); end
            if deadlineWarning; reasonMask=bitor(reasonMask,uint16(128)); end
            if highRisk; reasonMask=bitor(reasonMask,uint16(256)); end
        end
        function number=getNumInputsImpl(~); number=8; end
        function number=getNumOutputsImpl(~); number=14; end
        function names=getInputNamesImpl(~)
            names=["EgoState","SemanticGrid","FusedTracks","BehaviourCommand", ...
                "PlannerStatus","TrackingStatus","DynamicsStatus","NominalControl"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','InputsValid','CriticalFault','EmergencyHazard', ...
                'DegradedCondition','EgoMoving','EgoSpeed','MinimumTTC', ...
                'NearestDistance','MaximumInputAge','CrossTrackError','SideslipAngle', ...
                'FrictionUtilization','ReasonMask'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','logical','logical','logical','logical','logical', ...
                'single','single','single','single','single','single','single','uint16'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nIndependent Safety\nAssessment";
        end
    end
end
