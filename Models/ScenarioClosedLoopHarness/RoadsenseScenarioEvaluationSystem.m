classdef RoadsenseScenarioEvaluationSystem < matlab.System
    %ROADSENSESCENARIOEVALUATIONSYSTEM Accumulate truth-isolated acceptance metrics.
    properties
        RequiredClearance (1,1) single = single(0.10)
        MaximumAllowedReplanLatency (1,1) single = single(0.10)
        MaximumAllowedPathSmoothness (1,1) single = single(0.18)
        MaximumAllowedJerk (1,1) single = single(10.0)
        MaximumAllowedCrossTrackError (1,1) single = single(3.0)
        MinimumReadyRatio (1,1) single = single(0.70)
    end
    properties (Access=private)
        Initialized logical = false
        LastScenarioID uint16 = uint16(0)
        MinimumClearanceState single = single(inf)
        CollisionState logical = false
        GoalState logical = false
        LastPlanID uint32 = uint32(0)
        ReplanCountState uint32 = uint32(0)
        LastReplanLatencyState single = single(0)
        MaximumReplanLatencyState single = single(0)
        PathSmoothnessState single = single(0)
        LastAcceleration single = single(0)
        LastControlTime double = 0
        HasControl logical = false
        MaximumJerkState single = single(0)
        MaximumCrossTrackState single = single(0)
        PreviousIntervention logical = false
        InterventionCountState uint32 = uint32(0)
        ReadyTicks uint32 = uint32(0)
        TotalTicks uint32 = uint32(0)
    end
    methods (Access=protected)
        function varargout=stepImpl(object,ego,localPlan,safeControl,safetyStatus, ...
                integrationStatus,plannerStatus,trackingStatus,dynamicsStatus, ...
                truthActors,scenarioStatus,sensorStatus)
            scenarioID=scenarioStatus.ScenarioID;
            if ~object.Initialized || scenarioID~=object.LastScenarioID
                object.resetScenario(scenarioID);
            end
            object.TotalTicks=object.TotalTicks+uint32(1);
            if integrationStatus.PipelineReady
                object.ReadyTicks=object.ReadyTicks+uint32(1);
            end

            [clearance,collisionNow]=object.truthClearance(ego,truthActors);
            if isfinite(clearance)
                object.MinimumClearanceState=min(object.MinimumClearanceState,single(clearance));
            end
            object.CollisionState=object.CollisionState || collisionNow;
            object.GoalState=object.GoalState || integrationStatus.GoalReached;

            if localPlan.Valid && localPlan.Count>0
                n=min(double(localPlan.Count),numel(localPlan.Curvatures));
                smoothness=sqrt(mean(double(localPlan.Curvatures(1:n)).^2));
                if isfinite(smoothness)
                    object.PathSmoothnessState=max(object.PathSmoothnessState,single(smoothness));
                end
            end
            if plannerStatus.PlanID~=uint32(0) && plannerStatus.PlanID~=object.LastPlanID
                object.LastPlanID=plannerStatus.PlanID;
                object.ReplanCountState=object.ReplanCountState+uint32(1);
                object.LastReplanLatencyState=plannerStatus.ExecutionTime;
                object.MaximumReplanLatencyState=max(object.MaximumReplanLatencyState, ...
                    plannerStatus.ExecutionTime);
            end
            % Startup fail-safe commands are intentionally excluded from
            % comfort and intervention metrics until the full pipeline is ready.
            nominalComfortSample=logical(safeControl.Valid && ...
                integrationStatus.PipelineReady && ~safetyStatus.OverrideActive && ...
                ~safeControl.EmergencyStop);
            if nominalComfortSample
                if object.HasControl
                    dt=max(safeControl.Timestamp-object.LastControlTime,0.02);
                    jerk=abs(double(safeControl.AccelerationCommand-object.LastAcceleration))/dt;
                    if isfinite(jerk); object.MaximumJerkState=max(object.MaximumJerkState,single(jerk)); end
                end
                object.LastAcceleration=safeControl.AccelerationCommand;
                object.LastControlTime=safeControl.Timestamp; object.HasControl=true;
            else
                % Safety interventions are evaluated by their own counter;
                % do not bridge their discontinuity into the comfort metric.
                object.HasControl=false;
            end
            if trackingStatus.Valid
                object.MaximumCrossTrackState=max(object.MaximumCrossTrackState, ...
                    abs(trackingStatus.CrossTrackError));
            end
            intervention=logical(integrationStatus.PipelineReady && ...
                (safetyStatus.OverrideActive || safeControl.EmergencyStop));
            if intervention && ~object.PreviousIntervention
                object.InterventionCountState=object.InterventionCountState+uint32(1);
            end
            object.PreviousIntervention=intervention;

            readyRatio=single(double(object.ReadyTicks)/max(double(object.TotalTicks),1));
            dropoutMask=uint8(sensorStatus.CameraDropout)+ ...
                bitshift(uint8(sensorStatus.LidarDropout),1)+ ...
                bitshift(uint8(sensorStatus.RadarDropout),2);
            finished=bitand(scenarioStatus.EventMask,uint16(128))~=0;
            valid=logical(ego.Valid && truthActors.Valid && scenarioStatus.Valid && ...
                sensorStatus.Valid && integrationStatus.Valid && dynamicsStatus.Valid);
            pass=logical(finished && valid && object.GoalState && ~object.CollisionState && ...
                object.MinimumClearanceState>=object.RequiredClearance && ...
                object.MaximumReplanLatencyState<=object.MaximumAllowedReplanLatency && ...
                object.PathSmoothnessState<=object.MaximumAllowedPathSmoothness && ...
                object.MaximumJerkState<=object.MaximumAllowedJerk && ...
                object.MaximumCrossTrackState<=object.MaximumAllowedCrossTrackError && ...
                readyRatio>=object.MinimumReadyRatio);
            varargout={ego.Timestamp,scenarioID,scenarioStatus.ElapsedTime, ...
                object.MinimumClearanceState,object.CollisionState,object.GoalState, ...
                finished,object.ReplanCountState,object.LastReplanLatencyState, ...
                object.MaximumReplanLatencyState,object.PathSmoothnessState, ...
                object.MaximumJerkState,object.MaximumCrossTrackState, ...
                object.InterventionCountState,readyRatio,dropoutMask,pass,valid};
        end
        function resetImpl(object); object.resetScenario(uint16(0)); object.Initialized=false; end
        function resetScenario(object,scenarioID)
            object.Initialized=true; object.LastScenarioID=scenarioID;
            object.MinimumClearanceState=single(inf); object.CollisionState=false;
            object.GoalState=false; object.LastPlanID=uint32(0); object.ReplanCountState=uint32(0);
            object.LastReplanLatencyState=single(0); object.MaximumReplanLatencyState=single(0);
            object.PathSmoothnessState=single(0); object.LastAcceleration=single(0);
            object.LastControlTime=0; object.HasControl=false; object.MaximumJerkState=single(0);
            object.MaximumCrossTrackState=single(0); object.PreviousIntervention=false;
            object.InterventionCountState=uint32(0); object.ReadyTicks=uint32(0);
            object.TotalTicks=uint32(0);
        end
        function [clearance,collision]=truthClearance(~,ego,truth)
            clearance=inf; collision=false;
            if ~ego.Valid || ~truth.Valid; return; end
            valid=find(truth.ValidMask);
            if isempty(valid); return; end
            yaw=double(ego.Yaw); c=cos(yaw); s=sin(yaw);
            delta=truth.Positions(valid,1:2)-double(ego.Position(1:2).');
            forward=c*delta(:,1)+s*delta(:,2); left=-s*delta(:,1)+c*delta(:,2);
            relativeYaw=double(truth.Yaws(valid))-yaw;
            actorLength=double(truth.Dimensions(valid,1)); actorWidth=double(truth.Dimensions(valid,2));
            halfLong=2.30+0.5*(abs(cos(relativeYaw)).*actorLength+abs(sin(relativeYaw)).*actorWidth);
            halfLat=0.95+0.5*(abs(sin(relativeYaw)).*actorLength+abs(cos(relativeYaw)).*actorWidth);
            gapLong=abs(forward)-halfLong; gapLat=abs(left)-halfLat;
            separations=zeros(size(gapLong));
            both=gapLong>0 & gapLat>0; separations(both)=hypot(gapLong(both),gapLat(both));
            longitudinalOnly=gapLong>0 & ~both; separations(longitudinalOnly)=gapLong(longitudinalOnly);
            lateralOnly=gapLat>0 & ~both; separations(lateralOnly)=gapLat(lateralOnly);
            overlap=gapLong<=0 & gapLat<=0; separations(overlap)=max(gapLong(overlap),gapLat(overlap));
            clearance=min(separations); collision=any(overlap);
        end
        function n=getNumInputsImpl(~); n=11; end
        function n=getNumOutputsImpl(~); n=18; end
        function names=getInputNamesImpl(~)
            names=["EgoState","LocalPlan","SafeControl","SafetyStatus", ...
                "IntegrationStatus","PlannerStatus","TrackingStatus","DynamicsStatus", ...
                "TruthActors","ScenarioStatus","SensorStatus"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','ScenarioID','ElapsedTime','MinimumClearance', ...
                'Collision','GoalReached','ScenarioFinished','ReplanCount', ...
                'LastReplanLatency','MaximumReplanLatency','PathSmoothness','MaximumJerk', ...
                'MaximumCrossTrackError','SafetyInterventionCount','PipelineReadyRatio', ...
                'SensorDropoutMask','Pass','Valid'};
        end
        function varargout=getOutputSizeImpl(~); varargout=repmat({[1 1]},1,18); end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint16','single','single','logical','logical', ...
                'logical','uint32','single','single','single','single','single', ...
                'uint32','single','uint8','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~); varargout=repmat({true},1,18); end
        function varargout=isOutputComplexImpl(~); varargout=repmat({false},1,18); end
        function icon=getIconImpl(~); icon="Truth-Isolated\nScenario Evaluation"; end
    end
end
