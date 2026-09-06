classdef RoadsenseIntegrationMonitorSystem < matlab.System
    %ROADSENSEINTEGRATIONMONITORSYSTEM Summarize complete pipeline health.
    properties (Access=private)
        CycleID uint32 = uint32(0)
    end
    methods (Access=protected)
        function [timestamp,cycleID,perceptionValid,fusionValid,predictionValid, ...
                mapValid,behaviourValid,planValid,trackingValid,safetyValid, ...
                vehicleValid,pipelineReady,safetyOverride,emergencyStop, ...
                goalReached,statusValid] = stepImpl(object,ego,imageSemantic, ...
                lidarGrid,fusedTracks,predictions,semanticGrid,behaviour,localPlan, ...
                plannerStatus,nominalControl,trackingStatus,safetyStatus, ...
                safeControl,dynamicsStatus,routeContext)
            object.CycleID=object.CycleID+uint32(1);
            timestamp=ego.Timestamp;
            cycleID=object.CycleID;
            perceptionValid=logical(imageSemantic.Valid && lidarGrid.Valid);
            fusionValid=logical(fusedTracks.Valid);
            predictionValid=logical(predictions.Valid);
            mapValid=logical(semanticGrid.Valid);
            behaviourValid=logical(behaviour.Valid);
            plannerCode=uint8(plannerStatus.Code);
            planValid=logical(localPlan.Valid && plannerCode~=uint8(0) && ...
                plannerCode~=uint8(4));
            trackingValid=logical(nominalControl.Valid && trackingStatus.Valid);
            safetyValid=logical(safetyStatus.Valid && safeControl.Valid);
            vehicleValid=logical(ego.Valid && dynamicsStatus.Valid);
            pipelineReady=logical(perceptionValid && fusionValid && ...
                predictionValid && mapValid && behaviourValid && planValid && ...
                trackingValid && safetyValid && vehicleValid && routeContext.Valid);
            safetyOverride=logical(safetyStatus.OverrideActive);
            emergencyStop=logical(safeControl.EmergencyStop);
            goalReached=logical(routeContext.Valid && routeContext.GoalReached);
            statusValid=isfinite(timestamp);
        end

        function resetImpl(object)
            object.CycleID=uint32(0);
        end

        function number=getNumInputsImpl(~); number=15; end
        function number=getNumOutputsImpl(~); number=16; end
        function names=getInputNamesImpl(~)
            names=["EgoState","ImageSemantic","LidarGrid","FusedTracks", ...
                "Predictions","SemanticGrid","BehaviourCommand","LocalPlan", ...
                "PlannerStatus","NominalControl","TrackingStatus","SafetyStatus", ...
                "SafeControl","DynamicsStatus","RouteContext"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','CycleID','PerceptionValid','FusionValid', ...
                'PredictionValid','MapValid','BehaviourValid','PlanValid', ...
                'TrackingValid','SafetyValid','VehicleValid','PipelineReady', ...
                'SafetyOverride','EmergencyStop','GoalReached','Valid'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','logical','logical','logical','logical', ...
                'logical','logical','logical','logical','logical','logical', ...
                'logical','logical','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nIntegration Health";
        end
    end
end
