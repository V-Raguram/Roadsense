classdef RoadsenseSafetyCommandSystem < matlab.System
    %ROADSENSESAFETYCOMMANDSYSTEM Gate nominal actuation and publish status.
    properties
        MaximumSteering (1,1) single = single(0.55)
        EmergencyDeceleration (1,1) single = single(6.0)
        MinimalRiskDeceleration (1,1) single = single(3.0)
        DegradedSpeed (1,1) single = single(4.0)
    end
    properties (Access=private)
        SequenceID (1,1) uint32 = uint32(0)
        PreviousState (1,1) uint8 = uint8(0)
    end
    methods (Access=protected)
        function [timestamp,planID,steeringAngle,steeringRate,accelerationCommand, ...
                throttle,brake,targetSpeed,emergencyStop,controlValid,statusTimestamp, ...
                sequenceID,stateCodeOut,reasonMaskOut,minimumTTCOut,nearestDistanceOut, ...
                maximumInputAgeOut,crossTrackErrorOut,sideslipAngleOut, ...
                frictionUtilizationOut,overrideActive,emergencyLatchedOut,safeToDrive, ...
                statusValid] = stepImpl(object,nominal,timestampIn,stateCode,inputsValid, ...
                egoSpeed,minimumTTC,nearestDistance,maximumInputAge,crossTrackError, ...
                sideslipAngle,frictionUtilization,reasonMask,emergencyLatched)
            timestamp=timestampIn; planID=nominal.PlanID; stateCodeOut=uint8(stateCode);
            if stateCodeOut~=object.PreviousState
                object.SequenceID=object.SequenceID+uint32(1);
                object.PreviousState=stateCodeOut;
            end
            sequenceID=object.SequenceID;
            steeringAngle=min(max(nominal.SteeringAngle,-object.MaximumSteering), ...
                object.MaximumSteering);
            steeringRate=nominal.SteeringRate;
            accelerationCommand=nominal.AccelerationCommand;
            targetSpeed=nominal.TargetSpeed; emergencyStop=false;
            overrideActive=stateCodeOut~=uint8(1);
            switch stateCodeOut
                case uint8(1)
                    % Pass nominal control unchanged.
                case uint8(2)
                    targetSpeed=min(targetSpeed,object.DegradedSpeed);
                    % Degraded means reduced authority, not permanent
                    % immobilisation. Permit gentle propulsion below the
                    % degraded speed ceiling, hold at the ceiling, and brake
                    % when it is exceeded.
                    if egoSpeed>object.DegradedSpeed+single(0.10)
                        accelerationCommand=min(accelerationCommand,single(-1.5));
                    elseif egoSpeed<object.DegradedSpeed-single(0.20)
                        accelerationCommand=min(max(accelerationCommand,single(-1.5)),single(0.8));
                    else
                        accelerationCommand=min(accelerationCommand,single(0));
                    end
                case uint8(3)
                    accelerationCommand=-object.EmergencyDeceleration;
                    targetSpeed=single(0); emergencyStop=true;
                case uint8(4)
                    steeringAngle=single(0); steeringRate=single(0); targetSpeed=single(0);
                    if egoSpeed>single(0.10)
                        accelerationCommand=-object.MinimalRiskDeceleration;
                    else
                        accelerationCommand=single(0);
                    end
                    emergencyStop=true;
                otherwise
                    steeringAngle=single(0); steeringRate=single(0); targetSpeed=single(0);
                    accelerationCommand=single(0); emergencyStop=true;
            end
            throttle=single(max(0,double(accelerationCommand))/3.0);
            brake=single(max(0,-double(accelerationCommand))/ ...
                double(object.EmergencyDeceleration));
            throttle=min(throttle,single(1)); brake=min(brake,single(1));
            if stateCodeOut==uint8(5); brake=single(1); end
            controlValid=isfinite(timestamp);
            statusTimestamp=timestamp; reasonMaskOut=reasonMask;
            minimumTTCOut=minimumTTC; nearestDistanceOut=nearestDistance;
            maximumInputAgeOut=maximumInputAge; crossTrackErrorOut=crossTrackError;
            sideslipAngleOut=sideslipAngle; frictionUtilizationOut=frictionUtilization;
            emergencyLatchedOut=emergencyLatched;
            safeToDrive=logical(inputsValid && (stateCodeOut==uint8(1) || stateCodeOut==uint8(2)));
            statusValid=isfinite(timestamp);
        end
        function resetImpl(object)
            object.SequenceID=uint32(0); object.PreviousState=uint8(0);
        end
        function number=getNumInputsImpl(~); number=13; end
        function number=getNumOutputsImpl(~); number=24; end
        function names=getInputNamesImpl(~)
            names=["NominalControl","Timestamp","StateCode","InputsValid","EgoSpeed", ...
                "MinimumTTC","NearestDistance","MaximumInputAge","CrossTrackError", ...
                "SideslipAngle","FrictionUtilization","ReasonMask","EmergencyLatched"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','PlanID','SteeringAngle','SteeringRate', ...
                'AccelerationCommand','Throttle','Brake','TargetSpeed','EmergencyStop', ...
                'ControlValid','StatusTimestamp','SequenceID','StateCode','ReasonMask', ...
                'MinimumTTC','NearestDistance','MaximumInputAge','CrossTrackError', ...
                'SideslipAngle','FrictionUtilization','OverrideActive','EmergencyLatchedOut', ...
                'SafeToDrive','StatusValid'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','single','single','single','single','single', ...
                'single','logical','logical','double','uint32','uint8','uint16','single', ...
                'single','single','single','single','single','logical','logical','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nSafety Command\nGate";
        end
    end
end
