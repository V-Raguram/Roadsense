classdef RoadsenseBehaviourCommandSystem < matlab.System
    %ROADSENSEBEHAVIOURCOMMANDSYSTEM Package a selected mode for the planner.

    properties (Access = private)
        PreviousMode (1,1) uint8 = uint8(255)
        SequenceID (1,1) uint32 = uint32(0)
    end

    methods (Access = protected)
        function [timestamp,sequenceID,modeCode,targetSpeed,maximumAcceleration, ...
                minimumAcceleration,desiredClearance,stopDistance,leadTrackID, ...
                minimumTTC,forwardRisk,unknownFraction,reasonMask,replanRequested, ...
                emergencyRequested,outputValid] = stepImpl(object,timestampInput, ...
                modeCodeInput,inputsValid,cruiseTarget,followTarget,yieldTarget, ...
                creepTarget,cautiousTarget,avoidTarget,clearanceInput,stopDistanceInput, ...
                leadTrackIDInput,minimumTTCInput,forwardRiskInput,unknownFractionInput, ...
                reasonMaskInput)
            timestamp=timestampInput;
            modeCode=uint8(modeCodeInput);
            replanRequested=modeCode ~= object.PreviousMode;
            if replanRequested
                object.SequenceID=object.SequenceID+uint32(1);
                object.PreviousMode=modeCode;
            end
            sequenceID=object.SequenceID;
            minimumAcceleration=single(-3.0);
            switch modeCode
                case 1
                    targetSpeed=cruiseTarget; maximumAcceleration=single(2.0);
                case 2
                    targetSpeed=cautiousTarget; maximumAcceleration=single(1.2);
                case 3
                    targetSpeed=followTarget; maximumAcceleration=single(1.5);
                    minimumAcceleration=single(-3.5);
                case 4
                    targetSpeed=yieldTarget; maximumAcceleration=single(0.8);
                    minimumAcceleration=single(-4.0);
                case 5
                    targetSpeed=creepTarget; maximumAcceleration=single(0.8);
                    minimumAcceleration=single(-2.0);
                case {6,10}
                    targetSpeed=avoidTarget; maximumAcceleration=single(1.0);
                    minimumAcceleration=single(-3.0);
                case 7
                    targetSpeed=single(0); maximumAcceleration=single(0);
                case 8
                    targetSpeed=single(0); maximumAcceleration=single(0);
                    minimumAcceleration=single(-6.0);
                case 9
                    targetSpeed=single(0); maximumAcceleration=single(0);
                    minimumAcceleration=single(-3.5);
                otherwise
                    targetSpeed=single(0); maximumAcceleration=single(0);
            end
            if ~inputsValid && modeCode ~= uint8(9)
                targetSpeed=single(0);
            end
            desiredClearance=clearanceInput;
            stopDistance=stopDistanceInput;
            leadTrackID=leadTrackIDInput;
            minimumTTC=minimumTTCInput;
            forwardRisk=forwardRiskInput;
            unknownFraction=unknownFractionInput;
            reasonMask=reasonMaskInput;
            emergencyRequested=modeCode == uint8(8);
            outputValid=isfinite(timestampInput);
        end

        function resetImpl(object)
            object.PreviousMode=uint8(255);
            object.SequenceID=uint32(0);
        end
        function number=getNumInputsImpl(~); number=16; end
        function number=getNumOutputsImpl(~); number=16; end
        function names=getInputNamesImpl(~)
            names=["Timestamp","ModeCode","InputsValid","CruiseTarget","FollowTarget", ...
                "YieldTarget","CreepTarget","CautiousTarget","AvoidTarget", ...
                "DesiredClearance","StopDistance","LeadTrackID","MinimumTTC", ...
                "ForwardRisk","UnknownFraction","ReasonMask"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','SequenceID','ModeCode','TargetSpeed', ...
                'MaximumAcceleration','MinimumAcceleration','DesiredClearance', ...
                'StopDistance','LeadTrackID','MinimumTTC','ForwardRisk', ...
                'UnknownFraction','ReasonMask','ReplanRequested', ...
                'EmergencyRequested','OutputValid'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout=repmat({[1 1]},1,getNumOutputsImpl(object));
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','uint8','single','single','single','single', ...
                'single','uint32','single','single','single','uint16','logical', ...
                'logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nBehaviour Command";
        end
    end
end
