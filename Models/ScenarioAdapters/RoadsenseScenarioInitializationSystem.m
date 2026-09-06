classdef RoadsenseScenarioInitializationSystem < matlab.System
    %ROADSENSESCENARIOINITIALIZATIONSYSTEM Produce plant initialization and road state.
    properties (Access=private)
        Initialized logical = false
        LastScenarioID uint16 = uint16(0)
        PreviousResetRequest logical = false
    end
    methods (Access=protected)
        function [egoTimestamp,position,velocity,acceleration,yaw,pitch,roll,yawRate, ...
                steeringAngle,egoValid,roadTimestamp,friction,grade,bank, ...
                rollingResistance,roadValid,resetOut]=stepImpl(object,currentTime, ...
                scenario,resetRequest)
            egoTimestamp=currentTime; position=scenario.InitialPosition;
            yaw=scenario.InitialYaw; velocity=[double(scenario.InitialSpeed)*cos(yaw); ...
                double(scenario.InitialSpeed)*sin(yaw);0];
            acceleration=zeros(3,1); pitch=0; roll=0; yawRate=0; steeringAngle=0;
            egoValid=logical(scenario.Valid && isfinite(currentTime) && ...
                all(isfinite(position)) && isfinite(yaw) && isfinite(scenario.InitialSpeed));
            roadTimestamp=currentTime;
            friction=min(max(scenario.FrictionCoefficient,single(0.05)),single(1.5));
            grade=scenario.Grade; bank=scenario.Bank;
            rollingResistance=min(max(scenario.RollingResistance,single(0)),single(0.2));
            roadValid=logical(scenario.Valid && isfinite(currentTime) && ...
                isfinite(friction) && isfinite(grade) && isfinite(bank));
            scenarioChanged=logical(egoValid && (~object.Initialized || ...
                scenario.ScenarioID~=object.LastScenarioID));
            explicitPulse=logical(resetRequest && ~object.PreviousResetRequest);
            resetOut=logical(explicitPulse || scenarioChanged);
            object.PreviousResetRequest=logical(resetRequest);
            if egoValid
                object.Initialized=true; object.LastScenarioID=scenario.ScenarioID;
            end
        end
        function resetImpl(object)
            object.Initialized=false; object.LastScenarioID=uint16(0);
            object.PreviousResetRequest=false;
        end
        function number=getNumInputsImpl(~); number=3; end
        function number=getNumOutputsImpl(~); number=17; end
        function names=getInputNamesImpl(~)
            names=["CurrentTime","Scenario","ResetRequest"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'EgoTimestamp','Position','Velocity','Acceleration','Yaw', ...
                'Pitch','Roll','YawRate','SteeringAngle','EgoValid','RoadTimestamp', ...
                'FrictionCoefficient','Grade','Bank','RollingResistance','RoadValid','Reset'};
        end
        function varargout=getOutputSizeImpl(~)
            varargout={[1 1],[3 1],[3 1],[3 1],[1 1],[1 1],[1 1],[1 1], ...
                [1 1],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1],[1 1]};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','double','double','double','double','double', ...
                'double','double','double','logical','double','single','single', ...
                'single','single','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~); varargout=repmat({true},1,17); end
        function varargout=isOutputComplexImpl(~); varargout=repmat({false},1,17); end
        function icon=getIconImpl(~); icon="Roadsense\nScenario Init"; end
    end
end
