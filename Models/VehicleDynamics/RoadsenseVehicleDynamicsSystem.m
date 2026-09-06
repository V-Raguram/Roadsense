classdef RoadsenseVehicleDynamicsSystem < matlab.System
    %ROADSENSEVEHICLEDYNAMICSSYSTEM Nonlinear planar dynamic bicycle plant.

    properties (Nontunable)
        SampleTime (1,1) double = 0.02
    end

    properties
        Mass (1,1) single = single(1650)
        YawInertia (1,1) single = single(2800)
        FrontAxleDistance (1,1) single = single(1.2)
        RearAxleDistance (1,1) single = single(1.6)
        FrontCorneringStiffness (1,1) single = single(80000)
        RearCorneringStiffness (1,1) single = single(85000)
        SteeringTimeConstant (1,1) single = single(0.08)
        AccelerationTimeConstant (1,1) single = single(0.15)
        MaximumSteering (1,1) single = single(0.55)
        MaximumSteeringRate (1,1) single = single(1.0)
        MaximumAcceleration (1,1) single = single(3.0)
        MaximumDeceleration (1,1) single = single(6.0)
        MaximumJerk (1,1) single = single(10.0)
        LowSpeedThreshold (1,1) single = single(2.0)
        MaximumSpeed (1,1) single = single(50.0)
        Gravity (1,1) single = single(9.81)
    end

    properties (Access=private)
        X (1,1) double = 0
        Y (1,1) double = 0
        Z (1,1) double = 0
        YawState (1,1) double = 0
        LongitudinalSpeedState (1,1) double = 0
        LateralSpeedState (1,1) double = 0
        YawRateState (1,1) double = 0
        SteeringStateInternal (1,1) double = 0
        AccelerationState (1,1) double = 0
        StateTimestamp (1,1) double = 0
        Initialized (1,1) logical = false
    end

    methods (Access=protected)
        function [timestamp,position,velocity,acceleration,yaw,pitch,roll,yawRate, ...
                steeringAngle,egoValid,statusTimestamp,longitudinalSpeed,lateralSpeed, ...
                sideslipAngle,frontSlipAngle,rearSlipAngle,yawAcceleration, ...
                longitudinalAcceleration,lateralAcceleration,frictionUtilization, ...
                steeringState,resetApplied,tireForceSaturated,statusValid] = ...
                stepImpl(object,control,initialState,road,reset)
            resetApplied=logical(reset || ~object.Initialized);
            if resetApplied
                object.loadInitialState(initialState);
            end

            frontSlip=0; rearSlip=0; yawAccelerationValue=0;
            longitudinalAccelerationValue=0; lateralAccelerationValue=0;
            frictionUtilizationValue=0; tireForceSaturated=false;
            roadValid=logical(road.Valid && isfinite(road.FrictionCoefficient) && ...
                isfinite(road.Grade) && isfinite(road.Bank) && isfinite(road.RollingResistance));

            if ~resetApplied && object.Initialized
                [frontSlip,rearSlip,yawAccelerationValue, ...
                    longitudinalAccelerationValue,lateralAccelerationValue, ...
                    frictionUtilizationValue,tireForceSaturated]=object.integrate(control,road,roadValid);
            end

            % The plant owns simulation-state time.  Control timestamps can
            % lag while a command is held between controller task hits, so
            % copying them would freeze ego time and falsely stale the stack.
            if ~resetApplied && object.Initialized
                nextTimestamp=object.StateTimestamp+object.SampleTime;
                if isfinite(control.Timestamp)
                    nextTimestamp=max(nextTimestamp,control.Timestamp);
                end
                object.StateTimestamp=nextTimestamp;
            elseif object.Initialized && isfinite(control.Timestamp)
                object.StateTimestamp=max(object.StateTimestamp,control.Timestamp);
            end
            timestamp=object.StateTimestamp; statusTimestamp=timestamp;
            yaw=object.YawState; pitch=double(road.Grade); roll=double(road.Bank);
            yawRate=object.YawRateState; steeringAngle=object.SteeringStateInternal;
            position=[object.X;object.Y;object.Z];
            rotation=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
            worldVelocity=rotation*[object.LongitudinalSpeedState;object.LateralSpeedState];
            velocity=[worldVelocity;0];
            worldAcceleration=rotation*[longitudinalAccelerationValue;lateralAccelerationValue];
            acceleration=[worldAcceleration;0];
            longitudinalSpeed=single(object.LongitudinalSpeedState);
            lateralSpeed=single(object.LateralSpeedState);
            sideslipAngle=single(atan2(object.LateralSpeedState, ...
                max(abs(object.LongitudinalSpeedState),0.1)));
            frontSlipAngle=single(frontSlip); rearSlipAngle=single(rearSlip);
            yawAcceleration=single(yawAccelerationValue);
            longitudinalAcceleration=single(longitudinalAccelerationValue);
            lateralAcceleration=single(lateralAccelerationValue);
            frictionUtilization=single(frictionUtilizationValue);
            steeringState=single(object.SteeringStateInternal);
            finiteState=all(isfinite([position;velocity;acceleration;yaw;yawRate;steeringAngle]));
            egoValid=logical(object.Initialized && finiteState);
            statusValid=logical(egoValid && roadValid);
        end

        function resetImpl(object)
            object.X=0; object.Y=0; object.Z=0; object.YawState=0;
            object.LongitudinalSpeedState=0; object.LateralSpeedState=0;
            object.YawRateState=0; object.SteeringStateInternal=0;
            object.AccelerationState=0; object.StateTimestamp=0; object.Initialized=false;
        end

        function number=getNumInputsImpl(~); number=4; end
        function number=getNumOutputsImpl(~); number=24; end
        function names=getInputNamesImpl(~)
            names=["VehicleControl","InitialState","RoadCondition","Reset"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','Position','Velocity','Acceleration','Yaw','Pitch', ...
                'Roll','YawRate','SteeringAngle','EgoValid','StatusTimestamp', ...
                'LongitudinalSpeed','LateralSpeed','SideslipAngle','FrontSlipAngle', ...
                'RearSlipAngle','YawAcceleration','LongitudinalAcceleration', ...
                'LateralAcceleration','FrictionUtilization','SteeringState', ...
                'ResetApplied','TireForceSaturated','StatusValid'};
        end
        function varargout=getOutputSizeImpl(~)
            one=[1 1]; vector=[3 1];
            varargout={one,vector,vector,vector,one,one,one,one,one,one,one, ...
                one,one,one,one,one,one,one,one,one,one,one,one,one};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','double','double','double','double','double', ...
                'double','double','double','logical','double','single','single', ...
                'single','single','single','single','single','single','single', ...
                'single','logical','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(object)
            varargout=repmat({true},1,getNumOutputsImpl(object));
        end
        function varargout=isOutputComplexImpl(object)
            varargout=repmat({false},1,getNumOutputsImpl(object));
        end
        function icon=getIconImpl(~)
            icon="Roadsense\nDynamic Bicycle Plant\nTyre + Road Physics";
        end
    end

    methods (Access=private)
        function loadInitialState(object,initialState)
            if initialState.Valid && all(isfinite([initialState.Position; ...
                    initialState.Velocity;initialState.Acceleration;initialState.Yaw; ...
                    initialState.YawRate;initialState.SteeringAngle]))
                object.X=initialState.Position(1); object.Y=initialState.Position(2);
                object.Z=initialState.Position(3); object.YawState=initialState.Yaw;
                rotation=[cos(object.YawState) sin(object.YawState); ...
                    -sin(object.YawState) cos(object.YawState)];
                bodyVelocity=rotation*initialState.Velocity(1:2);
                bodyAcceleration=rotation*initialState.Acceleration(1:2);
                object.LongitudinalSpeedState=max(0,bodyVelocity(1));
                object.LateralSpeedState=bodyVelocity(2);
                object.YawRateState=initialState.YawRate;
                object.SteeringStateInternal=min(max(initialState.SteeringAngle, ...
                    -double(object.MaximumSteering)),double(object.MaximumSteering));
                object.AccelerationState=bodyAcceleration(1);
                object.StateTimestamp=initialState.Timestamp;
                object.Initialized=true;
            else
                object.Initialized=false;
            end
        end

        function [frontSlip,rearSlip,yawAcceleration,longitudinalAcceleration, ...
                lateralAcceleration,frictionUtilization,saturated]= ...
                integrate(object,control,road,roadValid)
            dt=object.SampleTime; gravity=double(object.Gravity);
            if roadValid
                friction=min(max(double(road.FrictionCoefficient),0.10),1.30);
                grade=double(road.Grade); bank=double(road.Bank);
                rolling=max(0,double(road.RollingResistance));
            else
                friction=0.60; grade=0; bank=0; rolling=0.02;
            end
            commandFinite=isfinite(control.SteeringAngle) && isfinite(control.AccelerationCommand);
            if commandFinite && control.Valid
                steeringTarget=double(control.SteeringAngle);
                accelerationTarget=double(control.AccelerationCommand);
            else
                steeringTarget=0; accelerationTarget=-double(object.MaximumDeceleration);
            end
            steeringTarget=min(max(steeringTarget,-double(object.MaximumSteering)), ...
                double(object.MaximumSteering));
            accelerationTarget=min(max(accelerationTarget,-double(object.MaximumDeceleration)), ...
                double(object.MaximumAcceleration));

            steeringDerivative=(steeringTarget-object.SteeringStateInternal)/ ...
                max(double(object.SteeringTimeConstant),1e-3);
            steeringDerivative=min(max(steeringDerivative,-double(object.MaximumSteeringRate)), ...
                double(object.MaximumSteeringRate));
            accelerationDerivative=(accelerationTarget-object.AccelerationState)/ ...
                max(double(object.AccelerationTimeConstant),1e-3);
            accelerationDerivative=min(max(accelerationDerivative,-double(object.MaximumJerk)), ...
                double(object.MaximumJerk));
            object.SteeringStateInternal=object.SteeringStateInternal+steeringDerivative*dt;
            object.AccelerationState=object.AccelerationState+accelerationDerivative*dt;

            mass=double(object.Mass); inertia=double(object.YawInertia);
            frontDistance=double(object.FrontAxleDistance);
            rearDistance=double(object.RearAxleDistance);
            wheelbase=frontDistance+rearDistance;
            u=object.LongitudinalSpeedState; v=object.LateralSpeedState;
            yawRate=object.YawRateState; steering=object.SteeringStateInternal;
            safeSpeed=max(abs(u),0.5);
            frontSlip=atan2(v+frontDistance*yawRate,safeSpeed)-steering;
            rearSlip=atan2(v-rearDistance*yawRate,safeSpeed);
            frontNormal=mass*gravity*rearDistance/wheelbase*cos(grade)*cos(bank);
            rearNormal=mass*gravity*frontDistance/wheelbase*cos(grade)*cos(bank);
            rawFrontForce=-double(object.FrontCorneringStiffness)*frontSlip;
            rawRearForce=-double(object.RearCorneringStiffness)*rearSlip;
            frontLimit=friction*frontNormal; rearLimit=friction*rearNormal;
            frontForce=min(max(rawFrontForce,-frontLimit),frontLimit);
            rearForce=min(max(rawRearForce,-rearLimit),rearLimit);
            saturated=logical(abs(rawFrontForce-frontForce)>1e-5 || ...
                abs(rawRearForce-rearForce)>1e-5);

            totalLateralForce=frontForce*cos(steering)+rearForce;
            lateralAcceleration=totalLateralForce/mass-gravity*sin(bank);
            requestedLongitudinal=object.AccelerationState-gravity*sin(grade);
            if u>0.05
                requestedLongitudinal=requestedLongitudinal-rolling*gravity*cos(grade);
            end
            lateralUtilization=abs(totalLateralForce)/(friction*mass*gravity);
            availableLongitudinal=friction*gravity*sqrt(max(0,1-min(lateralUtilization,1)^2));
            longitudinalAcceleration=min(max(requestedLongitudinal,-availableLongitudinal), ...
                availableLongitudinal);
            if abs(requestedLongitudinal-longitudinalAcceleration)>1e-5; saturated=true; end

            if u<double(object.LowSpeedThreshold)
                targetYawRate=u/wheelbase*tan(steering);
                yawAcceleration=(targetYawRate-yawRate)/0.15;
                lateralSpeedDerivative=-v/0.10;
                lateralAcceleration=u*yawRate-gravity*sin(bank);
            else
                yawAcceleration=(frontDistance*frontForce*cos(steering)- ...
                    rearDistance*rearForce)/inertia;
                lateralSpeedDerivative=lateralAcceleration-u*yawRate;
            end
            longitudinalSpeedDerivative=longitudinalAcceleration+v*yawRate;
            object.LongitudinalSpeedState=min(max(0,u+longitudinalSpeedDerivative*dt), ...
                double(object.MaximumSpeed));
            object.LateralSpeedState=min(max(v+lateralSpeedDerivative*dt,-15),15);
            object.YawRateState=min(max(yawRate+yawAcceleration*dt,-2.5),2.5);
            object.YawState=object.wrapAngle(object.YawState+object.YawRateState*dt);
            rotation=[cos(object.YawState) -sin(object.YawState); ...
                sin(object.YawState) cos(object.YawState)];
            worldVelocity=rotation*[object.LongitudinalSpeedState;object.LateralSpeedState];
            object.X=object.X+worldVelocity(1)*dt; object.Y=object.Y+worldVelocity(2)*dt;
            object.Z=object.Z+object.LongitudinalSpeedState*sin(grade)*dt;
            totalForce=sqrt((mass*longitudinalAcceleration)^2+totalLateralForce^2);
            frictionUtilization=min(totalForce/max(friction*mass*gravity,1),1.5);
        end

        function angle=wrapAngle(~,angle)
            angle=mod(angle+pi,2*pi)-pi;
        end
    end
end
