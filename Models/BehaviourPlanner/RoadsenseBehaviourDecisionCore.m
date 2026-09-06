function [modeCode,emergencyHoldCounter,yieldClearCounter] = ...
    RoadsenseBehaviourDecisionCore(inputsValid,emergencyHazard,goalStop, ...
    yieldRequired,followRequired,creepRecommended,avoidRecommended, ...
    cautiousRecommended,previousMode,emergencyHoldCounter,yieldClearCounter, ...
    emergencyHoldTicks,yieldClearTicks)
%ROADSENSEBEHAVIOURDECISIONCORE Priority and hysteresis for the Stateflow chart.

if emergencyHazard
    emergencyHoldCounter=emergencyHoldTicks;
elseif emergencyHoldCounter > 0
    emergencyHoldCounter=emergencyHoldCounter-uint16(1);
end

if yieldRequired
    yieldClearCounter=uint16(0);
elseif previousMode == uint8(4)
    yieldClearCounter=min(yieldClearCounter+uint16(1),yieldClearTicks);
else
    yieldClearCounter=yieldClearTicks;
end

if ~inputsValid
    modeCode=uint8(9);              % MinimalRiskStop
elseif emergencyHazard || emergencyHoldCounter > 0
    modeCode=uint8(8);              % EmergencyBrake
elseif goalStop
    modeCode=uint8(7);              % Stop
elseif yieldRequired || (previousMode == uint8(4) && yieldClearCounter < yieldClearTicks)
    modeCode=uint8(4);              % Yield
elseif avoidRecommended
    modeCode=uint8(10);             % AvoidObstacle
elseif followRequired
    modeCode=uint8(3);              % Follow
elseif creepRecommended
    modeCode=uint8(5);              % Creep
elseif cautiousRecommended
    modeCode=uint8(2);              % Cautious
else
    modeCode=uint8(1);              % Cruise
end
end
