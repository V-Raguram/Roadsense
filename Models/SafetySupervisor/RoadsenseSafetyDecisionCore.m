function [stateCode,emergencyCounter,recoveryCounter,emergencyLatched]= ...
        RoadsenseSafetyDecisionCore(inputsValid,criticalFault,emergencyHazard, ...
        degradedCondition,egoMoving,previousState,emergencyCounter,recoveryCounter, ...
        emergencyHoldTicks,recoveryTicks)
%ROADSENSESAFETYDECISIONCORE Priority, latching, and recovery state logic.
emergencyLatched=false;
if emergencyHazard
    stateCode=uint8(3); emergencyCounter=max(uint16(1),emergencyHoldTicks);
    recoveryCounter=uint16(0); return
end
if previousState==uint8(3) && emergencyCounter>uint16(1)
    stateCode=uint8(3); emergencyCounter=emergencyCounter-uint16(1);
    recoveryCounter=uint16(0); emergencyLatched=true; return
end
emergencyCounter=uint16(0);
if criticalFault || ~inputsValid
    stateCode=uint8(4); recoveryCounter=uint16(0); return
end
if previousState==uint8(4) || previousState==uint8(5)
    if egoMoving
        stateCode=uint8(4); recoveryCounter=uint16(0); return
    end
    recoveryCounter=min(recoveryCounter+uint16(1),recoveryTicks);
    if recoveryCounter<recoveryTicks
        stateCode=uint8(5); return
    end
end
recoveryCounter=uint16(0);
if degradedCondition
    stateCode=uint8(2);
else
    stateCode=uint8(1);
end
end
