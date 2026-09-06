classdef SafetyState < Simulink.IntEnumType
    %SAFETYSTATE Independent vehicle-command supervision state.
    enumeration
        Startup         (0)
        Normal          (1)
        Degraded        (2)
        EmergencyBrake  (3)
        MinimalRiskStop (4)
        StandstillHold  (5)
    end
    methods (Static)
        function value=getDefaultValue(); value=RoadsenseTypes.SafetyState.Startup; end
        function value=getDataScope(); value="Auto"; end
        function value=getHeaderFile(); value=""; end
        function value=addClassNameToEnumNames(); value=true; end
    end
end
