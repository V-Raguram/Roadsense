classdef BehaviourMode < Simulink.IntEnumType
    %BEHAVIOURMODE Supervisory driving state selected by Stateflow.

    enumeration
        Initialise      (0)
        Cruise          (1)
        Cautious        (2)
        Follow          (3)
        Yield           (4)
        Creep           (5)
        Overtake        (6)
        Stop            (7)
        EmergencyBrake  (8)
        MinimalRiskStop (9)
        AvoidObstacle  (10)
    end

    methods (Static)
        function value = getDefaultValue()
            value = RoadsenseTypes.BehaviourMode.Initialise;
        end

        function value = getDataScope()
            value = "Auto";
        end

        function value = getHeaderFile()
            value = "";
        end

        function value = addClassNameToEnumNames()
            value = true;
        end
    end
end
