classdef PlannerCode < Simulink.IntEnumType
    %PLANNERCODE Outcome of the most recent local-planning cycle.

    enumeration
        NotRun             (0)
        Success            (1)
        ReusedPreviousPlan (2)
        NoFeasiblePath     (3)
        InvalidInput       (4)
        DeadlineMiss       (5)
        EmergencyFallback  (6)
    end

    methods (Static)
        function value = getDefaultValue()
            value = RoadsenseTypes.PlannerCode.NotRun;
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

