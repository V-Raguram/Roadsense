classdef SemanticClass < Simulink.IntEnumType
    %SEMANTICCLASS Meaning of a semantic-grid cell.
    % Grid storage uses uint8 values matching these numeric codes.

    enumeration
        Unknown          (0)
        DrivableSurface  (1)
        NonDrivable      (2)
        RoadEdge         (3)
        Footpath         (4)
        Pothole          (5)
        Vehicle          (6)
        VulnerableUser   (7)
        Animal           (8)
        StaticObstacle   (9)
    end

    methods (Static)
        function value = getDefaultValue()
            value = RoadsenseTypes.SemanticClass.Unknown;
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

