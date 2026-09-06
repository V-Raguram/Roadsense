classdef SensorType < Simulink.IntEnumType
    %SENSORTYPE Origin of a detection or estimate.

    enumeration
        Unknown     (0)
        Camera      (1)
        Lidar       (2)
        Radar       (3)
        Fused       (4)
        GroundTruth (5)
    end

    methods (Static)
        function value = getDefaultValue()
            value = RoadsenseTypes.SensorType.Unknown;
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

