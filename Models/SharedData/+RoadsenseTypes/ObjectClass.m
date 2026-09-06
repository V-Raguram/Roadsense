classdef ObjectClass < Simulink.IntEnumType
    %OBJECTCLASS Semantic class assigned to a detected or tracked object.

    enumeration
        Unknown        (0)
        Car            (1)
        Truck          (2)
        Bus            (3)
        AutoRickshaw   (4)
        Motorcycle     (5)
        Bicycle        (6)
        Pedestrian     (7)
        Pushcart       (8)
        Animal         (9)
        StaticObstacle (10)
    end

    methods (Static)
        function value = getDefaultValue()
            value = RoadsenseTypes.ObjectClass.Unknown;
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

