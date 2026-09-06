classdef RoadsenseScenarioSensorStatusSystem < matlab.System
    %ROADSENSESCENARIOSENSORSTATUSSYSTEM Pack visibility and dropout status.
    methods (Access=protected)
        function varargout=stepImpl(~,time,status,cameraCount,lidarCount,radarCount, ...
                cameraOccluded,lidarOccluded,radarOccluded,cameraDropout,lidarDropout,radarDropout)
            occluded=max([cameraOccluded lidarOccluded radarOccluded]);
            sourceValid=isfinite(time) && status.Valid;
            varargout={time,status.ScenarioID,cameraCount,lidarCount,radarCount, ...
                occluded,cameraDropout,lidarDropout,radarDropout,sourceValid};
        end
        function n=getNumInputsImpl(~); n=11; end
        function n=getNumOutputsImpl(~); n=10; end
        function names=getInputNamesImpl(~)
            names=["Time","ScenarioStatus","CameraCount","LidarCount","RadarCount", ...
                "CameraOccluded","LidarOccluded","RadarOccluded", ...
                "CameraDropout","LidarDropout","RadarDropout"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','ScenarioID','CameraVisibleCount','LidarVisibleCount', ...
                'RadarVisibleCount','OccludedCount','CameraDropout','LidarDropout', ...
                'RadarDropout','Valid'};
        end
        function varargout=getOutputSizeImpl(~); varargout=repmat({[1 1]},1,10); end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint16','uint16','uint16','uint16','uint16', ...
                'logical','logical','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~); varargout=repmat({true},1,10); end
        function varargout=isOutputComplexImpl(~); varargout=repmat({false},1,10); end
        function icon=getIconImpl(~); icon="Sensor Source\nHealth"; end
    end
end
