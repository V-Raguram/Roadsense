classdef RoadsenseLidarFrameAdapterSystem < matlab.System
    %ROADSENSELIDARFRAMEADAPTERSYSTEM Normalize fixed-capacity LiDAR scans.
    properties (Nontunable)
        MaxPoints (1,1) double = 120000
    end
    properties (Access=private)
        FrameID uint32 = uint32(0)
    end
    methods (Access=protected)
        function [timestamp,frameID,count,points,intensity,overflow,valid]= ...
                stepImpl(object,currentTime,pointsIn,intensityIn,countIn, ...
                overflowIn,inputValid)
            timestamp=currentTime; points=zeros(object.MaxPoints,3,"single");
            intensity=zeros(object.MaxPoints,1,"single"); count=uint32(0);
            requested=double(countIn); bounded=min(max(floor(requested),0),object.MaxPoints);
            overflow=logical(overflowIn || requested>object.MaxPoints);
            valid=logical(inputValid && isfinite(currentTime));
            if bounded>0
                finiteData=all(isfinite(pointsIn(1:bounded,:)),"all") && ...
                    all(isfinite(intensityIn(1:bounded)));
                valid=logical(valid && finiteData);
            end
            if valid
                object.FrameID=object.FrameID+uint32(1); count=uint32(bounded);
                points(1:bounded,:)=single(pointsIn(1:bounded,:));
                intensity(1:bounded)=min(max(single(intensityIn(1:bounded)),0),1);
            end
            frameID=object.FrameID;
        end
        function resetImpl(object); object.FrameID=uint32(0); end
        function number=getNumInputsImpl(~); number=6; end
        function number=getNumOutputsImpl(~); number=7; end
        function names=getInputNamesImpl(~)
            names=["CurrentTime","Points","Intensity","Count","Overflow","InputValid"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','FrameID','Count','Points','Intensity','Overflow','Valid'};
        end
        function varargout=getOutputSizeImpl(object)
            varargout={[1 1],[1 1],[1 1],[object.MaxPoints 3], ...
                [object.MaxPoints 1],[1 1],[1 1]};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','uint32','single','single','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~)
            varargout={true,true,true,true,true,true,true};
        end
        function varargout=isOutputComplexImpl(~)
            varargout={false,false,false,false,false,false,false};
        end
        function icon=getIconImpl(~); icon="Roadsense\nLiDAR Adapter"; end
    end
end
