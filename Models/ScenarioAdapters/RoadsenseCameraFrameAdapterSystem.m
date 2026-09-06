classdef RoadsenseCameraFrameAdapterSystem < matlab.System
    %ROADSENSECAMERAFRAMEADAPTERSYSTEM Validate and identify camera frames.
    properties (Access=private)
        FrameID uint32 = uint32(0)
    end
    methods (Access=protected)
        function [timestamp,frameID,image,valid]=stepImpl(object,currentTime,imageIn,inputValid)
            valid=logical(inputValid && isfinite(currentTime));
            if valid; object.FrameID=object.FrameID+uint32(1); end
            timestamp=currentTime; frameID=object.FrameID; image=imageIn;
            if ~valid; image(:)=uint8(0); end
        end
        function resetImpl(object); object.FrameID=uint32(0); end
        function number=getNumInputsImpl(~); number=3; end
        function number=getNumOutputsImpl(~); number=4; end
        function names=getInputNamesImpl(~); names=["CurrentTime","Image","InputValid"]; end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','FrameID','Image','Valid'};
        end
        function varargout=getOutputSizeImpl(~)
            varargout={[1 1],[1 1],[480 640 3],[1 1]};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','uint8','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~); varargout={true,true,true,true}; end
        function varargout=isOutputComplexImpl(~); varargout={false,false,false,false}; end
        function icon=getIconImpl(~); icon="Roadsense\nCamera Adapter"; end
    end
end
