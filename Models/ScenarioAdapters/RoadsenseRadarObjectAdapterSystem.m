classdef RoadsenseRadarObjectAdapterSystem < matlab.System
    %ROADSENSERADAROBJECTADAPTERSYSTEM Validate Cartesian radar detections.
    properties (Nontunable)
        MaxDetections (1,1) double = 256
    end
    properties (Access=private)
        FrameID uint32 = uint32(0)
    end
    methods (Access=protected)
        function [timestamp,frameID,count,positions,velocities,positionCovariances, ...
                velocityCovariances,scores,validMask,overflow,valid]=stepImpl(object, ...
                currentTime,positionsIn,velocitiesIn,positionCovariancesIn, ...
                velocityCovariancesIn,scoresIn,countIn,overflowIn,inputValid)
            timestamp=currentTime; count=uint16(0);
            positions=zeros(object.MaxDetections,3,"single");
            velocities=zeros(object.MaxDetections,3,"single");
            positionCovariances=zeros(3,3,object.MaxDetections,"single");
            velocityCovariances=zeros(3,3,object.MaxDetections,"single");
            scores=zeros(object.MaxDetections,1,"single");
            validMask=false(object.MaxDetections,1);
            requested=double(countIn); bounded=min(max(floor(requested),0),object.MaxDetections);
            overflow=logical(overflowIn || requested>object.MaxDetections);
            valid=logical(inputValid && isfinite(currentTime));
            if bounded>0
                valid=logical(valid && all(isfinite(positionsIn(1:bounded,:)),"all") && ...
                    all(isfinite(velocitiesIn(1:bounded,:)),"all"));
            end
            if valid
                object.FrameID=object.FrameID+uint32(1); count=uint16(bounded);
                positions(1:bounded,:)=single(positionsIn(1:bounded,:));
                velocities(1:bounded,:)=single(velocitiesIn(1:bounded,:));
                scores(1:bounded)=min(max(single(scoresIn(1:bounded)),0),1);
                validMask(1:bounded)=true;
                for index=1:bounded
                    positionCovariances(:,:,index)=object.diagonalCovariance( ...
                        positionCovariancesIn(:,:,index),single([0.8 0.8 1.5]));
                    velocityCovariances(:,:,index)=object.diagonalCovariance( ...
                        velocityCovariancesIn(:,:,index),single([1.5 1.5 2.0]));
                end
            end
            frameID=object.FrameID;
        end
        function resetImpl(object); object.FrameID=uint32(0); end
        function number=getNumInputsImpl(~); number=9; end
        function number=getNumOutputsImpl(~); number=11; end
        function names=getInputNamesImpl(~)
            names=["CurrentTime","Positions","Velocities","PositionCovariances", ...
                "VelocityCovariances","Scores","Count","Overflow","InputValid"];
        end
        function varargout=getOutputNamesImpl(~)
            varargout={'Timestamp','FrameID','Count','Positions','Velocities', ...
                'PositionCovariances','VelocityCovariances','Scores','ValidMask', ...
                'Overflow','Valid'};
        end
        function varargout=getOutputSizeImpl(object)
            n=object.MaxDetections;
            varargout={[1 1],[1 1],[1 1],[n 3],[n 3],[3 3 n],[3 3 n], ...
                [n 1],[n 1],[1 1],[1 1]};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','uint32','uint16','single','single','single', ...
                'single','single','logical','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~)
            varargout=repmat({true},1,11);
        end
        function varargout=isOutputComplexImpl(~)
            varargout=repmat({false},1,11);
        end
        function icon=getIconImpl(~); icon="Roadsense\nRadar Adapter"; end
    end
    methods (Static,Access=private)
        function covariance=diagonalCovariance(raw,defaultSigma)
            diagonal=single([raw(1,1) raw(2,2) raw(3,3)]);
            fallback=defaultSigma.^2;
            invalid=~isfinite(diagonal) | diagonal<=single(1e-6);
            diagonal(invalid)=fallback(invalid);
            covariance=diag(diagonal);
        end
    end
end
