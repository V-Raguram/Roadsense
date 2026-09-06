classdef RoadsenseScenarioRadarSystem < matlab.System
    %ROADSENSESCENARIORADARSYSTEM Generate noisy Cartesian radar detections.
    properties (Nontunable)
        MaximumRange (1,1) single = single(150)
        HorizontalFOV (1,1) single = single(deg2rad(140))
        PositionSigma (3,1) single = single([0.35;0.25;0.50])
        VelocitySigma (3,1) single = single([0.40;0.30;0.50])
    end
    methods (Access=protected)
        function [positions,velocities,positionCovariances,velocityCovariances, ...
                scores,count,overflow,valid,visibleCount,occludedCount,dropout]= ...
                stepImpl(object,time,ego,truth,definition,status)
            positions=zeros(256,3,"single"); velocities=zeros(256,3,"single");
            positionCovariances=zeros(3,3,256,"single");
            velocityCovariances=zeros(3,3,256,"single"); scores=zeros(256,1,"single");
            count=uint16(0); overflow=false; valid=false; visibleCount=uint16(0);
            occludedCount=uint16(0); dropout=false;
            baseValid=isfinite(time) && ego.Valid && truth.Valid && definition.Valid && status.Valid;
            if ~baseValid; return; end
            [~,~,dropout]=RoadsenseScenarioSensorUtilities.dropoutSchedule(status.ScenarioID,time);
            if dropout; return; end
            [indices,idealPositions,idealVelocities,occludedCount]= ...
                RoadsenseScenarioSensorUtilities.visibleActors(truth,ego, ...
                double(object.MaximumRange),double(object.HorizontalFOV));
            accepted=0;
            for item=1:numel(indices)
                actorIndex=indices(item); classID=truth.ClassIDs(actorIndex);
                probability=object.detectionProbability(classID);
                phase=double(truth.ActorIDs(actorIndex))*0.173+floor(double(time)/0.05)*0.271;
                pseudoRandom=0.5+0.5*sin(phase*12.9898);
                if pseudoRandom>probability; continue; end
                accepted=accepted+1;
                if accepted>256; overflow=true; break; end
                noisePhase=double(truth.ActorIDs(actorIndex))*0.31+double(time)*[0.7 1.1 1.7];
                positionNoise=double(object.PositionSigma(:).').*sin(noisePhase);
                velocityNoise=double(object.VelocitySigma(:).').*cos(1.4*noisePhase);
                positions(accepted,:)=single(idealPositions(item,:)+positionNoise);
                velocities(accepted,:)=single(idealVelocities(item,:)+velocityNoise);
                positionCovariances(:,:,accepted)=diag(object.PositionSigma.^2);
                velocityCovariances(:,:,accepted)=diag(object.VelocitySigma.^2);
                range=hypot(idealPositions(item,1),idealPositions(item,2));
                scores(accepted)=single(max(0.35,probability*(1-0.25*range/double(object.MaximumRange))));
            end
            count=uint16(min(accepted,256)); visibleCount=count; valid=true;
        end
        function probability=detectionProbability(~,classID)
            switch double(classID)
                case {1,2,3,4}; probability=0.96;
                case 5; probability=0.90;
                case {6,7,8}; probability=0.82;
                case 9; probability=0.70;
                otherwise; probability=0.75;
            end
        end
        function n=getNumInputsImpl(~); n=5; end
        function n=getNumOutputsImpl(~); n=11; end
        function names=getInputNamesImpl(~); names=["Time","Ego","TruthActors","Definition","Status"]; end
        function [a,b,c,d,e,f,g,h,i,j,k]=getOutputNamesImpl(~)
            a='Positions'; b='Velocities'; c='PositionCovariances'; d='VelocityCovariances';
            e='Scores'; f='Count'; g='Overflow'; h='Valid'; i='VisibleCount';
            j='OccludedCount'; k='Dropout';
        end
        function [a,b,c,d,e,f,g,h,i,j,k]=getOutputSizeImpl(~)
            a=[256 3]; b=[256 3]; c=[3 3 256]; d=[3 3 256]; e=[256 1];
            [f,g,h,i,j,k]=deal([1 1]);
        end
        function [a,b,c,d,e,f,g,h,i,j,k]=getOutputDataTypeImpl(~)
            a='single'; b='single'; c='single'; d='single'; e='single'; f='uint16';
            g='logical'; h='logical'; i='uint16'; j='uint16'; k='logical';
        end
        function varargout=isOutputFixedSizeImpl(~); varargout=repmat({true},1,11); end
        function varargout=isOutputComplexImpl(~); varargout=repmat({false},1,11); end
        function icon=getIconImpl(~); icon="Radar\nRange + Velocity"; end
    end
end
