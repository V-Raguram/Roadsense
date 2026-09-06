classdef RoadsenseScenarioCameraSystem < matlab.System
    %ROADSENSESCENARIOCAMERASYSTEM Render deterministic RGB camera evidence.
    properties (Nontunable)
        HorizontalFOV (1,1) single = single(deg2rad(90))
        MaximumRange (1,1) single = single(80)
        FocalLength (2,1) single = single([400;400])
        PrincipalPoint (2,1) single = single([319.5;239.5])
        CameraHeight (1,1) single = single(1.4)
        PitchDown (1,1) single = single(deg2rad(5))
    end
    methods (Access=protected)
        function [image,valid,visibleCount,occludedCount,dropout]=stepImpl(object,time,ego,truth,definition,status)
            image=zeros(480,640,3,"uint8"); visibleCount=uint16(0);
            occludedCount=uint16(0); dropout=false;
            baseValid=isfinite(time) && ego.Valid && truth.Valid && definition.Valid && status.Valid;
            if ~baseValid; valid=false; return; end
            [dropout,~,~]=RoadsenseScenarioSensorUtilities.dropoutSchedule(status.ScenarioID,time);
            if dropout; valid=false; return; end
            image=object.renderBackground(time,status.EventMask,status.ScenarioID);
            [indices,positions,~,occludedCount]=RoadsenseScenarioSensorUtilities.visibleActors( ...
                truth,ego,double(object.MaximumRange),double(object.HorizontalFOV));
            for item=numel(indices):-1:1
                x=positions(item,1); y=positions(item,2);
                if x<=0.5; continue; end
                width=double(truth.Dimensions(indices(item),2));
                height=double(truth.Dimensions(indices(item),3));
                u=double(object.PrincipalPoint(1))-double(object.FocalLength(1))*y/x;
                % Invert the exact pitched-camera ground projection used by
                % semantic perception.  The former small-angle expression
                % added pitch and height terms, causing distant actors to be
                % reconstructed only 5--8 m ahead and triggering false stops.
                pitch=double(object.PitchDown); heightCamera=double(object.CameraHeight);
                rayDown=(heightCamera*cos(pitch)-x*sin(pitch))/ ...
                    (x*cos(pitch)+heightCamera*sin(pitch));
                bottom=double(object.PrincipalPoint(2))+ ...
                    double(object.FocalLength(2))*rayDown;
                top=bottom-double(object.FocalLength(2))*height/x;
                halfWidth=max(2,double(object.FocalLength(1))*width/(2*x));
                columns=max(1,floor(u-halfWidth)):min(640,ceil(u+halfWidth));
                rows=max(1,floor(top)):min(480,ceil(bottom));
                if isempty(rows) || isempty(columns); continue; end
                color=RoadsenseScenarioSensorUtilities.classColor(truth.ClassIDs(indices(item)));
                for channel=1:3; image(rows,columns,channel)=color(channel); end
                borderRows=unique([rows(1:min(2,end)) rows(max(1,end-1):end)]);
                borderColumns=unique([columns(1:min(2,end)) columns(max(1,end-1):end)]);
                image(borderRows,columns,:)=uint8(20); image(rows,borderColumns,:)=uint8(20);
                visibleCount=visibleCount+uint16(1);
            end
            valid=true;
        end
        function image=renderBackground(~,time,eventMask,scenarioID)
            [column,row]=meshgrid(1:640,1:480); horizon=220;
            image=zeros(480,640,3,"uint8"); sky=row<=horizon;
            roadHalf=55+1.05*max(row-horizon,0); roadMask=row>horizon & abs(column-320)<=roadHalf;
            image(:,:,1)=uint8(104+46*sky+mod(column+floor(time*10),9));
            image(:,:,2)=uint8(122+62*sky+mod(row,7));
            image(:,:,3)=uint8(92+118*sky);
            for channel=1:3
                plane=image(:,:,channel); values=[78 76 72];
                plane(roadMask)=uint8(values(channel)+mod(column(roadMask)+row(roadMask),5));
                image(:,:,channel)=plane;
            end
            edge=abs(abs(column-320)-roadHalf)<3 & row>horizon;
            image(repmat(edge,1,1,3))=uint8(215);
            if bitand(eventMask,uint16(64))~=0
                pothole=(column-390).^2/34^2+(row-370).^2/13^2<1;
                for channel=1:3
                    plane=image(:,:,channel); plane(pothole)=uint8(35+5*channel); image(:,:,channel)=plane;
                end
            end
            if scenarioID==uint16(4)
                awning=row>155 & row<220 & (column<115 | column>525);
                red=image(:,:,1); green=image(:,:,2); blue=image(:,:,3);
                red(awning)=uint8(205); green(awning)=uint8(85); blue(awning)=uint8(45);
                image=cat(3,red,green,blue);
            end
        end
        function n=getNumInputsImpl(~); n=5; end
        function n=getNumOutputsImpl(~); n=5; end
        function names=getInputNamesImpl(~); names=["Time","Ego","TruthActors","Definition","Status"]; end
        function [a,b,c,d,e]=getOutputNamesImpl(~)
            a='Image'; b='Valid'; c='VisibleCount'; d='OccludedCount'; e='Dropout';
        end
        function [a,b,c,d,e]=getOutputSizeImpl(~)
            a=[480 640 3]; b=[1 1]; c=[1 1]; d=[1 1]; e=[1 1];
        end
        function [a,b,c,d,e]=getOutputDataTypeImpl(~)
            a='uint8'; b='logical'; c='uint16'; d='uint16'; e='logical';
        end
        function [a,b,c,d,e]=isOutputFixedSizeImpl(~); [a,b,c,d,e]=deal(true); end
        function [a,b,c,d,e]=isOutputComplexImpl(~); [a,b,c,d,e]=deal(false); end
        function icon=getIconImpl(~); icon="Camera\nRGB + Occlusion"; end
    end
end
