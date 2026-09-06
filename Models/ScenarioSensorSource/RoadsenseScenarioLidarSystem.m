classdef RoadsenseScenarioLidarSystem < matlab.System
    %ROADSENSESCENARIOLIDARSYSTEM Generate ground, pothole, and actor returns.
    properties (Nontunable)
        MaximumRange (1,1) single = single(70)
        MaximumPoints (1,1) double = 120000
    end
    methods (Access=protected)
        function [points,intensity,count,overflow,valid,visibleCount,occludedCount,dropout]= ...
                stepImpl(object,time,ego,truth,definition,status)
            points=zeros(120000,3,"single"); intensity=zeros(120000,1,"single");
            count=uint32(0); overflow=false; valid=false; visibleCount=uint16(0);
            occludedCount=uint16(0); dropout=false;
            baseValid=isfinite(time) && ego.Valid && truth.Valid && definition.Valid && status.Valid;
            if ~baseValid; return; end
            [~,dropout,~]=RoadsenseScenarioSensorUtilities.dropoutSchedule(status.ScenarioID,time);
            if dropout; return; end

            [xGrid,yGrid]=meshgrid(single(-10:0.75:60),single(-20:0.75:20));
            zGrid=single(0.008)*sin(single(0.31)*xGrid+single(0.17)*yGrid+single(time));
            if bitand(status.EventMask,uint16(64))~=0
                worldCenter=object.potholeWorldCenter(status.ScenarioID);
                [egoCenter,~]=RoadsenseScenarioSensorUtilities.worldToEgo(ego,worldCenter,zeros(1,3));
                radius=((xGrid-single(egoCenter(1)))/single(1.8)).^2+ ...
                    ((yGrid-single(egoCenter(2)))/single(1.1)).^2;
                zGrid=zGrid-single(0.18)*exp(-single(2.2)*radius);
            end
            ground=[xGrid(:) yGrid(:) zGrid(:)]; groundIntensity=single(0.25)+ ...
                single(0.08)*sin(single(0.11)*(1:size(ground,1)).');
            used=size(ground,1); points(1:used,:)=ground;
            intensity(1:used)=min(max(groundIntensity,0),1);

            [indices,positions,~,occludedCount]=RoadsenseScenarioSensorUtilities.visibleActors( ...
                truth,ego,double(object.MaximumRange),2*pi);
            for item=1:numel(indices)
                actorIndex=indices(item); dimensions=double(truth.Dimensions(actorIndex,:));
                relativeYaw=double(truth.Yaws(actorIndex))-double(ego.Yaw);
                actorPoints=object.boxSurface(positions(item,:),dimensions,relativeYaw, ...
                    double(truth.ActorIDs(actorIndex)),time);
                available=min(size(actorPoints,1),double(object.MaximumPoints)-used);
                if available<=0; overflow=true; break; end
                slots=used+(1:available); points(slots,:)=single(actorPoints(1:available,:));
                reflectivity=single(0.48+0.045*min(double(truth.ClassIDs(actorIndex)),9));
                intensity(slots)=min(reflectivity,single(0.95)); used=used+available;
                visibleCount=visibleCount+uint16(1);
                if available<size(actorPoints,1); overflow=true; break; end
            end
            count=uint32(used); valid=true;
        end
        function center=potholeWorldCenter(~,scenarioID)
            switch double(scenarioID)
                case 1; center=[42 0 0];
                case 5; center=[48 0 0];
                otherwise; center=[35 0 0];
            end
        end
        function cloud=boxSurface(~,center,dimensions,yaw,actorID,time)
            length=max(dimensions(1),0.5); width=max(dimensions(2),0.4);
            height=max(dimensions(3),0.5);
            xLine=linspace(-length/2,length/2,7); yLine=linspace(-width/2,width/2,5);
            zLine=linspace(0.12,height,5);
            [yy,zz]=meshgrid(yLine,zLine); sideX=[-length/2*ones(numel(yy),1);length/2*ones(numel(yy),1)];
            sideY=[yy(:);yy(:)]; sideZ=[zz(:);zz(:)];
            [xx,zz]=meshgrid(xLine,zLine); flankX=[xx(:);xx(:)];
            flankY=[-width/2*ones(numel(xx),1);width/2*ones(numel(xx),1)];
            flankZ=[zz(:);zz(:)]; local=[sideX sideY sideZ;flankX flankY flankZ];
            rotation=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
            xy=(rotation*local(:,1:2).').';
            cloud=[xy(:,1)+center(1),xy(:,2)+center(2),local(:,3)+center(3)];
            phase=double(actorID)*0.037+double(time)*0.71+(1:size(cloud,1)).';
            cloud=cloud+0.008*[sin(phase) cos(phase*1.3) sin(phase*0.7)];
        end
        function n=getNumInputsImpl(~); n=5; end
        function n=getNumOutputsImpl(~); n=8; end
        function names=getInputNamesImpl(~); names=["Time","Ego","TruthActors","Definition","Status"]; end
        function [a,b,c,d,e,f,g,h]=getOutputNamesImpl(~)
            a='Points'; b='Intensity'; c='Count'; d='Overflow'; e='Valid';
            f='VisibleCount'; g='OccludedCount'; h='Dropout';
        end
        function [a,b,c,d,e,f,g,h]=getOutputSizeImpl(~)
            a=[120000 3]; b=[120000 1]; [c,d,e,f,g,h]=deal([1 1]);
        end
        function [a,b,c,d,e,f,g,h]=getOutputDataTypeImpl(~)
            a='single'; b='single'; c='uint32'; d='logical'; e='logical';
            f='uint16'; g='uint16'; h='logical';
        end
        function [a,b,c,d,e,f,g,h]=isOutputFixedSizeImpl(~); [a,b,c,d,e,f,g,h]=deal(true); end
        function [a,b,c,d,e,f,g,h]=isOutputComplexImpl(~); [a,b,c,d,e,f,g,h]=deal(false); end
        function icon=getIconImpl(~); icon="LiDAR\nGround + Objects"; end
    end
end
