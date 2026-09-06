classdef RoadsenseScenarioSensorUtilities
    %ROADSENSESCENARIOSENSORUTILITIES Shared deterministic sensor geometry.
    methods (Static)
        function [positions,velocities]=worldToEgo(ego,worldPositions,worldVelocities)
            yaw=double(ego.Yaw); c=cos(yaw); s=sin(yaw);
            delta=double(worldPositions)-double(ego.Position(:).');
            positions=[c*delta(:,1)+s*delta(:,2), ...
                -s*delta(:,1)+c*delta(:,2),delta(:,3)];
            relativeVelocity=double(worldVelocities)-double(ego.Velocity(:).');
            velocities=[c*relativeVelocity(:,1)+s*relativeVelocity(:,2), ...
                -s*relativeVelocity(:,1)+c*relativeVelocity(:,2),relativeVelocity(:,3)];
        end

        function [indices,egoPositions,egoVelocities,occludedCount]=visibleActors( ...
                truth,ego,maximumRange,horizontalFOV)
            indices=zeros(0,1); egoPositions=zeros(0,3); egoVelocities=zeros(0,3);
            occludedCount=uint16(0);
            if ~truth.Valid || ~ego.Valid; return; end
            valid=find(truth.ValidMask & truth.ActorIDs>0);
            if isempty(valid); return; end
            [allPositions,allVelocities]=RoadsenseScenarioSensorUtilities.worldToEgo( ...
                ego,truth.Positions(valid,:),truth.Velocities(valid,:));
            ranges=hypot(allPositions(:,1),allPositions(:,2));
            bearings=atan2(allPositions(:,2),allPositions(:,1));
            if horizontalFOV<2*pi-1e-4
                inView=allPositions(:,1)>0.5 & ranges<=maximumRange & ...
                    abs(bearings)<=horizontalFOV/2;
            else
                inView=ranges>0.5 & ranges<=maximumRange;
            end
            valid=valid(inView); allPositions=allPositions(inView,:);
            allVelocities=allVelocities(inView,:); ranges=ranges(inView);
            bearings=bearings(inView);
            if isempty(valid); return; end
            [~,order]=sort(ranges,"ascend"); valid=valid(order);
            allPositions=allPositions(order,:); allVelocities=allVelocities(order,:);
            bearings=bearings(order); ranges=ranges(order);
            bins=false(1,720); accepted=false(numel(valid),1);
            fov=double(horizontalFOV);
            for k=1:numel(valid)
                halfAngle=atan2(max(double(truth.Dimensions(valid(k),2))/2,0.25), ...
                    max(ranges(k),0.5));
                center=floor((bearings(k)+fov/2)/fov*719)+1;
                halfBins=max(1,ceil(halfAngle/fov*720));
                candidates=center+(-halfBins:halfBins);
                if fov>=2*pi-1e-4
                    candidates=mod(candidates-1,720)+1;
                else
                    candidates=candidates(candidates>=1 & candidates<=720);
                end
                overlap=mean(bins(candidates));
                if overlap<0.60
                    accepted(k)=true; bins(candidates)=true;
                else
                    occludedCount=occludedCount+uint16(1);
                end
            end
            indices=valid(accepted); egoPositions=allPositions(accepted,:);
            egoVelocities=allVelocities(accepted,:);
        end

        function [camera,lidar,radar]=dropoutSchedule(scenarioID,time)
            id=double(scenarioID); t=double(time);
            camera=(id==2 && t>=5.8 && t<6.1) || (id==4 && t>=11.0 && t<11.3);
            lidar=(id==1 && t>=12.0 && t<12.2) || (id==4 && t>=16.0 && t<16.2);
            radar=(id==3 && t>=9.0 && t<9.15) || (id==5 && t>=6.5 && t<6.65);
        end

        function color=classColor(classID)
            palette=uint8([120 120 120;40 100 220;145 75 35;225 70 40; ...
                245 155 20;135 55 185;25 165 140;30 205 65; ...
                190 115 25;150 95 45;70 70 70]);
            index=min(max(double(classID)+1,1),size(palette,1)); color=palette(index,:);
        end
    end
end
