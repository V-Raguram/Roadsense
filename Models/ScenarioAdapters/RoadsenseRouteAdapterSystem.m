classdef RoadsenseRouteAdapterSystem < matlab.System
    %ROADSENSEROUTEADAPTERSYSTEM Convert a world route to ego-frame guidance.
    properties (Nontunable)
        MaxReferencePoints (1,1) double = 256
    end
    properties (Access=private)
        LastScenarioID uint16 = uint16(0)
        ProgressIndex double = 1
    end
    methods (Access=protected)
        function [routeTimestamp,desiredSpeed,speedLimit,distanceToGoal,goalReached, ...
                allowAvoidance,uncontrolledIntersection,occludedArea,mergeRequired, ...
                routeValid,referenceTimestamp,referenceCount,referencePositions, ...
                referenceYaws,recommendedSpeeds,referenceValidMask,referenceValid]= ...
                stepImpl(object,currentTime,ego,scenario)
            routeTimestamp=currentTime; referenceTimestamp=currentTime;
            desiredSpeed=single(0); speedLimit=single(0); distanceToGoal=single(inf);
            goalReached=false; allowAvoidance=scenario.AllowObstacleAvoidance;
            uncontrolledIntersection=scenario.UncontrolledIntersection;
            occludedArea=scenario.OccludedArea; mergeRequired=scenario.MergeRequired;
            referenceCount=uint16(0);
            referencePositions=zeros(object.MaxReferencePoints,2,"single");
            referenceYaws=zeros(object.MaxReferencePoints,1,"single");
            recommendedSpeeds=zeros(object.MaxReferencePoints,1,"single");
            referenceValidMask=false(object.MaxReferencePoints,1);
            count=min(double(scenario.Count),object.MaxReferencePoints);
            routeValid=logical(isfinite(currentTime) && ego.Valid && scenario.Valid && ...
                count>=2 && all(scenario.ValidMask(1:count)) && ...
                all(isfinite(scenario.WorldPositions(1:count,1:2)),"all"));
            referenceValid=routeValid;
            if ~routeValid; return; end

            if scenario.ScenarioID~=object.LastScenarioID
                object.ProgressIndex=1; object.LastScenarioID=scenario.ScenarioID;
            end
            startSearch=max(1,floor(object.ProgressIndex)-3);
            routeXY=scenario.WorldPositions(1:count,1:2);
            distances=vecnorm(routeXY(startSearch:count,:)-ego.Position(1:2).',2,2);
            [~,localIndex]=min(distances);
            nearest=startSearch+localIndex-1;
            object.ProgressIndex=max(object.ProgressIndex,nearest);
            first=min(floor(object.ProgressIndex),count);
            outputCount=min(count-first+1,object.MaxReferencePoints);

            delta=routeXY(first:first+outputCount-1,:)-ego.Position(1:2).';
            cosine=cos(ego.Yaw); sine=sin(ego.Yaw);
            referencePositions(1:outputCount,1)=single(cosine*delta(:,1)+sine*delta(:,2));
            referencePositions(1:outputCount,2)=single(-sine*delta(:,1)+cosine*delta(:,2));
            worldYaws=zeros(outputCount,1);
            for index=1:outputCount
                worldIndex=first+index-1;
                if worldIndex<count
                    direction=routeXY(worldIndex+1,:)-routeXY(worldIndex,:);
                else
                    direction=routeXY(worldIndex,:)-routeXY(max(1,worldIndex-1),:);
                end
                worldYaws(index)=atan2(direction(2),direction(1));
            end
            referenceYaws(1:outputCount)=single(atan2(sin(worldYaws-ego.Yaw), ...
                cos(worldYaws-ego.Yaw)));
            recommendedSpeeds(1:outputCount)=scenario.RecommendedSpeeds(first:first+outputCount-1);
            referenceValidMask(1:outputCount)=true; referenceCount=uint16(outputCount);

            goalDelta=routeXY(count,:)-ego.Position(1:2).';
            distance=norm(routeXY(first,:)-ego.Position(1:2).');
            if first<count
                distance=distance+sum(vecnorm(diff(routeXY(first:count,:),1,1),2,2));
            end
            distanceToGoal=single(distance);
            goalRadius=max(single(0.25),scenario.GoalRadius);
            goalReached=norm(goalDelta)<=double(goalRadius);
            speedLimit=max(single(0),scenario.SpeedLimit);
            desiredSpeed=min(max(single(0),scenario.DesiredSpeed),speedLimit);
            pointSpeed=scenario.RecommendedSpeeds(first);
            if pointSpeed>0; desiredSpeed=min(desiredSpeed,pointSpeed); end
        end
        function resetImpl(object)
            object.LastScenarioID=uint16(0); object.ProgressIndex=1;
        end
        function number=getNumInputsImpl(~); number=3; end
        function number=getNumOutputsImpl(~); number=17; end
        function names=getInputNamesImpl(~); names=["CurrentTime","EgoState","Scenario"]; end
        function varargout=getOutputNamesImpl(~)
            varargout={'RouteTimestamp','DesiredSpeed','SpeedLimit','DistanceToGoal', ...
                'GoalReached','AllowObstacleAvoidance','UncontrolledIntersection', ...
                'OccludedArea','MergeRequired','RouteValid','ReferenceTimestamp', ...
                'ReferenceCount','ReferencePositions','ReferenceYaws', ...
                'RecommendedSpeeds','ReferenceValidMask','ReferenceValid'};
        end
        function varargout=getOutputSizeImpl(object)
            one=[1 1]; n=object.MaxReferencePoints;
            varargout={one,one,one,one,one,one,one,one,one,one,one,one, ...
                [n 2],[n 1],[n 1],[n 1],one};
        end
        function varargout=getOutputDataTypeImpl(~)
            varargout={'double','single','single','single','logical','logical', ...
                'logical','logical','logical','logical','double','uint16','single', ...
                'single','single','logical','logical'};
        end
        function varargout=isOutputFixedSizeImpl(~); varargout=repmat({true},1,17); end
        function varargout=isOutputComplexImpl(~); varargout=repmat({false},1,17); end
        function icon=getIconImpl(~); icon="Roadsense\nWorld-to-Ego Route"; end
    end
end
