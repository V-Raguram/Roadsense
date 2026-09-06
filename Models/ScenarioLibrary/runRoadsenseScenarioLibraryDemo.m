function result=runRoadsenseScenarioLibraryDemo(showFigures)
%RUNROADSENSESCENARIOLIBRARYDEMO Visualize routes, actors, and timed events.
arguments
    showFigures (1,1) logical = true
end
visibility="off"; if showFigures; visibility="on"; end
names=["Unmarked village road","Uncontrolled urban intersection", ...
    "Highway merge with slow traffic","Dense mixed-traffic market", ...
    "Sudden cattle crossing"];
colors=lines(10); rows=cell(5,1); summaryRows=cell(5,1);

routeFigure=figure("Name","Roadsense five-scenario library", ...
    "Color","white","Visible",visibility,"Position",[100 80 1450 820]);
layout=tiledlayout(routeFigure,2,3,"TileSpacing","compact","Padding","compact");
heading=title(layout,"Roadsense | Five required Indian-road scenarios", ...
    "FontWeight","bold","FontSize",16); heading.Color=[0.08 0.08 0.08];
for id=1:5
    [definition,~,status,plans]=RoadsenseScenarioCatalog(id,0);
    duration=double(status.Duration); times=(0:0.1:duration).';
    active=zeros(size(times)); masks=zeros(size(times),"uint16");
    complete=false(size(times));
    for timeIndex=1:numel(times)
        [~,actors,currentStatus]=RoadsenseScenarioCatalog(id,times(timeIndex));
        active(timeIndex)=double(actors.Count); masks(timeIndex)=currentStatus.EventMask;
        complete(timeIndex)=bitand(currentStatus.EventMask,uint16(128))~=0;
    end
    rows{id}=table(repmat(id,numel(times),1),times,active,masks,complete, ...
        'VariableNames',{'ScenarioID','Time_s','ActiveActors','EventMask','Complete'});
    route=definition.WorldPositions(1:double(definition.Count),:);
    delta=diff(route(:,1:2)); routeLength=sum(hypot(delta(:,1),delta(:,2)));
    required=requiredMask(id); observed=uint16(0);
    for value=masks.'; observed=bitor(observed,value); end
    summaryRows{id}=table(id,names(id),duration,routeLength,numel(plans), ...
        all(bitand(observed,required)==required),complete(end), ...
        'VariableNames',{'ScenarioID','Name','Duration_s','RouteLength_m', ...
        'TrafficActors','RequiredEventsObserved','CompletionFlagObserved'});

    ax=nexttile(layout,id); hold(ax,"on"); grid(ax,"on"); axis(ax,"equal");
    styleAxes(ax);
    roadWidth=roadWidthFor(id);
    plot(ax,route(:,1),route(:,2),"Color",[0.75 0.75 0.75], ...
        "LineWidth",roadWidth,"HandleVisibility","off");
    plot(ax,route(:,1),route(:,2),"b-","LineWidth",2,"DisplayName","Ego route");
    for actorIndex=1:numel(plans)
        path=plans(actorIndex).Waypoints;
        plot(ax,path(:,1),path(:,2),"--","LineWidth",1.5, ...
            "Color",colors(double(plans(actorIndex).ClassID)+1,:), ...
            "DisplayName",classLabel(plans(actorIndex).ClassID));
        scatter(ax,path(1,1),path(1,2),30, ...
            colors(double(plans(actorIndex).ClassID)+1,:),"filled", ...
            "HandleVisibility","off");
    end
    scatter(ax,route(1,1),route(1,2),55,[0.1 0.65 0.2],"filled", ...
        "DisplayName","Start");
    scatter(ax,route(end,1),route(end,2),65,[0.85 0.15 0.15],"pentagram", ...
        "filled","DisplayName","Goal");
    plotTitle=title(ax,sprintf("%d. %s",id,names(id))); plotTitle.Color=[0.08 0.08 0.08];
    xlabel(ax,"World x (m)"); ylabel(ax,"World y (m)");
end
legend(nexttile(layout,6),"off"); axis(nexttile(layout,6),"off");
text(nexttile(layout,6),0.04,0.92,{"Legend","Blue: ego route","Dashed: road-user paths", ...
    "Green: start","Red: goal","Grey: usable road envelope"}, ...
    "FontSize",11,"Color",[0.08 0.08 0.08],"VerticalAlignment","top");

timeline=vertcat(rows{:}); summary=vertcat(summaryRows{:});
timelineFigure=figure("Name","Roadsense scenario event schedules", ...
    "Color","white","Visible",visibility,"Position",[120 100 1250 760]);
timelineLayout=tiledlayout(timelineFigure,5,1,"TileSpacing","compact","Padding","compact");
heading=title(timelineLayout,"Roadsense | Actor load and event schedule", ...
    "FontWeight","bold","FontSize",15); heading.Color=[0.08 0.08 0.08];
for id=1:5
    ax=nexttile(timelineLayout,id); subset=timeline(timeline.ScenarioID==id,:);
    styleAxes(ax);
    stairs(ax,subset.Time_s,subset.ActiveActors,"LineWidth",1.8,"Color",[0.05 0.40 0.78]);
    grid(ax,"on"); ylabel(ax,sprintf("S%d actors",id)); xlim(ax,[0 max(subset.Time_s)]);
    if id==5; xlabel(ax,"Scenario time (s)"); else; ax.XTickLabel=[]; end
end
result=struct("Summary",summary,"Timeline",timeline, ...
    "RouteFigure",routeFigure,"TimelineFigure",timelineFigure);
end

function styleAxes(ax)
ax.Color=[0.98 0.98 0.98]; ax.XColor=[0.15 0.15 0.15];
ax.YColor=[0.15 0.15 0.15]; ax.GridColor=[0.72 0.72 0.72];
ax.GridAlpha=0.45; ax.FontName="Arial"; ax.FontSize=9;
end

function value=requiredMask(id)
values=uint16([8+16+64,2+8,4+32,2+8,16+64]); value=values(id);
end
function value=roadWidthFor(id)
values=[10 11 14 9 11]; value=values(id);
end
function label=classLabel(id)
labels=["Unknown","Car","Truck","Bus","Auto-rickshaw","Motorcycle", ...
    "Bicycle","Pedestrian","Pushcart","Animal","Obstacle"];
label=labels(double(id)+1);
end
