function applyRoadsenseModelStyle(modelName)
%APPLYROADSENSEMODELSTYLE Apply the common visual language to a model.
%   The generators remain the source of truth for block placement.  This
%   helper only standardizes typography and role-based colours so every
%   regenerated Roadsense diagram has the same readable appearance.

arguments
    modelName (1,1) string
end

if ~bdIsLoaded(modelName)
    error("Roadsense:ModelNotLoaded", ...
        "Load model '%s' before applying the Roadsense style.",modelName);
end

set_param(modelName,"ScreenColor","[0.98 0.98 0.98]", ...
    "ZoomFactor","FitSystem");

blocks=find_system(modelName,"LookUnderMasks","all", ...
    "FollowLinks","off","Type","Block");
for index=1:numel(blocks)
    block=blocks{index};
    applyIfSupported(block,"FontName","Arial");
    applyIfSupported(block,"FontSize","10");
    applyIfSupported(block,"ForegroundColor","black");

    blockType=get_param(block,"BlockType");
    switch blockType
        case "Inport"
            colour="[0.78 0.89 1.00]";
        case "Outport"
            colour="[0.78 0.95 0.82]";
        case "ModelReference"
            colour="[0.80 0.91 1.00]";
            applyIfSupported(block,"FontWeight","bold");
        case "MATLABSystem"
            colour="[0.84 0.91 1.00]";
            applyIfSupported(block,"FontWeight","bold");
        case {"BusCreator","BusSelector"}
            colour="[1.00 0.93 0.70]";
        case "RateTransition"
            colour="[1.00 0.86 0.57]";
        case "UnitDelay"
            colour="[0.88 0.82 1.00]";
        case "Constant"
            colour="[0.91 0.91 0.91]";
        case {"Goto","From"}
            colour="[0.87 0.95 0.91]";
        case "SubSystem"
            colour="[0.91 0.94 0.98]";
        otherwise
            colour="";
    end
    if strlength(colour)>0
        % Keep intentional component-specific colours (for example the
        % blue environment, green autonomy stack and grey evaluator).
        currentColour="";
        try
            currentColour=string(get_param(block,"BackgroundColor"));
        catch
        end
        if currentColour=="" || strcmpi(currentColour,"white")
            applyIfSupported(block,"BackgroundColor",colour);
        end
    end
end

annotations=find_system(modelName,"FindAll","on","Type","annotation");
for index=1:numel(annotations)
    applyIfSupported(annotations(index),"FontName","Arial");
    applyIfSupported(annotations(index),"ForegroundColor","[0.10 0.16 0.24]");
end
end

function applyIfSupported(object,parameter,value)
try
    set_param(object,parameter,value);
catch exception
    if ~contains(exception.identifier,"InvSimulinkObjectName") && ...
            ~contains(exception.identifier,"ParamUnknown")
        rethrow(exception);
    end
end
end
