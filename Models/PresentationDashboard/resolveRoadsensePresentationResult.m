function result=resolveRoadsensePresentationResult(source)
%RESOLVEROADSENSEPRESENTATIONRESULT Load a validation result or MAT path.
if isstruct(source) && isfield(source,"Timeline")
    result=source; return
end
if ~(isstring(source)||ischar(source)) || ~isfile(source)
    error("Roadsense:Presentation:Source","Source must be a scenario-result struct or MAT file.");
end
loaded=load(source);
if isfield(loaded,"result"); result=loaded.result;
elseif isfield(loaded,"report") && isfield(loaded.report,"Runs")
    result=loaded.report.Runs{1}.ScenarioResults{1};
else; error("Roadsense:Presentation:Source","MAT file contains no supported Roadsense result.");
end
end
