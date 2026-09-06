function configuration=RoadsenseValidationConfiguration(root)
%ROADSENSEVALIDATIONCONFIGURATION Default reproducible benchmark settings.
arguments
    root (1,1) string = string(fileparts(fileparts(fileparts(mfilename("fullpath")))))
end
configuration=struct;
configuration.ScenarioIDs=1:5;
configuration.ScenarioNames=["Unmarked Village Road", ...
    "Uncontrolled Urban Intersection","Highway Merge With Slow Vehicles", ...
    "Dense Market Mixed Traffic","Sudden Cattle Crossing", ...
    "Temporary 3D Confidence Demo"];
configuration.CompletionBuffer=0.20;
configuration.OutputDirectory=fullfile(root,"Results","ValidationRunner");
configuration.UseFastRestart=false;
configuration.GenerateFigures=true;
configuration.SaveTimelines=true;
configuration.ContinueOnError=true;
configuration.ContractVersion=uint32(16);
end
