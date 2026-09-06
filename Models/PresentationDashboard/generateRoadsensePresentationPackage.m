function manifest=generateRoadsensePresentationPackage(evidenceDirectory,outputDirectory,options)
%GENERATEROADSENSEPRESENTATIONPACKAGE Build dashboards and optional MP4 files.
arguments
    evidenceDirectory (1,1) string
    outputDirectory (1,1) string
    options.GenerateVideos (1,1) logical = true
    options.FrameRate (1,1) double = 20
    options.PlaybackSpeed (1,1) double = 3
end
if ~isfolder(outputDirectory); mkdir(outputDirectory); end
rows=cell(0,4);
for id=1:5
    source=fullfile(evidenceDirectory,"scenario_"+id+"_result.mat");
    if ~isfile(source); continue; end
    dashboard=fullfile(outputDirectory,"scenario_"+id+"_dashboard.png");
    createRoadsensePresentationDashboard(source,dashboard); video="";
    if options.GenerateVideos
        video=fullfile(outputDirectory,"scenario_"+id+"_demonstration.mp4");
        renderRoadsenseDemonstrationVideo(source,video,FrameRate=options.FrameRate, ...
            PlaybackSpeed=options.PlaybackSpeed);
    end
    rows(end+1,:)={uint16(id),string(source),string(dashboard),string(video)}; %#ok<AGROW>
end
manifest=cell2table(rows,"VariableNames",["ScenarioID","Source","Dashboard","Video"]);
writetable(manifest,fullfile(outputDirectory,"presentation_manifest.csv"));
end
