function manifest=buildRoadsenseSubmissionPackage(evidenceDirectory,outputDirectory,options)
%BUILDROADSENSESUBMISSIONPACKAGE Assemble an accepted, reproducible delivery.
arguments
    evidenceDirectory (1,1) string
    outputDirectory (1,1) string
    options.GenerateVideos (1,1) logical = true
    options.CreateZip (1,1) logical = true
end
summaryPath=fullfile(evidenceDirectory,"validation_summary.csv");
if ~isfile(summaryPath); error("Roadsense:Submission:Evidence","Missing validation summary."); end
summary=readtable(summaryPath,"TextType","string");
if height(summary)~=5 || ~all(asLogical(summary.SimulationSucceeded)) || ...
        ~all(asLogical(summary.CollisionFree)) || ~all(asLogical(summary.Completed)) || ...
        ~all(asLogical(summary.GoalReached)) || ~all(asLogical(summary.Pass))
    error("Roadsense:Submission:Evidence","Packaging requires five fully accepted scenarios.");
end
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
if ~isfolder(outputDirectory); mkdir(outputDirectory); end

copyTree(fullfile(root,"Models"),fullfile(outputDirectory,"Models"));
copyTree(fullfile(root,"Scenarios"),fullfile(outputDirectory,"Scenarios"));
copyTree(fullfile(root,"docs"),fullfile(outputDirectory,"docs"));
copyTree(fullfile(root,"Tests"),fullfile(outputDirectory,"Tests"));
copyTree(evidenceDirectory,fullfile(outputDirectory,"Evidence"));
copyfile(fullfile(root,"README.md"),fullfile(outputDirectory,"README.md"),"f");
copyfile(fullfile(root,"setupRoadsense.m"),fullfile(outputDirectory,"setupRoadsense.m"),"f");
copyfile(fullfile(root,"openRoadsense.m"),fullfile(outputDirectory,"openRoadsense.m"),"f");
dataDirectory=fullfile(outputDirectory,"Data"); if ~isfolder(dataDirectory); mkdir(dataDirectory); end
copyfile(fullfile(root,"Data","Roadsense_Data.sldd"),fullfile(dataDirectory,"Roadsense_Data.sldd"),"f");
for file=["Roadsense_DeploymentProfile.mat","Roadsense_DeploymentProfile.json"]
    source=fullfile(root,"Data",file); if isfile(source); copyfile(source,fullfile(dataDirectory,file),"f"); end
end
presentationDirectory=fullfile(outputDirectory,"Presentation");
generateRoadsensePresentationPackage(evidenceDirectory,presentationDirectory, ...
    GenerateVideos=options.GenerateVideos,FrameRate=15,PlaybackSpeed=4);
reportDirectory=fullfile(outputDirectory,"Reports");
generateRoadsenseTechnicalReport(evidenceDirectory, ...
    fullfile(reportDirectory,"Roadsense_Technical_Report.md"));
manifest=createInventory(outputDirectory);
writetable(manifest,fullfile(outputDirectory,"submission_manifest.csv"));
if options.CreateZip
    zip(outputDirectory+".zip",outputDirectory);
end
end

function copyTree(source,destination)
if ~isfolder(source); error("Roadsense:Submission:Source","Missing source folder: %s",source); end
if ~isfolder(destination); mkdir(destination); end
copyfile(fullfile(source,"*"),destination,"f");
end

function manifest=createInventory(root)
files=dir(fullfile(root,"**","*")); files=files(~[files.isdir]);
paths=strings(numel(files),1); bytes=zeros(numel(files),1,"uint64");
for index=1:numel(files)
    absolute=string(fullfile(files(index).folder,files(index).name));
    paths(index)=replace(erase(absolute,string(root)+filesep),"\","/");
    bytes(index)=uint64(files(index).bytes);
end
manifest=table(paths,bytes,'VariableNames',{'RelativePath','Bytes'});
manifest=sortrows(manifest,"RelativePath");
end

function values=asLogical(values)
if islogical(values)||isnumeric(values); values=logical(values); return; end
values=ismember(lower(strtrim(string(values))),["true","1","yes"]);
end
