function manifest=freezeRoadsenseDeploymentProfile(profileName,evidenceDirectory,options)
%FREEZEROADSENSEDEPLOYMENTPROFILE Persist a validated default tuning profile.
arguments
    profileName (1,1) string {mustBeMember(profileName,["baseline","balanced","cautious"])}
    evidenceDirectory (1,1) string
    options.RequireAcceptedEvidence (1,1) logical = true
end
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
summaryPath=fullfile(evidenceDirectory,"validation_summary.csv");
accepted=false;
if isfile(summaryPath)
    summary=readtable(summaryPath,"TextType","string");
    required=["SimulationSucceeded","CollisionFree","Completed","GoalReached","Pass"];
    hasRequired=all(ismember(required,string(summary.Properties.VariableNames)));
    accepted=height(summary)==5 && hasRequired;
    if accepted
        for name=required
            accepted=accepted && allTrue(summary.(name));
        end
    end
end
if options.RequireAcceptedEvidence && ~accepted
    error("Roadsense:Deployment:Evidence", ...
        "Freezing requires five fully accepted scenarios in validation_summary.csv.");
end
profile=RoadsenseTuningProfile(profileName);
manifest=struct("Project","Roadsense","ProfileName",profileName, ...
    "ContractVersion",profile.ContractVersion,"TuningSchemaVersion",profile.TuningSchemaVersion, ...
    "CreatedUTC",string(datetime("now","TimeZone","UTC","Format","yyyy-MM-dd'T'HH:mm:ss'Z'")), ...
    "EvidenceDirectory",string(evidenceDirectory),"FiveScenarioAccepted",accepted, ...
    "Values",profile.Values);
matPath=fullfile(root,"Data","Roadsense_DeploymentProfile.mat");
jsonPath=fullfile(root,"Data","Roadsense_DeploymentProfile.json");
save(matPath,"manifest");
writelines(string(jsonencode(manifest,PrettyPrint=true)),jsonPath);
applyRoadsenseTuningProfile(profile,fullfile(root,"Data","Roadsense_Data.sldd"));
fprintf("Frozen Roadsense deployment profile: %s\n",profileName);
end

function tf=allTrue(values)
if islogical(values) || isnumeric(values)
    tf=all(logical(values));
else
    tf=all(ismember(lower(strtrim(string(values))),["true","1","yes"]));
end
end
