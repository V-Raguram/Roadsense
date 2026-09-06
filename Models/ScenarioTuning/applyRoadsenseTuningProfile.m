function snapshot=applyRoadsenseTuningProfile(profile,dictionaryPath)
%APPLYROADSENSETUNINGPROFILE Apply algorithm parameters and return rollback data.
arguments
    profile (1,1) struct
    dictionaryPath (1,1) string = defaultPath()
end
dictionary=Simulink.data.dictionary.open(dictionaryPath);
cleanup=onCleanup(@() close(dictionary)); section=getSection(dictionary,"Design Data");
names=fieldnames(profile.Values); snapshot=struct("DictionaryPath",dictionaryPath, ...
    "Names",{names},"Values",{cell(size(names))});
for index=1:numel(names)
    entry=getEntry(section,names{index}); current=getValue(entry);
    snapshot.Values{index}=current.Value;
    updated=current; updated.Value=profile.Values.(names{index}); setValue(entry,updated);
end
saveChanges(dictionary); clear cleanup;
end

function path=defaultPath()
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
path=fullfile(root,"Data","Roadsense_Data.sldd");
end
