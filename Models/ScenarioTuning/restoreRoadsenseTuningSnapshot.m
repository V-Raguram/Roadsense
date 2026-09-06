function restoreRoadsenseTuningSnapshot(snapshot)
%RESTOREROADSENSETUNINGSNAPSHOT Restore exact pre-tuning dictionary values.
dictionary=Simulink.data.dictionary.open(snapshot.DictionaryPath);
cleanup=onCleanup(@() close(dictionary)); section=getSection(dictionary,"Design Data");
for index=1:numel(snapshot.Names)
    entry=getEntry(section,snapshot.Names{index}); current=getValue(entry);
    current.Value=snapshot.Values{index}; setValue(entry,current);
end
saveChanges(dictionary); clear cleanup;
end
