function dictionaryPath = createRoadsenseDataDictionary(dictionaryPath)
%CREATEROADSENSEDATADICTIONARY Create or update the shared data dictionary.
%   This operation is idempotent. It updates only entries owned by the
%   Roadsense shared-data contract and preserves unrelated dictionary data.

arguments
    dictionaryPath (1,1) string = defaultDictionaryPath()
end

dictionaryPath = string(char(java.io.File(dictionaryPath).getCanonicalPath()));
parent = fileparts(dictionaryPath);
if ~isfolder(parent)
    mkdir(parent);
end

if isfile(dictionaryPath)
    dictionary = Simulink.data.dictionary.open(dictionaryPath);
else
    dictionary = Simulink.data.dictionary.create(dictionaryPath);
end
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary,"Design Data");

contract = defineRoadsenseContract();
ownedValues = contract.Buses;
parameterNames = fieldnames(contract.Parameters);
for index = 1:numel(parameterNames)
    name = parameterNames{index};
    ownedValues.(name) = contract.Parameters.(name);
end
names = fieldnames(ownedValues);
for index = 1:numel(names)
    name = names{index};
    newValue = ownedValues.(name);
    try
        entry = getEntry(designData,name);
        setValue(entry,newValue);
    catch exception
        if exception.identifier == "SLDD:sldd:EntryNotFound"
            addEntry(designData,name,newValue);
        else
            rethrow(exception);
        end
    end
end

saveChanges(dictionary);
clear cleanup
fprintf("Updated %d Roadsense entries in %s\n",numel(names),dictionaryPath);
end

function path = defaultDictionaryPath()
sharedDataDir = fileparts(mfilename("fullpath"));
root = fileparts(fileparts(sharedDataDir));
path = fullfile(root,"Data","Roadsense_Data.sldd");
end
