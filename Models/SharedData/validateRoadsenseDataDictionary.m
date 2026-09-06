function report = validateRoadsenseDataDictionary(dictionaryPath)
%VALIDATEROADSENSEDATADICTIONARY Validate shared interface consistency.

arguments
    dictionaryPath (1,1) string
end

dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary,"Design Data");
contract = defineRoadsenseContract();

expected = [fieldnames(contract.Buses); fieldnames(contract.Parameters)];
for index = 1:numel(expected)
    getEntry(designData,expected{index});
end

gridRows = double(getValue(getEntry(designData,"RsGridRows")).Value);
gridCols = double(getValue(getEntry(designData,"RsGridCols")).Value);
resolution = double(getValue(getEntry(designData,"RsGridResolution")).Value);
xLimits = double(getValue(getEntry(designData,"RsGridXLimits")).Value);
yLimits = double(getValue(getEntry(designData,"RsGridYLimits")).Value);

assert(gridCols == round(diff(xLimits)/resolution), ...
    "Roadsense:Contract:GridColumns", ...
    "Grid columns do not match x limits and resolution.");
assert(gridRows == round(diff(yLimits)/resolution), ...
    "Roadsense:Contract:GridRows", ...
    "Grid rows do not match y limits and resolution.");

predictionSteps = double(getValue(getEntry(designData,"RsPredictionSteps")).Value);
predictionStep = double(getValue(getEntry(designData,"RsPredictionStep")).Value);
predictionHorizon = double(getValue(getEntry(designData,"RsPredictionHorizon")).Value);
assert(abs((predictionSteps-1)*predictionStep-predictionHorizon) < 1e-6, ...
    "Roadsense:Contract:PredictionHorizon", ...
    "Prediction samples, interval, and horizon are inconsistent.");

rates = ["RsTsPerception","RsTsSemanticInference","RsTsLidarPerception","RsTsFusion","RsTsMap", ...
    "RsTsPrediction","RsTsPlanning","RsTsControl","RsTsScenario"];
rateValues = zeros(size(rates));
for index = 1:numel(rates)
    rateValues(index) = double(getValue(getEntry(designData,rates(index))).Value);
end
baseRate = double(getValue(getEntry(designData,"RsTsBase")).Value);
assert(all(abs(rateValues/baseRate-round(rateValues/baseRate)) < 1e-9), ...
    "Roadsense:Contract:SampleTimes", ...
    "All component sample times must be integer multiples of the base rate.");

report = struct( ...
    "Valid",true, ...
    "ContractVersion",double(getValue(getEntry(designData,"RsContractVersion")).Value), ...
    "EntryCount",numel(expected), ...
    "GridSize",[gridRows gridCols], ...
    "GridCellCount",gridRows*gridCols, ...
    "PredictionHorizon",predictionHorizon, ...
    "BaseSampleTime",baseRate);
end
