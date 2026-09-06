function modelName=openRoadsense()
%OPENROADSENSE Configure the project and open the presentation-ready harness.
project=setupRoadsense;
modelName="Roadsense_ScenarioClosedLoopHarness";
modelPath=fullfile(project.Root,"Models","ScenarioClosedLoopHarness",modelName+".slx");
if ~isfile(modelPath); createRoadsenseScenarioClosedLoopHarnessModel(); end
load_system(modelPath);
open_system(modelName);
set_param(modelName,"ZoomFactor","FitSystem");
fprintf("Opened the complete Roadsense closed-loop model: %s\n",modelPath);
end
