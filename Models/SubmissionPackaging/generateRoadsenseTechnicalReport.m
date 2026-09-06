function reportPath=generateRoadsenseTechnicalReport(evidenceDirectory,reportPath)
%GENERATEROADSENSETECHNICALREPORT Build the final evidence-backed report.
arguments
    evidenceDirectory (1,1) string
    reportPath (1,1) string = ""
end
summaryPath=fullfile(evidenceDirectory,"validation_summary.csv");
if ~isfile(summaryPath)
    error("Roadsense:Submission:Evidence","Missing validation_summary.csv.");
end
summary=readtable(summaryPath,"TextType","string");
if height(summary)~=5
    error("Roadsense:Submission:Evidence","The technical report requires five scenario rows.");
end
root=fileparts(fileparts(fileparts(mfilename("fullpath"))));
if strlength(reportPath)==0
    reportPath=fullfile(root,"Reports","Roadsense_Technical_Report.md");
end
folder=fileparts(reportPath); if ~isfolder(folder); mkdir(folder); end

lines=strings(0,1);
lines(end+1)="# Roadsense: Adaptive Path Planning on Unstructured Indian Roads";
lines(end+1)="";
lines(end+1)="## Executive summary";
lines(end+1)="";
lines(end+1)="Roadsense is a closed-loop MATLAB/Simulink autonomy stack for mixed, non-lane-based Indian traffic. It integrates camera semantics, LiDAR terrain and obstacle processing, radar velocity, probabilistic sensor fusion, multimodal motion prediction, semantic cost-map fusion, behaviour planning, lattice-style local planning, trajectory control, an independent safety supervisor, and a bicycle-model vehicle plant.";
lines(end+1)="";
lines(end+1)=sprintf("This report is generated from the five-scenario evidence set `%s`. The measured completion rate is %.1f%%, collision-free rate is %.1f%%, and strict acceptance rate is %.1f%%.",evidenceDirectory,100*mean(asLogical(summary.Completed)),100*mean(asLogical(summary.CollisionFree)),100*mean(asLogical(summary.Pass)));
lines(end+1)="";
lines(end+1)="## System architecture";
lines(end+1)="";
lines(end+1)="`Scenario/RoadRunner sensors -> adapters -> semantic and LiDAR perception -> JPDA/IMM fusion -> class-aware prediction -> semantic map -> behaviour planner -> local planner -> controller -> safety supervisor -> vehicle dynamics -> evaluator`";
lines(end+1)="";
lines(end+1)="All interfaces use fixed-size typed buses from `Roadsense_Data.sldd`. Multirate execution uses 10 Hz perception/planning, 20 Hz fusion, 50 Hz control/plant, and a 100 Hz base model interface with explicit rate transitions.";
lines(end+1)="";
lines(end+1)="## AI, prediction, and planning approach";
lines(end+1)="";
lines(end+1)="Camera processing supports a real SegNet baseline and a deterministic synthetic-colour validation mode. LiDAR processing estimates ground, roughness, potholes, occupancy, and clustered objects. Camera class, LiDAR extent, and radar velocity are fused into JPDA tracks initialized with an IMM filter. Every detection uses a consistent position/velocity measurement layout; missing radar velocity is represented with deliberately weak uncertainty.";
lines(end+1)="";
lines(end+1)="The predictor maintains track history and produces continue, brake/stop, left-deviation, and right-deviation modes with class-dependent uncertainty. The semantic map combines terrain, unknown-space cost, current occupancy, and probability-weighted future occupancy. Behaviour logic handles cruise, follow/yield, stop, merge, and recovery. The local planner scores dynamically feasible candidates for collision risk, progress, road cost, comfort, and route tracking, then replans at 10 Hz.";
lines(end+1)="";
lines(end+1)="## Safety and truth boundary";
lines(end+1)="";
lines(end+1)="The safety supervisor independently checks perception freshness, stopping distance, track and predicted occupancy, map validity, actuator limits, and planner health. Scenario actor truth is isolated to sensor synthesis and post-run evaluation; it is never available to perception, prediction, planning, control, or safety decisions.";
lines(end+1)="";
lines(end+1)="## Required scenario results";
lines(end+1)="";
lines(end+1)="| ID | Scenario | Simulated | Collision-free | Completed | Goal | Pass | Clearance (m) | Latency (ms) | Smoothness (1/m) | Jerk (m/s^3) |";
lines(end+1)="|---:|---|:---:|:---:|:---:|:---:|:---:|---:|---:|---:|---:|";
for index=1:height(summary)
    row=summary(index,:);
    lines(end+1)=sprintf("| %d | %s | %s | %s | %s | %s | %s | %.3f | %.2f | %.4f | %.2f |", ...
        row.ScenarioID,row.ScenarioName,yesno(row.SimulationSucceeded), ...
        yesno(row.CollisionFree),yesno(row.Completed),yesno(row.GoalReached), ...
        yesno(row.Pass),row.MinimumClearanceM,row.MaximumReplanLatencyMs, ...
        row.PathSmoothnessInvM,row.MaximumJerkMps3);
end
lines(end+1)="";
lines(end+1)="## Scenario assets";
lines(end+1)="";
lines(end+1)="Five native `drivingScenario` assets cover the unmarked village road, uncontrolled urban intersection, highway slow-vehicle merge, dense market, and sudden cattle crossing. Two RoadRunner HD-map assets provide the detailed village and urban road networks and can be imported as `.rrscene` files with the supplied importer.";
lines(end+1)="";
lines(end+1)="## Validation method and metrics";
lines(end+1)="";
lines(end+1)="The runner performs independent closed-loop simulations, preserves complete timelines, and writes CSV, MAT, PNG, Markdown, and MP4 evidence. Acceptance requires successful simulation, collision-free execution, route completion, goal arrival, pipeline readiness, bounded cross-track error, bounded replan latency, path smoothness, and nominal-control jerk. Evaluation thresholds are never modified by tuning profiles.";
lines(end+1)="";
lines(end+1)="## Reproduction";
lines(end+1)="";
lines(end+1)="```matlab";
lines(end+1)="project = setupRoadsense;";
lines(end+1)="report = runRoadsenseValidationSuite(ScenarioIDs=1:5, UseFastRestart=false);";
lines(end+1)="presentation = generateRoadsensePresentationPackage(report.OutputDirectory, ...";
lines(end+1)="    fullfile(project.Root,'Results','Presentation','Final'));";
lines(end+1)="```";
lines(end+1)="";
lines(end+1)="## Deliverables and limitations";
lines(end+1)="";
lines(end+1)="The submission contains source, generated SLX models, the shared dictionary, five MATLAB scenarios, two RoadRunner maps/scenes when available, test and validation evidence, presentation dashboards/videos, and a frozen deployment manifest. Synthetic sensor validation demonstrates closed-loop behaviour reproducibly; final sensor-domain claims should additionally be supported by RoadRunner rendering and IDD fine-tuning on the target hardware.";
writelines(lines,reportPath);
end

function values=asLogical(values)
if islogical(values)||isnumeric(values); values=logical(values); return; end
values=ismember(lower(strtrim(string(values))),["true","1","yes"]);
end

function value=yesno(flag)
if asLogical(flag); value="YES"; else; value="NO"; end
end
