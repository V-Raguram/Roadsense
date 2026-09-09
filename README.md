# Roadsense

Roadsense is an uncertainty-aware perception and path-planning stack for
autonomous driving on unstructured Indian roads. The implementation targets
MATLAB and Simulink R2025b with RoadRunner scenario co-simulation.

Development is intentionally component-by-component. The repository contains
the executable models, MATLAB implementation, test suites, scenario assets,
and shared data dictionary needed to reproduce the stack.

## Current components

`Models/SharedData` defines the shared data contract used by every future
model reference. `Models/SemanticPerception` contains the first executable AI
model, which converts an RGB camera frame into semantic evidence and preliminary
camera-object regions. `Models/LidarPerception` extracts ground, damaged-surface
evidence, and geometric obstacles from normalized point clouds.
`Models/SensorFusion` combines camera class, LiDAR geometry, and radar velocity
and maintains persistent JPDA/IMM tracks. `Models/MotionPrediction` produces
class-aware multimodal futures with propagated uncertainty.
`Models/SemanticMapFusion` combines perception and prediction into the
planner's traversability/collision-cost grid. `Models/BehaviourPlanner` uses
Stateflow priority and hysteresis to select safe supervisory commands.
`Models/LocalPlanner` samples lane-independent trajectories, checks static and
multimodal dynamic collisions, and provides an emergency braking fallback.
`Models/TrajectoryController` closes the loop with bounded Stanley/curvature
lateral control, PI speed control, and deterministic fail-safe braking.
`Models/VehicleDynamics` provides actuator lag, nonlinear bicycle motion,
friction limits, slope/bank loads, and the updated ego state.
`Models/SafetySupervisor` independently monitors the stack and Stateflow-gates
the final actuator request. `Models/ClosedLoopIntegration` connects all ten
component models with explicit multi-rate scheduling and safety-gated vehicle
feedback. `Models/ScenarioAdapters` validates raw sensor arrays, generates
typed buses, transforms world routes into ego coordinates, and sequences
scenario startup. `Models/ScenarioLibrary` supplies all five required
Indian-road routes, actor truth trajectories, timed events, native
`drivingScenario` assets, and five RoadRunner HD maps/scenes/scenarios.
`Models/ScenarioSensorSource` converts scenario
truth into bounded camera, LiDAR, and radar measurements with occlusion,
deterministic noise, dropouts, covariance, and damaged-road evidence.
`Models/ScenarioClosedLoopHarness` joins all of those blocks to the complete
autonomy stack, closes delayed ego feedback, isolates ground truth, and records
collision, completion, replanning, smoothness, comfort, and readiness metrics.
`Models/ValidationRunner` executes scenario sweeps and generates compact CSV,
MAT, Markdown, trajectory-figure, and scorecard evidence while avoiding large
semantic-grid histories.
`Models/ScenarioTuning` compares rollback-safe baseline, balanced, and cautious
profiles and selects a safety-first recommendation without changing evaluator
thresholds.
`Models/PresentationDashboard` creates judge-facing PNG dashboards and annotated
MP4 replays from truth-isolated validation evidence.
`Models/PresentationDashboard/runRoadsense3DReplay.m` replays the actual
closed-loop ego trajectory in the Unreal-based Simulink 3D Animation viewer
using a native SUV mesh, terrain, road surface, lighting, and brake lamps.
`Models/SubmissionPackaging` creates the evidence-backed technical report and
refuses to assemble a final delivery unless all five scenarios are accepted.
`Models/RoadRunnerIntegration` provides the organized actor-behaviour wrapper,
native RoadRunner message interface, 3D ego pose publication, and registered
MathWorks vision, radar, and LiDAR sensor blocks.
Run the
following from the repository root:

```matlab
openRoadsense
```

For a true 3D vehicle replay after (or while) the autonomous stack runs, use:

```matlab
runRoadsense3DReplay(ScenarioID=1,StopTime=32)
```

To run the complete 3D RoadRunner village stage, use:

```matlab
runRoadsenseRoadRunnerScenario(1)
```

IDs 1 through 5 select the village, intersection, highway merge, market, and
cattle-crossing stages. For a lightweight fallback that does not require
RoadRunner, the temporary MATLAB 3D confidence demo can be opened with:

```matlab
startRoadsenseConfidenceDemo
```

The launcher replays a previously verified full-stack result immediately when
one exists. Its temporary scene contains a fused roadworks obstacle that forces
the behaviour planner into obstacle-avoidance mode and the local planner onto a
lateral bypass. The viewer displays the selected plan, driven trail, live mode,
clearance and goal state. To execute the complete closed-loop Simulink model
again before the real-time replay, use
`startRoadsenseConfidenceDemo(RunModel=true)`. The temporary scenario is
separate from the five required benchmark scenarios and does not replace
RoadRunner validation.

This opens the presentation-ready closed-loop harness and fits the organized
environment, autonomy, plant, and evaluator regions to the Simulink window.
For component tests and scripted validation, run:

```matlab
project = setupRoadsense;
results = runtests("Tests/SharedData");
semanticResults = runtests("Tests/SemanticPerception");
lidarResults = runtests("Tests/LidarPerception");
fusionResults = runtests("Tests/SensorFusion");
predictionResults = runtests("Tests/MotionPrediction");
mapResults = runtests("Tests/SemanticMapFusion");
behaviourResults = runtests("Tests/BehaviourPlanner");
plannerResults = runtests("Tests/LocalPlanner");
controllerResults = runtests("Tests/TrajectoryController");
dynamicsResults = runtests("Tests/VehicleDynamics");
safetyResults = runtests("Tests/SafetySupervisor");
integrationResults = runtests("Tests/ClosedLoopIntegration");
adapterResults = runtests("Tests/ScenarioAdapters");
scenarioResults = runtests("Tests/ScenarioLibrary");
sensorSourceResults = runtests("Tests/ScenarioSensorSource");
harnessResults = runtests("Tests/ScenarioClosedLoopHarness");
validationResults = runtests("Tests/ValidationRunner");
scenarioTuningResults = runtests("Tests/ScenarioTuning");
presentationResults = runtests("Tests/PresentationDashboard");
submissionResults = runtests("Tests/SubmissionPackaging");
table(results)
table(semanticResults)
table(lidarResults)
table(fusionResults)
table(predictionResults)
table(mapResults)
table(behaviourResults)
table(plannerResults)
table(controllerResults)
table(dynamicsResults)
table(safetyResults)
table(integrationResults)
table(adapterResults)
table(scenarioResults)
table(sensorSourceResults)
table(harnessResults)
table(validationResults)
table(scenarioTuningResults)
table(presentationResults)
```

The setup command creates or updates `Data/Roadsense_Data.sldd` without
overwriting unrelated dictionary entries.

## Repository layout

```text
Data/                 Generated Simulink data dictionaries
Models/SharedData/    Shared enums, buses, parameters, and generators
Models/SensorFusion/  Spatial fusion, JPDA/IMM tracking, model generator, demo
Models/MotionPrediction/ Class-aware multimodal motion prediction
Models/SemanticMapFusion/ Camera/LiDAR/dynamic traversability map
Models/BehaviourPlanner/ Hazard assessment, Stateflow, and commands
Models/LocalPlanner/  Adaptive trajectory sampling and collision checking
Models/TrajectoryController/ Closed-loop steering and speed control
Models/VehicleDynamics/ Dynamic bicycle plant and road-condition response
Models/SafetySupervisor/ Independent monitoring and final command gate
Models/ClosedLoopIntegration/ Multi-rate referenced-model closed loop
Models/ScenarioAdapters/ Raw sensor, route, road, and reset adapters
Models/ScenarioLibrary/ Five Indian-road definitions, actors, events, and assets
Models/ScenarioSensorSource/ Synthetic camera, LiDAR, radar, and sensor health
Models/ScenarioClosedLoopHarness/ Full scenario-to-vehicle validation harness
Models/ValidationRunner/ Automated five-scenario benchmark and evidence reports
Models/ScenarioTuning/ Reproducible closed-loop calibration and profile ranking
Models/PresentationDashboard/ Static evidence dashboards and MP4 replay export
Models/SubmissionPackaging/ Technical report and guarded final package builder
Models/RoadRunnerIntegration/ RoadRunner actor interface and native sensor bank
Scenarios/MATLAB/       Native Automated Driving Toolbox scenario assets
Scenarios/RoadRunner/   Five RoadRunner HD maps, scenes, and dynamic scenarios
Tests/SharedData/     Automated shared-contract tests
Tests/SensorFusion/   Multi-sensor fusion and occlusion tests
Tests/MotionPrediction/ Prediction contracts, behavior, and integration tests
Tests/SemanticMapFusion/ Projection, occupancy, risk, cost, and model tests
Tests/BehaviourPlanner/ Decision priority, hysteresis, bounds, and model tests
Tests/LocalPlanner/   Trajectory feasibility, collision, fallback, model tests
Tests/TrajectoryController/ Tracking, limiting, fail-safe, and runtime tests
Tests/VehicleDynamics/ Physical response, friction, reset, and runtime tests
Tests/SafetySupervisor/ Fault priority, latching, override, and runtime tests
Tests/ClosedLoopIntegration/ Scheduling, health, hierarchy, and actuator-path tests
Tests/ScenarioAdapters/ Sensor packing, route transforms, startup, and runtime tests
Tests/ScenarioLibrary/ Routes, road users, event schedules, assets, and model tests
Tests/ScenarioSensorSource/ Sensor geometry, noise, dropout, adapters, and model tests
Tests/ScenarioClosedLoopHarness/ Truth isolation, metrics, rates, and hierarchy tests
Tests/ValidationRunner/ Metric extraction, reporting, and selective logging tests
Tests/ScenarioTuning/ Profile rollback, ranking, timing, and artifact tests
Tests/PresentationDashboard/ Dashboard, video, and evidence-loading tests
Tests/SubmissionPackaging/ Report generation and five-scenario acceptance gate
Tests/RoadRunnerIntegration/ Native messages, sensors, rates, and five-stage assets
```
