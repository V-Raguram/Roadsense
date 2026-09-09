# Roadsense RoadRunner Integration

## 1. Purpose

Connect the complete Roadsense Simulink autonomy stack to a real RoadRunner
Scenario vehicle so the planned ego motion is visible in a detailed 3D scene.

## 2. Inputs

The RoadRunner Scenario Reader receives the pose message for the ego actor. The
workspace variable `RsRoadRunnerScenarioID` selects one of the five official
Indian-road stages. Three additional Reader paths obtain vision target poses,
vision lane boundaries, and radar target poses from RoadRunner sensor topics.

## 3. Outputs

The RoadRunner Scenario Writer publishes a `BusActorPose` message containing
the ego position, velocity, attitude, and angular velocity at 50 Hz. Logged
native sensor evidence includes camera object/lane detections, radar detections,
and a LiDAR point cloud.

## 4. Simulink Blocks Used

RoadRunner Scenario, RoadRunner Scenario Reader/Writer, Receive/Send, Vision
Detection Generator, Driving Radar Data Generator, Lidar Point Cloud Generator,
Model Reference, Bus Selector, Reshape, Mux, and Bus Creator.

## 5. Algorithms Used

The referenced closed-loop harness contains camera and LiDAR perception, radar
fusion, motion prediction, semantic mapping, behaviour planning, local planning,
trajectory control, independent safety supervision, and vehicle dynamics. The
native RoadRunner sensor bank is the measured-data boundary being adapted into
those typed Roadsense inputs; the deterministic internal source remains active
for repeatable closed-loop qualification until that adapter is enabled.

## 6. Rates

The autonomy plant runs at 100 Hz, control and RoadRunner pose publication at
50 Hz, fusion at 20 Hz, and perception/planning at 10 Hz.

## 7. Safety Behaviour

RoadRunner only renders the pose produced after the independent safety
supervisor and closed-loop vehicle dynamics. It does not bypass the planner.

## 8. Five-Stage Coverage

The same model is reused for village road, uncontrolled intersection, highway
merge, dense market, and cattle-crossing scenarios by changing one scenario ID.

## 9. Validation

The model logs both native RoadRunner sensor measurements and the existing
planning, safety, integration, and acceptance buses while RoadRunner displays
the moving 3D actors. Sensor IDs are camera 1, radar 2, and LiDAR 3.

## 10. Generation and Launch

Run `createRoadsenseRoadRunnerIntegrationModel` to regenerate the SLX file.
Run `runRoadsenseRoadRunnerScenario(1)` from the repository root to launch the
village-road stage. IDs 1 through 5 select the five visual stages; the launcher
loads the RoadRunner bus types and project before starting co-simulation.
