classdef testRoadsenseSharedData < matlab.unittest.TestCase
    %TESTROADSENSESHAREDDATA Component tests for the shared data contract.

    properties
        Root
        DictionaryPath
    end

    methods (TestMethodSetup)
        function configure(testCase)
            testFile = mfilename("fullpath");
            testCase.Root = fileparts(fileparts(fileparts(testFile)));
            addpath(fullfile(testCase.Root,"Models","SharedData"));
            testCase.DictionaryPath = fullfile(testCase.Root,"Data","Roadsense_Data.sldd");
            createRoadsenseDataDictionary(testCase.DictionaryPath);
        end
    end

    methods (Test)
        function dictionaryExists(testCase)
            testCase.verifyTrue(isfile(testCase.DictionaryPath));
        end

        function contractValidates(testCase)
            report = validateRoadsenseDataDictionary(testCase.DictionaryPath);
            testCase.verifyTrue(report.Valid);
            testCase.verifyEqual(report.ContractVersion,16);
            testCase.verifyEqual(report.GridSize,[200 320]);
            testCase.verifyEqual(report.GridCellCount,64000);
            testCase.verifyEqual(report.PredictionHorizon,3,"AbsTol",1e-6);
            testCase.verifyEqual(report.BaseSampleTime,0.01,"AbsTol",1e-12);
        end

        function enumValuesRemainStable(testCase)
            testCase.verifyEqual(int32(RoadsenseTypes.ObjectClass.AutoRickshaw),int32(4));
            testCase.verifyEqual(int32(RoadsenseTypes.ObjectClass.Animal),int32(9));
            testCase.verifyEqual(int32(RoadsenseTypes.SensorType.Lidar),int32(2));
            testCase.verifyEqual(int32(RoadsenseTypes.BehaviourMode.EmergencyBrake),int32(8));
            testCase.verifyEqual(int32(RoadsenseTypes.BehaviourMode.AvoidObstacle),int32(10));
            testCase.verifyEqual(int32(RoadsenseTypes.SemanticClass.Pothole),int32(5));
            testCase.verifyEqual(int32(RoadsenseTypes.SafetyState.EmergencyBrake),int32(3));
        end

        function fixedCapacitiesMatchBusDimensions(testCase)
            contract = defineRoadsenseContract();
            detectionElement = findElement( ...
                contract.Buses.RsSensorDetectionArrayBus,"Detections");
            trackElement = findElement( ...
                contract.Buses.RsTrackedObjectArrayBus,"Objects");
            predictionElement = findElement( ...
                contract.Buses.RsPredictionArrayBus,"Predictions");
            pointElement = findElement( ...
                contract.Buses.RsPlannedTrajectoryBus,"Points");

            testCase.verifyEqual(detectionElement.Dimensions,[256 1]);
            testCase.verifyEqual(trackElement.Dimensions,[128 1]);
            testCase.verifyEqual(predictionElement.Dimensions,[128 1]);
            testCase.verifyEqual(pointElement.Dimensions,[61 1]);
        end

        function gridBusMatchesConfiguredGeometry(testCase)
            contract = defineRoadsenseContract();
            occupancy = findElement(contract.Buses.RsSemanticGridBus,"Occupancy");
            risk = findElement(contract.Buses.RsSemanticGridBus,"PredictedRisk");
            combined = findElement(contract.Buses.RsSemanticGridBus,"CombinedCost");
            observed = findElement(contract.Buses.RsSemanticGridBus,"ObservedMask");
            testCase.verifyEqual(occupancy.Dimensions,[200 320]);
            testCase.verifyEqual(risk.Dimensions,[200 320]);
            testCase.verifyEqual(combined.Dimensions,[200 320]);
            testCase.verifyEqual(observed.Dimensions,[200 320]);
            testCase.verifyEqual(occupancy.DataType,'single');
            testCase.verifyEqual(observed.DataType,'boolean');
        end


        function cameraInterfacesHaveFixedShapes(testCase)
            contract = defineRoadsenseContract();
            image = findElement(contract.Buses.RsCameraFrameBus,"Image");
            labels = findElement(contract.Buses.RsImageSemanticBus,"SemanticLabel");
            boxes = findElement(contract.Buses.RsCameraObjectListBus,"BoundingBoxes");
            testCase.verifyEqual(image.Dimensions,[480 640 3]);
            testCase.verifyEqual(labels.Dimensions,[120 160]);
            testCase.verifyEqual(boxes.Dimensions,[256 4]);
        end


        function lidarInterfacesHaveFixedShapes(testCase)
            contract = defineRoadsenseContract();
            points = findElement(contract.Buses.RsLidarFrameBus,"Points");
            occupancy = findElement(contract.Buses.RsLidarGridEvidenceBus,"Occupancy");
            positions = findElement(contract.Buses.RsLidarObjectListBus,"Positions");
            testCase.verifyEqual(points.Dimensions,[120000 3]);
            testCase.verifyEqual(occupancy.Dimensions,[200 320]);
            testCase.verifyEqual(positions.Dimensions,[256 3]);
        end

        function fusionInterfacesHaveFixedShapes(testCase)
            contract = defineRoadsenseContract();
            radarVelocity = findElement(contract.Buses.RsRadarObjectListBus,"Velocities");
            trackPosition = findElement(contract.Buses.RsFusedTrackListBus,"Positions");
            trackCovariance = findElement(contract.Buses.RsFusedTrackListBus,"StateCovariances");
            testCase.verifyEqual(radarVelocity.Dimensions,[256 3]);
            testCase.verifyEqual(trackPosition.Dimensions,[128 3]);
            testCase.verifyEqual(trackCovariance.Dimensions,[6 6 128]);
        end

        function motionPredictionInterfaceHasFixedShapes(testCase)
            contract = defineRoadsenseContract();
            positions = findElement(contract.Buses.RsMotionPredictionListBus,"Positions");
            probabilities = findElement(contract.Buses.RsMotionPredictionListBus,"ModeProbabilities");
            covariance = findElement(contract.Buses.RsMotionPredictionListBus,"PositionCovariances");
            testCase.verifyEqual(positions.Dimensions,[128 16 4 2]);
            testCase.verifyEqual(probabilities.Dimensions,[128 4]);
            testCase.verifyEqual(covariance.Dimensions,[2 2 16 4 128]);
        end

        function behaviourInterfacesContainSafetyFields(testCase)
            contract = defineRoadsenseContract();
            mode = findElement(contract.Buses.RsBehaviourCommandBus,"Mode");
            reason = findElement(contract.Buses.RsBehaviourCommandBus,"ReasonMask");
            contextSpeed = findElement(contract.Buses.RsRouteContextBus,"DesiredSpeed");
            testCase.verifyEqual(mode.DataType,'Enum: RoadsenseTypes.BehaviourMode');
            testCase.verifyEqual(reason.DataType,'uint16');
            testCase.verifyEqual(contextSpeed.DataType,'single');
        end

        function localPlannerInterfacesHaveFixedShapes(testCase)
            contract=defineRoadsenseContract();
            reference=findElement(contract.Buses.RsReferencePathBus,"Positions");
            planPositions=findElement(contract.Buses.RsLocalPlanBus,"Positions");
            steering=findElement(contract.Buses.RsLocalPlanBus,"SteeringAngles");
            testCase.verifyEqual(reference.Dimensions,[256 2]);
            testCase.verifyEqual(planPositions.Dimensions,[61 2]);
            testCase.verifyEqual(steering.Dimensions,[61 1]);
        end

        function controllerInterfacesContainActuationAndTracking(testCase)
            contract=defineRoadsenseContract();
            steering=findElement(contract.Buses.RsVehicleControlBus,"SteeringAngle");
            throttle=findElement(contract.Buses.RsVehicleControlBus,"Throttle");
            crossTrack=findElement(contract.Buses.RsTrackingStatusBus,"CrossTrackError");
            testCase.verifyEqual(steering.DataType,'single');
            testCase.verifyEqual(steering.Unit,'rad');
            testCase.verifyEqual(throttle.Max,1);
            testCase.verifyEqual(crossTrack.Unit,'m');
        end

        function dynamicsInterfacesExposeRoadAndPhysicalState(testCase)
            contract=defineRoadsenseContract();
            friction=findElement(contract.Buses.RsRoadConditionBus,"FrictionCoefficient");
            sideslip=findElement(contract.Buses.RsVehicleDynamicsStatusBus,"SideslipAngle");
            utilization=findElement(contract.Buses.RsVehicleDynamicsStatusBus,"FrictionUtilization");
            testCase.verifyEqual(friction.DataType,'single');
            testCase.verifyEqual(sideslip.Unit,'rad');
            testCase.verifyEqual(utilization.Dimensions,1);
        end

        function safetyInterfaceContainsIndependentEvidence(testCase)
            contract=defineRoadsenseContract();
            state=findElement(contract.Buses.RsSafetyStatusBus,"State");
            ttc=findElement(contract.Buses.RsSafetyStatusBus,"MinimumTTC");
            reason=findElement(contract.Buses.RsSafetyStatusBus,"ReasonMask");
            testCase.verifyEqual(state.DataType,'Enum: RoadsenseTypes.SafetyState');
            testCase.verifyEqual(ttc.Unit,'s');
            testCase.verifyEqual(reason.DataType,'uint16');
        end

        function integrationInterfaceExposesEndToEndReadiness(testCase)
            contract=defineRoadsenseContract();
            ready=findElement(contract.Buses.RsIntegrationStatusBus,"PipelineReady");
            override=findElement(contract.Buses.RsIntegrationStatusBus,"SafetyOverride");
            cycle=findElement(contract.Buses.RsIntegrationStatusBus,"CycleID");
            testCase.verifyEqual(ready.DataType,'boolean');
            testCase.verifyEqual(override.DataType,'boolean');
            testCase.verifyEqual(cycle.DataType,'uint32');
        end

        function scenarioDefinitionHasFixedWorldRoute(testCase)
            contract=defineRoadsenseContract();
            route=findElement(contract.Buses.RsScenarioDefinitionBus,"WorldPositions");
            initial=findElement(contract.Buses.RsScenarioDefinitionBus,"InitialPosition");
            scenario=findElement(contract.Buses.RsScenarioDefinitionBus,"ScenarioID");
            testCase.verifyEqual(route.Dimensions,[256 3]);
            testCase.verifyEqual(route.DataType,'double');
            testCase.verifyEqual(initial.Dimensions,[3 1]);
            testCase.verifyEqual(scenario.DataType,'uint16');
        end

        function scenarioTruthHasFixedActorCapacity(testCase)
            contract=defineRoadsenseContract();
            positions=findElement(contract.Buses.RsScenarioActorListBus,"Positions");
            events=findElement(contract.Buses.RsScenarioStatusBus,"EventMask");
            testCase.verifyEqual(positions.Dimensions,[64 3]);
            testCase.verifyEqual(positions.DataType,'double');
            testCase.verifyEqual(events.DataType,'uint16');
        end

        function scenarioSensorStatusExposesVisibilityAndDropout(testCase)
            contract=defineRoadsenseContract();
            camera=findElement(contract.Buses.RsScenarioSensorStatusBus,"CameraVisibleCount");
            occluded=findElement(contract.Buses.RsScenarioSensorStatusBus,"OccludedCount");
            radarDropout=findElement(contract.Buses.RsScenarioSensorStatusBus,"RadarDropout");
            testCase.verifyEqual(camera.DataType,'uint16');
            testCase.verifyEqual(occluded.Dimensions,1);
            testCase.verifyEqual(radarDropout.DataType,'boolean');
        end

        function scenarioEvaluationContainsRequiredAcceptanceMetrics(testCase)
            contract=defineRoadsenseContract();
            clearance=findElement(contract.Buses.RsScenarioEvaluationBus,"MinimumClearance");
            latency=findElement(contract.Buses.RsScenarioEvaluationBus,"MaximumReplanLatency");
            smoothness=findElement(contract.Buses.RsScenarioEvaluationBus,"PathSmoothness");
            completion=findElement(contract.Buses.RsScenarioEvaluationBus,"GoalReached");
            testCase.verifyEqual(clearance.Unit,'m');
            testCase.verifyEqual(latency.Unit,'s');
            testCase.verifyEqual(smoothness.Unit,'1/m');
            testCase.verifyEqual(completion.DataType,'boolean');
        end

        function sampleTimesAreHarmonic(testCase)
            contract = defineRoadsenseContract();
            names = ["RsTsPerception","RsTsSemanticInference","RsTsLidarPerception","RsTsFusion","RsTsMap", ...
                "RsTsPrediction","RsTsPlanning","RsTsControl","RsTsScenario"];
            values = zeros(size(names));
            for index = 1:numel(names)
                values(index) = contract.Parameters.(names(index)).Value;
            end
            baseRate = contract.Parameters.RsTsBase.Value;
            testCase.verifyEqual(values/baseRate,round(values/baseRate),"AbsTol",1e-12);
        end
    end
end

function element = findElement(bus,name)
names = string({bus.Elements.Name});
element = bus.Elements(names == name);
end
