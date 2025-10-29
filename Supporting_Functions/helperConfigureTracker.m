function tracker = helperConfigureTracker(txPosition, rxPosition, txOrientation, rxOrientation, rangeResolution, aoaResolution)
    % Sensor specification
    sensorSpec = trackerSensorSpec("aerospace", "radar", "bistatic");
    sensorSpec.MeasurementMode = "range-angle";
    sensorSpec.IsReceiverStationary = true;
    sensorSpec.IsEmitterStationary = true;
    sensorSpec.HasElevation = false;
    sensorSpec.HasRangeRate = false;
    sensorSpec.MaxNumLooksPerUpdate = 1;
    sensorSpec.MaxNumMeasurementsPerUpdate = 10;
    sensorSpec.EmitterPlatformPosition = txPosition;
    sensorSpec.EmitterPlatformOrientation = txOrientation.';
    sensorSpec.ReceiverPlatformPosition = rxPosition;
    sensorSpec.ReceiverPlatformOrientation = rxOrientation.';
    sensorSpec.RangeResolution = rangeResolution;
    sensorSpec.AzimuthResolution = aoaResolution;
    sensorSpec.ReceiverFieldOfView = [360 180];
    sensorSpec.EmitterFieldOfView = [360 180];
    sensorSpec.ReceiverRangeLimits = [0 500];
    sensorSpec.EmitterRangeLimits = [0 500];
    sensorSpec.DetectionProbability = 0.9;

    % Target specification
    targetSpec = trackerTargetSpec('custom');
    targetSpec.StateTransitionModel = targetStateTransitionModel('constant-velocity');
    targetSpec.StateTransitionModel.NumMotionDimensions = 2;
    targetSpec.StateTransitionModel.VelocityVariance = 15^2/3*eye(2);
    targetSpec.StateTransitionModel.AccelerationVariance = 0.01^2/3*eye(2);    
    
    tracker = multiSensorTargetTracker(targetSpec,sensorSpec,"jipda"); 
    tracker.ConfirmationExistenceProbability = 0.98; 
    tracker.MaxMahalanobisDistance = 10;
end