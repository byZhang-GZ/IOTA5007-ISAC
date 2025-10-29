function detections = helperFormatDetectionsForTracker(clusteredDetections, currentTime, rangeResoultion, aoaResolution)
    numDetections = size(clusteredDetections, 2);
    detections.ReceiverLookTime = currentTime;
    detections.ReceiverLookAzimuth = 0;
    detections.ReceiverLookElevation = 0;
    detections.EmitterLookAzimuth = 0;
    detections.EmitterLookElevation = 0;
    detections.DetectionTime = ones(1, numDetections) * currentTime;
    detections.Azimuth = clusteredDetections(2, :);
    detections.Range = clusteredDetections(1, :);

    % The measurement accuracy is proportional to the sensor resolution and
    % depends on the SNR.
    detections.AzimuthAccuracy = ones(1, numDetections) * aoaResolution/12;
    detections.RangeAccuracy = ones(1, numDetections) * rangeResoultion/4;    
end
