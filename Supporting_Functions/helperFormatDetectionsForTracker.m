function detections = helperFormatDetectionsForTracker(clusteredDetections, currentTime, rangeResoultion, aoaResolution)
    % clusteredDetections现在是3xN矩阵: [距离; 角度; 径向速度]
    numDetections = size(clusteredDetections, 2);
    
    % 创建objectDetection对象数组
    detections = cell(numDetections, 1);
    
    for i = 1:numDetections
        % 测量向量: [距离; 角度; 径向速度]
        if size(clusteredDetections, 1) >= 3
            measurement = clusteredDetections(:, i);
        else
            % 如果没有径向速度，只使用距离和角度
            measurement = [clusteredDetections(:, i); 0];
        end
        
        % 测量噪声协方差矩阵
        measurementNoise = diag([
            (rangeResoultion/4)^2;      % 距离噪声
            (aoaResolution/12)^2;        % 角度噪声
            0.5^2                        % 径向速度噪声
        ]);
        
        % 创建objectDetection对象
        detections{i} = objectDetection(currentTime, measurement, ...
            'MeasurementNoise', measurementNoise, ...
            'SensorIndex', 1);
    end
end


