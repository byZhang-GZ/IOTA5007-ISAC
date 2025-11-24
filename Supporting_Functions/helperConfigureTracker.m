function tracker = helperConfigureTracker(txPosition, rxPosition, txOrientation, rxOrientation, rangeResolution, aoaResolution)
    % 使用IMM滤波器配置目标跟踪器
    % IMM结合了CV和CT模型，能更好地跟踪机动目标
    
    % 创建初始化函数的句柄，绑定传感器参数
    initFcn = @(detection) helperInitIMM(detection, txPosition, rxPosition, ...
        txOrientation, rxOrientation);
    
    % 创建trackerGNN跟踪器（使用全局最近邻数据关联）
    % 使用FilterInitializationFcn来为每个新航迹初始化IMM滤波器
    tracker = trackerGNN('FilterInitializationFcn', initFcn, ...
        'AssignmentThreshold', [200 inf], ...  % 门限
        'ConfirmationThreshold', [2 3], ...    % 航迹确认：3帧内2次关联（降低阈值）
        'DeletionThreshold', [5 5], ...        % 航迹删除
        'HasCostMatrixInput', false, ...
        'MaxNumTracks', 10, ...
        'MaxNumSensors', 1);
end
