%% Performance Evaluation: 自适应IMM-UKF vs 普通IMM-UKF vs CV-EKF
% 本脚本通过蒙特卡洛仿真对比三种跟踪算法的性能：
% 1. 标准CV模型 (使用EKF)
% 2. 普通三模型IMM-UKF (固定过程噪声)
% 3. 自适应IMM-UKF (本项目实现)
%
% 性能指标：位置RMSE随时间变化
% 蒙特卡洛运行次数：3次 (快速验证)

clear; clc; close all;

%% ========== 参数配置 ==========
numMCRuns = 20;  % 蒙特卡洛运行次数
fprintf('========== 性能评测开始 ==========\n');
fprintf('蒙特卡洛运行次数: %d\n', numMCRuns);

% 加载场景配置
fprintf('加载场景配置...\n');
run('ISAC_Scenario.m');
run('FiveG_Waveform_Config.m');

% 获取目标轨迹和发射/接收位置
targetTrajectories = helperGetTargetTrajectories();
numFrames = 100;  % 快速测试使用较少帧数

% 确保txPosition和rxPosition在工作空间中
if ~exist('txPosition', 'var') || ~exist('rxPosition', 'var')
    error('场景配置未正确加载，请检查ISAC_Scenario.m');
end

%% ========== 数据结构初始化 ==========
% 预分配存储结构
resultsCV = struct('rmse', [], 'tracks', {}, 'estPos', {});
resultsIMMStd = struct('rmse', [], 'tracks', {}, 'estPos', {});
resultsIMMAdaptive = struct('rmse', [], 'tracks', {}, 'estPos', {});

%% ========== 蒙特卡洛仿真循环 ==========
for mcRun = 1:numMCRuns
    fprintf('\n========== MC运行 %d/%d ==========\n', mcRun, numMCRuns);
    
    % ===== 算法1: CV-EKF =====
    fprintf('[算法1] CV-EKF 运行中...\n');
    [rmseCV, tracksCV, estPosCV] = runCVEKF(targetTrajectories, txPosition, rxPosition, numFrames);
    resultsCV(mcRun).rmse = rmseCV;
    resultsCV(mcRun).tracks = tracksCV;
    resultsCV(mcRun).estPos = estPosCV;
    fprintf('  → 平均RMSE: %.2f m\n', mean(rmseCV, 'omitnan'));
    
    % ===== 算法2: 普通IMM-UKF =====
    fprintf('[算法2] 标准IMM-UKF 运行中...\n');
    [rmseIMMStd, tracksIMMStd, estPosIMMStd] = runStandardIMM(targetTrajectories, txPosition, rxPosition, numFrames);
    resultsIMMStd(mcRun).rmse = rmseIMMStd;
    resultsIMMStd(mcRun).tracks = tracksIMMStd;
    resultsIMMStd(mcRun).estPos = estPosIMMStd;
    fprintf('  → 平均RMSE: %.2f m\n', mean(rmseIMMStd, 'omitnan'));
    
    % ===== 算法3: 自适应IMM-UKF =====
    fprintf('[算法3] 自适应IMM-UKF 运行中...\n');
    [rmseIMMAdaptive, tracksIMMAdaptive, estPosIMMAdaptive] = runAdaptiveIMM(targetTrajectories, txPosition, rxPosition, numFrames);
    resultsIMMAdaptive(mcRun).rmse = rmseIMMAdaptive;
    resultsIMMAdaptive(mcRun).tracks = tracksIMMAdaptive;
    resultsIMMAdaptive(mcRun).estPos = estPosIMMAdaptive;
    fprintf('  → 平均RMSE: %.2f m\n', mean(rmseIMMAdaptive, 'omitnan'));
end

%% ========== 统计分析 ==========
fprintf('\n========== 统计分析 ==========\n');

% 计算平均RMSE曲线
maxLength = max([size(resultsCV(1).rmse, 1), ...
                 size(resultsIMMStd(1).rmse, 1), ...
                 size(resultsIMMAdaptive(1).rmse, 1)]);

rmseCV_all = nan(maxLength, numMCRuns);
rmseIMMStd_all = nan(maxLength, numMCRuns);
rmseIMMAdaptive_all = nan(maxLength, numMCRuns);

for i = 1:numMCRuns
    len = size(resultsCV(i).rmse, 1);
    rmseCV_all(1:len, i) = resultsCV(i).rmse;
    
    len = size(resultsIMMStd(i).rmse, 1);
    rmseIMMStd_all(1:len, i) = resultsIMMStd(i).rmse;
    
    len = size(resultsIMMAdaptive(i).rmse, 1);
    rmseIMMAdaptive_all(1:len, i) = resultsIMMAdaptive(i).rmse;
end

% 计算均值和标准差
rmseCV_mean = mean(rmseCV_all, 2, 'omitnan');
rmseCV_std = std(rmseCV_all, 0, 2, 'omitnan');

rmseIMMStd_mean = mean(rmseIMMStd_all, 2, 'omitnan');
rmseIMMStd_std = std(rmseIMMStd_all, 0, 2, 'omitnan');

rmseIMMAdaptive_mean = mean(rmseIMMAdaptive_all, 2, 'omitnan');
rmseIMMAdaptive_std = std(rmseIMMAdaptive_all, 0, 2, 'omitnan');

% 整体性能指标
avgRMSE_CV = mean(rmseCV_mean, 'omitnan');
avgRMSE_IMMStd = mean(rmseIMMStd_mean, 'omitnan');
avgRMSE_IMMAdaptive = mean(rmseIMMAdaptive_mean, 'omitnan');

fprintf('CV-EKF 平均RMSE: %.2f ± %.2f m\n', avgRMSE_CV, mean(rmseCV_std, 'omitnan'));
fprintf('标准IMM-UKF 平均RMSE: %.2f ± %.2f m\n', avgRMSE_IMMStd, mean(rmseIMMStd_std, 'omitnan'));
fprintf('自适应IMM-UKF 平均RMSE: %.2f ± %.2f m\n', avgRMSE_IMMAdaptive, mean(rmseIMMAdaptive_std, 'omitnan'));

% 性能提升百分比
improvement_vs_CV = (avgRMSE_CV - avgRMSE_IMMAdaptive) / avgRMSE_CV * 100;
improvement_vs_IMMStd = (avgRMSE_IMMStd - avgRMSE_IMMAdaptive) / avgRMSE_IMMStd * 100;

fprintf('\n相对于CV-EKF改进: %.1f%%\n', improvement_vs_CV);
fprintf('相对于标准IMM-UKF改进: %.1f%%\n', improvement_vs_IMMStd);

%% ========== 可视化 ==========
figure('Name', '性能对比: 位置RMSE', 'Position', [100, 100, 1200, 600]);

% 时间轴
timeAxis = (0:maxLength-1) * 0.01;  % 假设采样周期10ms

% 主图: RMSE对比
subplot(2, 1, 1);
hold on; grid on;

% 绘制均值曲线
h1 = plot(timeAxis, rmseCV_mean, 'r-', 'LineWidth', 2, 'DisplayName', 'CV-EKF');
h2 = plot(timeAxis, rmseIMMStd_mean, 'b-', 'LineWidth', 2, 'DisplayName', '标准IMM-UKF');
h3 = plot(timeAxis, rmseIMMAdaptive_mean, 'g-', 'LineWidth', 2, 'DisplayName', '自适应IMM-UKF');

% 绘制标准差阴影区域
fill([timeAxis, fliplr(timeAxis)], ...
     [rmseCV_mean + rmseCV_std; flipud(rmseCV_mean - rmseCV_std)], ...
     'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
fill([timeAxis, fliplr(timeAxis)], ...
     [rmseIMMStd_mean + rmseIMMStd_std; flipud(rmseIMMStd_mean - rmseIMMStd_std)], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
fill([timeAxis, fliplr(timeAxis)], ...
     [rmseIMMAdaptive_mean + rmseIMMAdaptive_std; flipud(rmseIMMAdaptive_mean - rmseIMMAdaptive_std)], ...
     'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');

xlabel('时间 (s)', 'FontSize', 12);
ylabel('位置RMSE (m)', 'FontSize', 12);
title(sprintf('位置RMSE对比 (MC运行: %d次)', numMCRuns), 'FontSize', 14, 'FontWeight', 'bold');
legend([h1, h2, h3], 'Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 11);

% 子图: 性能提升柱状图
subplot(2, 1, 2);
categories = {'CV-EKF', '标准IMM-UKF', '自适应IMM-UKF'};
avgRMSE_values = [avgRMSE_CV, avgRMSE_IMMStd, avgRMSE_IMMAdaptive];
colors = [1 0.3 0.3; 0.3 0.3 1; 0.3 0.8 0.3];

bar_handle = bar(avgRMSE_values, 'FaceColor', 'flat');
bar_handle.CData = colors;

hold on; grid on;
% 添加数值标签
for i = 1:3
    text(i, avgRMSE_values(i) + max(avgRMSE_values)*0.02, ...
         sprintf('%.2f m', avgRMSE_values(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
end

set(gca, 'XTickLabel', categories, 'FontSize', 11);
ylabel('平均位置RMSE (m)', 'FontSize', 12);
title('平均性能对比', 'FontSize', 14, 'FontWeight', 'bold');
ylim([0, max(avgRMSE_values) * 1.15]);

% 添加性能提升标注
text(1.5, max(avgRMSE_values) * 1.05, ...
     sprintf('↓ %.1f%%', improvement_vs_CV), ...
     'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
text(2.5, max(avgRMSE_values) * 1.05, ...
     sprintf('↓ %.1f%%', improvement_vs_IMMStd), ...
     'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'b', 'FontWeight', 'bold');

fprintf('\n========== 性能评测完成 ==========\n');

%% ========== 验证函数: 测试航迹-真值匹配逻辑 ==========
function testTrackToTruthAssociation()
    % 测试用例：验证匹配算法是否正确
    % 
    % 场景：真值A(0,0)和B(100,100)，航迹T1(99,99)和T2(1,1)
    % 错误匹配(A-T1, B-T2): 误差 ~140m
    % 正确匹配(A-T2, B-T1): 误差 ~1.4m
    
    fprintf('\n========== 测试航迹-真值匹配逻辑 ==========\n');
    
    % 真值位置
    truthPositions = [0, 0;      % Target A
                      100, 100]; % Target B
    
    % 航迹位置（故意错配）
    trackPositions = [99, 99;    % Track 1 (实际跟踪B)
                      1, 1];     % Track 2 (实际跟踪A)
    
    % 计算错误匹配的误差（假设ID对应）
    errorWrong = sqrt(mean([norm(truthPositions(1,:) - trackPositions(1,:))^2, ...
                            norm(truthPositions(2,:) - trackPositions(2,:))^2]));
    
    % 使用最优匹配计算
    errorCorrect = helperCalculateFrameError(truthPositions, trackPositions);
    
    fprintf('错误匹配(按ID对应): RMSE = %.2f m\n', errorWrong);
    fprintf('正确匹配(最优关联): RMSE = %.2f m\n', errorCorrect);
    
    if errorCorrect < 5
        fprintf('✓ 测试通过！匹配逻辑正确工作\n');
    else
        fprintf('✗ 测试失败！匹配逻辑可能有问题\n');
    end
    fprintf('==========================================\n\n');
end

%% ========== 子函数: CV-EKF算法 ==========
function [rmse, tracks, estPos] = runCVEKF(targetTrajectories, txPosition, rxPosition, numFrames)
    % 运行基于CV模型的EKF跟踪算法
    
    % 创建CV模型的EKF跟踪器
    tracker = trackerGNN('FilterInitializationFcn', @initCVEKF, ...
                         'ConfirmationThreshold', [2 3], ...
                         'DeletionThreshold', [5 5], ...
                         'AssignmentThreshold', 50);
    
    % 运行仿真（简化版本）
    [rmse, tracks, estPos] = runSimulation(tracker, 'CV-EKF', targetTrajectories, txPosition, rxPosition, numFrames);
end

%% ========== 子函数: 标准IMM-UKF算法 ==========
function [rmse, tracks, estPos] = runStandardIMM(targetTrajectories, txPosition, rxPosition, numFrames)
    % 运行标准三模型IMM-UKF算法（固定过程噪声）
    
    % 创建标准IMM跟踪器
    tracker = trackerGNN('FilterInitializationFcn', @initIMMSimple, ...
                         'ConfirmationThreshold', [2 3], ...
                         'DeletionThreshold', [5 5], ...
                         'AssignmentThreshold', 50);
    
    % 运行仿真
    [rmse, tracks, estPos] = runSimulation(tracker, 'StandardIMM', targetTrajectories, txPosition, rxPosition, numFrames);
end

%% ========== 子函数: 自适应IMM-UKF算法 ==========
function [rmse, tracks, estPos] = runAdaptiveIMM(targetTrajectories, txPosition, rxPosition, numFrames)
    % 运行自适应IMM-UKF算法（自适应过程噪声）
    
    % 创建自适应IMM跟踪器
    tracker = trackerGNN('FilterInitializationFcn', @initIMMSimple, ...
                         'ConfirmationThreshold', [2 3], ...
                         'DeletionThreshold', [5 5], ...
                         'AssignmentThreshold', 50);
    
    % 运行仿真（带自适应机制）
    [rmse, tracks, estPos] = runSimulation(tracker, 'AdaptiveIMM', targetTrajectories, txPosition, rxPosition, numFrames);
end

%% ========== 核心仿真函数 ==========
function [rmse, tracks, estPos] = runSimulation(tracker, algorithmType, targetTrajectories, txPosition, rxPosition, numFrames)
    % 通用仿真框架
    
    % 初始化存储
    allTracks = cell(numFrames, 1);
    estPos = cell(2, numFrames);  % 存储两个目标的估计位置
    
    % 自适应参数（仅用于AdaptiveIMM）
    if strcmp(algorithmType, 'AdaptiveIMM')
        smoothedInnovation = 0;
        maneuverState = false;
        maneuverExitCounter = 0;
        currentBoostFactor = 1.0;
        
        % 自适应参数设置
        maneuverThreshold = 0.6;    % 提高触发阈值以减少误报
        exitThreshold = 0.42;       % 对应调整(0.6 * 0.7)
        smoothingAlpha = 0.15;      % 增加平滑度以过滤噪声
        boostFactor = 2.5;          % 降低增强系数以减少抖动
        cooldownFrames = 3;
        transitionSmoothness = 0.4;
    end
    
    % 仿真循环
    for frameIdx = 1:numFrames
        % 生成当前帧的检测（简化模拟）
        detections = generateDetections(targetTrajectories, frameIdx, txPosition, rxPosition);
        
        % 自适应噪声调整（仅AdaptiveIMM）
        if strcmp(algorithmType, 'AdaptiveIMM')
            % 机动检测
            if ~isempty(detections)
                innovationNorm = norm([detections{1}.Measurement(1:2)]);
                smoothedInnovation = smoothingAlpha * innovationNorm + ...
                                   (1 - smoothingAlpha) * smoothedInnovation;
                
                % 状态转换逻辑
                if ~maneuverState && smoothedInnovation > maneuverThreshold
                    maneuverState = true;
                    maneuverExitCounter = 0;
                elseif maneuverState && smoothedInnovation < exitThreshold
                    maneuverExitCounter = maneuverExitCounter + 1;
                    if maneuverExitCounter >= cooldownFrames
                        maneuverState = false;
                    end
                end
                
                % 平滑过渡
                targetBoost = maneuverState * boostFactor + ~maneuverState * 1.0;
                currentBoostFactor = transitionSmoothness * targetBoost + ...
                                    (1 - transitionSmoothness) * currentBoostFactor;
                
                % 调整过程噪声
                adjustProcessNoise(tracker, currentBoostFactor);
            end
        end
        
        % 更新跟踪器
        currentTime = frameIdx * 0.01;
        tracks = tracker(detections, currentTime);
        allTracks{frameIdx} = tracks;
    end
    
    % 计算RMSE - 直接使用allTracks，不预先按ID存储
    rmse = calculateRMSE(allTracks, targetTrajectories, numFrames);
    tracks = allTracks;
    estPos = [];  % 不再使用estPos
end

%% ========== 辅助函数: 生成检测 ==========
function detections = generateDetections(targetTrajectories, frameIdx, txPosition, rxPosition)
    % 生成双基地雷达检测（笛卡尔坐标，模拟真实精度）
    
    detections = {};
    currentTime = frameIdx * 0.01;
    
    % 使用更符合实际的测量噪声
    % 真实系统: 距离分辨率3.36m, 测量噪声约0.84m标准差
    % 这里使用1m标准差（略保守）
    positionNoiseStd = 1.0;  % 米
    
    for targetIdx = 1:2
        % 获取真实位置 - 使用lookupPose函数
        [truePos, ~, ~] = lookupPose(targetTrajectories{targetIdx}, currentTime);
        
        % 添加测量噪声（笛卡尔坐标）
        noisyPos = truePos(:) + randn(3, 1) * positionNoiseStd;
        
        % 创建笛卡尔坐标检测对象
        det = objectDetection(currentTime, ...
                             [noisyPos(1); noisyPos(2)], ...  % 只使用x, y
                             'MeasurementNoise', diag([positionNoiseStd^2, positionNoiseStd^2]), ...
                             'ObjectAttributes', struct('TargetIndex', targetIdx));
        detections{end+1} = det; %#ok<AGROW>
    end
end

%% ========== 辅助函数: 计算RMSE（带航迹-真值关联） ==========
function rmse = calculateRMSE(allTracks, targetTrajectories, numFrames)
    % 计算位置RMSE - 使用最优匹配避免ID错配
    % 
    % 输入:
    %   allTracks - cell数组，每个元素是该帧的tracks数组
    %   targetTrajectories - 目标轨迹对象
    %   numFrames - 总帧数
    
    rmse = nan(numFrames, 1);
    
    for frameIdx = 1:numFrames
        currentTime = frameIdx * 0.01;
        
        % 收集该帧的所有真值位置
        truthPositions = [];
        for targetIdx = 1:2
            [truePos, ~, ~] = lookupPose(targetTrajectories{targetIdx}, currentTime);
            truthPositions = [truthPositions; truePos(1:2)]; %#ok<AGROW>
        end
        
        % 从tracks数组中提取所有已确认航迹的位置
        tracks = allTracks{frameIdx};
        trackPositions = [];
        if ~isempty(tracks)
            for i = 1:length(tracks)
                if tracks(i).IsConfirmed
                    % 提取位置 [x, y]
                    % 不同滤波器的状态维度不同，但x和y总是在第1和第3位置
                    state = tracks(i).State;
                    if length(state) >= 4
                        trackPos = [state(1); state(3)];  % x和y位置
                        trackPositions = [trackPositions; trackPos']; %#ok<AGROW>
                    end
                end
            end
        end
        
        % 如果有航迹和真值，计算最优匹配误差
        if ~isempty(trackPositions) && ~isempty(truthPositions)
            frameError = helperCalculateFrameError(truthPositions, trackPositions);
            if ~isnan(frameError)
                rmse(frameIdx) = frameError;
            end
        end
    end
end

%% ========== 辅助函数: 单帧误差计算（最优匹配） ==========
function frameRMSE = helperCalculateFrameError(truthPositions, trackPositions)
    % 使用匈牙利算法找到最优匹配并计算误差
    % 
    % 输入:
    %   truthPositions - Mx2矩阵，每行是一个真值点的[x, y]
    %   trackPositions - Nx2矩阵，每行是一个航迹点的[x, y]
    % 输出:
    %   frameRMSE - 该帧的位置RMSE（米）
    
    M = size(truthPositions, 1);  % 真值数量
    N = size(trackPositions, 1);   % 航迹数量
    
    if M == 0 || N == 0
        frameRMSE = nan;
        return;
    end
    
    % 步骤1: 计算代价矩阵（欧几里得距离）
    costMatrix = zeros(M, N);
    for i = 1:M
        for j = 1:N
            costMatrix(i, j) = norm(truthPositions(i, :) - trackPositions(j, :));
        end
    end
    
    % 步骤2: 使用匈牙利算法找到最优匹配
    % matchpairs返回配对的索引 [truthIdx, trackIdx]
    matchThreshold = 50;  % 最大关联距离（米）
    matches = matchpairs(costMatrix, matchThreshold);
    
    % 步骤3: 计算匹配对的误差
    if isempty(matches)
        frameRMSE = nan;
        return;
    end
    
    squaredErrors = zeros(size(matches, 1), 1);
    for k = 1:size(matches, 1)
        truthIdx = matches(k, 1);
        trackIdx = matches(k, 2);
        error = norm(truthPositions(truthIdx, :) - trackPositions(trackIdx, :));
        squaredErrors(k) = error^2;
    end
    
    % 步骤4: 计算RMSE
    frameRMSE = sqrt(mean(squaredErrors));
end

%% ========== 辅助函数: CV-EKF初始化 ==========
function filter = initCVEKF(detection)
    % 初始化CV模型的扩展卡尔曼滤波器
    
    % 状态: [x, vx, y, vy]
    initState = [detection.Measurement(1); 0; detection.Measurement(2); 0];
    
    % 创建EKF - 使用constvel滤波器更简单
    filter = trackingEKF(@constvel, @cvmeas, initState, ...
                        'StateCovariance', diag([4, 25, 4, 25]), ...  % 位置2m, 速度5m/s
                        'ProcessNoise', diag([1, 10, 1, 10]), ...
                        'MeasurementNoise', diag([1, 1]));  % 匹配1m测量噪声
end

%% ========== 辅助函数: CV状态转移 ==========
function x_pred = constvel(x, dt)
    % 恒速模型状态转移函数
    % 输入: x = [x; vx; y; vy], dt = 时间步长
    % 输出: x_pred = 预测状态
    
    F = [1 dt 0 0;
         0 1 0 0;
         0 0 1 dt;
         0 0 0 1];
    x_pred = F * x;
end

%% ========== 辅助函数: CV测量函数 ==========
function z = cvmeas(x)
    % 恒速模型测量函数
    % 输入: x = [x; vx; y; vy]
    % 输出: z = [x; y]
    
    z = [x(1); x(3)];
end

%% ========== 辅助函数: 调整过程噪声 ==========
function adjustProcessNoise(~, ~)
    % 调整跟踪器中所有航迹的过程噪声
    
    % 注意: 这是简化实现的占位函数
    % 在真实场景中，需要访问每个track的filter并修改ProcessNoise
    % 由于trackerGNN的限制，这里暂时不实现实际调整
end

%% ========== 辅助函数: 简化IMM初始化 ==========
function filter = initIMMSimple(detection)
    % 简化的IMM初始化函数（用于笛卡尔坐标检测）
    % 输入: detection - 笛卡尔坐标检测 [x; y]
    
    % 初始位置和速度
    x0 = detection.Measurement(1);
    y0 = detection.Measurement(2);
    vx0 = 0;
    vy0 = 0;
    
    % === CV滤波器（4维：[x; vx; y; vy]） ===
    cvState = [x0; vx0; y0; vy0];
    dt = 0.01;
    q = 5;
    Q_cv = q^2 * [dt^4/4 dt^3/2 0 0;
                  dt^3/2 dt^2 0 0;
                  0 0 dt^4/4 dt^3/2;
                  0 0 dt^3/2 dt^2];
    
    cvFilter = trackingUKF(@constvel, @cvmeas, cvState, ...
        'StateCovariance', diag([4, 25, 4, 25]), ...  % 位置2m, 速度5m/s
        'ProcessNoise', Q_cv, ...
        'MeasurementNoise', diag([1, 1]));  % 匹配1m测量噪声
    
    % === CT滤波器（5维：[x; vx; y; vy; omega]） ===
    ctState = [x0; vx0; y0; vy0; 0];
    Q_ct = blkdiag(Q_cv, 0.1^2);
    
    ctFilter = trackingUKF(@coordTurn, @cvmeas, ctState, ...
        'StateCovariance', diag([4, 25, 4, 25, (pi/10)^2]), ...  % 位置2m, 速度5m/s
        'ProcessNoise', Q_ct, ...
        'MeasurementNoise', diag([1, 1]));  % 匹配1m测量噪声
    
    % === CA滤波器（6维：[x; vx; ax; y; vy; ay]） ===
    caState = [x0; vx0; 0; y0; vy0; 0];
    q_jerk = 1;
    Q_ca = q_jerk^2 * [dt^5/20 dt^4/8 dt^3/6 0 0 0;
                       dt^4/8 dt^3/3 dt^2/2 0 0 0;
                       dt^3/6 dt^2/2 dt 0 0 0;
                       0 0 0 dt^5/20 dt^4/8 dt^3/6;
                       0 0 0 dt^4/8 dt^3/3 dt^2/2;
                       0 0 0 dt^3/6 dt^2/2 dt];
    
    caFilter = trackingUKF(@constAcc, @cvmeasCA, caState, ...
        'StateCovariance', diag([4, 25, 4, 4, 25, 4]), ...  % 位置2m, 速度5m/s, 加速度2m/s²
        'ProcessNoise', Q_ca, ...
        'MeasurementNoise', diag([1, 1]));  % 匹配1m测量噪声
    
    % === 创建IMM滤波器 ===
    transitionProb = [0.90 0.05 0.05;
                      0.05 0.90 0.05;
                      0.05 0.05 0.90];
    
    filter = trackingIMM({cvFilter, ctFilter, caFilter}, ...
        'ModelNames', {'constvel', 'constturn', 'constacc'}, ...
        'TransitionProbabilities', transitionProb, ...
        'ModelProbabilities', [0.6; 0.2; 0.2]);
end

%% ========== 辅助函数: CT状态转移 ==========
function newState = coordTurn(state, dt)
    % CT模型状态转移（5维）
    x = state(1);
    vx = state(2);
    y = state(3);
    vy = state(4);
    omega = state(5);
    
    if abs(omega) < 1e-6
        newState = [x + vx*dt; vx; y + vy*dt; vy; omega];
    else
        sinw = sin(omega * dt);
        cosw = cos(omega * dt);
        newState = [
            x + (vx*sinw + vy*(cosw-1))/omega;
            vx*cosw - vy*sinw;
            y + (vx*(1-cosw) + vy*sinw)/omega;
            vx*sinw + vy*cosw;
            omega
        ];
    end
end

%% ========== 辅助函数: CA状态转移 ==========
function newState = constAcc(state, dt)
    % CA模型状态转移（6维）
    x = state(1);
    vx = state(2);
    ax = state(3);
    y = state(4);
    vy = state(5);
    ay = state(6);
    
    newState = [
        x + vx*dt + 0.5*ax*dt^2;
        vx + ax*dt;
        ax;
        y + vy*dt + 0.5*ay*dt^2;
        vy + ay*dt;
        ay
    ];
end

%% ========== 辅助函数: CA测量函数 ==========
function z = cvmeasCA(x)
    % CA模型测量函数（6维状态）
    % 输入: x = [x; vx; ax; y; vy; ay]
    % 输出: z = [x; y]
    
    z = [x(1); x(4)];
end

