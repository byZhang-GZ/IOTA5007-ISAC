% 自适应过程噪声效果可视化
% 显示IMM模型概率、机动检测和Q增强系数的时间演化

% 加载跟踪结果
load('results/trackingResults.mat');

% ========== 调试信息：检查加载的变量 ==========
fprintf('\n=== Loaded Variables Check ===\n');
fprintf('Variables in workspace: ');
whos_output = whos;
fprintf('%s ', whos_output.name);
fprintf('\n');

if exist('targetTrajectories', 'var')
    fprintf('✓ targetTrajectories exists (%d trajectories)\n', length(targetTrajectories));
    for i = 1:length(targetTrajectories)
        fprintf('  - Trajectory %d: TimeRange = [%.2f, %.2f] s\n', ...
                i, targetTrajectories{i}.TimeOfArrival(1), targetTrajectories{i}.TimeOfArrival(end));
    end
else
    fprintf('✗ targetTrajectories NOT FOUND in results file!\n');
    fprintf('  Please re-run Channel_Simulation_and_Sensing_Data_Processing.m\n');
end
fprintf('=============================\n\n');

% 为每个跟踪目标创建可视化
numTracks = length(trackIDs);

% 速度过滤阈值和匹配阈值
VELOCITY_THRESHOLD = 2.0;  % m/s, 低于此速度视为静止杂波
DISTANCE_THRESHOLD = 50.0; % m, 匹配真值轨迹的最大距离（考虑双基地雷达测距误差）

for iTrack = 1:numTracks
    trackID = trackIDs(iTrack);
    
    % 提取该航迹的数据
    modelProb = squeeze(modelProbabilities(:, :, iTrack));
    
    % 检查是否有有效数据
    validIdx = ~all(isnan(modelProb), 1);
    if ~any(validIdx)
        fprintf('Track %d has no valid data, skipping\n', trackID);
        continue;
    end
    
    % ========== 速度过滤：排除静止杂波 ==========
    % 提取速度分量 (vx, vy 在状态向量的第2和第4个位置)
    vx = squeeze(targetStateEstimates(2, :, iTrack));
    vy = squeeze(targetStateEstimates(4, :, iTrack));
    
    % 计算有效时刻的速度
    validVelIdx = ~isnan(vx) & ~isnan(vy);
    if any(validVelIdx)
        speeds = sqrt(vx(validVelIdx).^2 + vy(validVelIdx).^2);
        avgSpeed = mean(speeds);
        
        % 过滤静止航迹
        if avgSpeed < VELOCITY_THRESHOLD
            fprintf('⊘ Skipping static track ID %d (avg speed: %.2f m/s < %.1f m/s)\n', ...
                    trackID, avgSpeed, VELOCITY_THRESHOLD);
            continue;
        end
    else
        fprintf('Track %d has no valid velocity data, skipping\n', trackID);
        continue;
    end
    
    % 重构机动概率和增强系数（基于保存的模型概率重新计算）
    maneuverProb = modelProb(2, :) + modelProb(3, :);  % CT + CA
    
    % 机动检测参数
    maneuverThreshold = 0.6;  % 与实际跟踪参数保持一致
    maneuverRegion = maneuverProb > maneuverThreshold;
    
    % 找到机动区间
    maneuverStarts = find(diff([0, maneuverRegion]) == 1);
    maneuverEnds = find(diff([maneuverRegion, 0]) == -1);
    
    % ========== Figure 1: IMM模型概率演化 ==========
    figure('Name', sprintf('Track %d - Model Probabilities', trackID), ...
           'Position', [100, 500, 600, 400]);
    hold on;
    plot(t, modelProb(1, :), 'b-', 'LineWidth', 2, 'DisplayName', 'CV Model');
    plot(t, modelProb(2, :), 'r-', 'LineWidth', 2, 'DisplayName', 'CT Model');
    plot(t, modelProb(3, :), 'g-', 'LineWidth', 2, 'DisplayName', 'CA Model');
    
    % 标注机动区域（CT+CA > 阈值）
    ylim([0, 1]);
    yLimits = ylim;
    for k = 1:length(maneuverStarts)
        tStart = t(maneuverStarts(k));
        tEnd = t(maneuverEnds(k));
        fill([tStart, tEnd, tEnd, tStart], ...
             [yLimits(1), yLimits(1), yLimits(2), yLimits(2)], ...
             [1, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
             'DisplayName', 'Maneuver Detection Region');
    end
    
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('Model Probability', 'FontName', 'Times New Roman', 'FontSize', 12);
    title(sprintf('Track %d - IMM Model Probability Evolution', trackID), ...
          'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontName', 'Times New Roman', 'FontSize', 11);
    grid on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
    hold off;
    
    % ========== Figure 2: 机动检测逻辑 ==========
    figure('Name', sprintf('Track %d - Maneuver Logic', trackID), ...
           'Position', [750, 500, 600, 400]);
    hold on;
    plot(t, maneuverProb, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Maneuver Probability (CT+CA)');
    yline(maneuverThreshold, 'r--', 'LineWidth', 2, 'DisplayName', 'Trigger Threshold');
    yline(maneuverThreshold * 0.7, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Exit Threshold (Hysteresis)');
    
    % 标注机动区域
    ylim([0, 1]);
    yLimits = ylim;
    for k = 1:length(maneuverStarts)
        tStart = t(maneuverStarts(k));
        tEnd = t(maneuverEnds(k));
        fill([tStart, tEnd, tEnd, tStart], ...
             [yLimits(1), yLimits(1), yLimits(2), yLimits(2)], ...
             [1, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('Probability', 'FontName', 'Times New Roman', 'FontSize', 12);
    title(sprintf('Track %d - Maneuver Detection Logic', trackID), ...
          'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontName', 'Times New Roman', 'FontSize', 11);
    grid on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
    hold off;
    
    % ========== Figure 3: 轨迹对比（智能真值匹配）==========
    figure('Name', sprintf('Track %d - Trajectory', trackID), ...
           'Position', [1400, 500, 600, 400]);
    hold on;
    
    % 提取估计轨迹
    estPos = squeeze(targetStateEstimates([1, 3], :, iTrack))';
    validEstIdx = ~isnan(estPos(:, 1));
    
    % ========== 智能真值匹配：使用最近邻算法 ==========
    matchedTruth = false;
    fprintf('\n--- Matching Track %d to ground truth ---\n', trackID);
    
    if exist('targetTrajectories', 'var') && ~isempty(targetTrajectories)
        % 计算估计轨迹的所有有效点与每条真值轨迹的平均距离
        numTrueTraj = length(targetTrajectories);
        avgDistances = inf(numTrueTraj, 1);
        
        fprintf('  Time range: [%.2f, %.2f] s\n', min(t(validEstIdx)), max(t(validEstIdx)));
        fprintf('  Valid estimation points: %d\n', sum(validEstIdx));
        
        for trajIdx = 1:numTrueTraj
            trueTraj = targetTrajectories{trajIdx};
            distances = [];
            errorCount = 0;
            
            % 遍历所有有效的估计点
            for k = find(validEstIdx)'
                try
                    [truePos, ~, ~] = lookupPose(trueTraj, t(k));
                    dist = norm(estPos(k, :) - truePos(1:2)');
                    distances = [distances, dist];
                catch ME
                    % 如果真值轨迹在该时刻不存在，跳过
                    errorCount = errorCount + 1;
                end
            end
            
            % 计算平均距离
            if ~isempty(distances)
                avgDistances(trajIdx) = mean(distances);
                fprintf('  Target %d: %d valid points, avg dist = %.2f m\n', ...
                        trajIdx, length(distances), avgDistances(trajIdx));
            else
                fprintf('  Target %d: NO valid overlap (all lookupPose failed)\n', trajIdx);
            end
        end
        
        % 找到最近的真值轨迹
        [minAvgDist, bestTrajIdx] = min(avgDistances);
        
        fprintf('  Best match: Target %d (dist = %.2f m, threshold = %.1f m)\n', ...
                bestTrajIdx, minAvgDist, DISTANCE_THRESHOLD);
        
        % 只有当距离小于阈值时才绘制真值轨迹
        if minAvgDist < DISTANCE_THRESHOLD
            trueTraj = targetTrajectories{bestTrajIdx};
            truePos = zeros(length(t), 2);
            for k = 1:length(t)
                try
                    [pos, ~, ~] = lookupPose(trueTraj, t(k));
                    truePos(k, :) = pos(1:2);
                catch
                    truePos(k, :) = [NaN, NaN];
                end
            end
            validTrueIdx = ~isnan(truePos(:, 1));
            fprintf('  Drawing ground truth: %d points\n', sum(validTrueIdx));
            plot(truePos(validTrueIdx, 1), truePos(validTrueIdx, 2), 'k--', ...
                 'LineWidth', 2.5, 'DisplayName', sprintf('Ground Truth (Target %d)', bestTrajIdx));
            matchedTruth = true;
            fprintf('✓ Track %d matched to Target %d (avg dist: %.2f m)\n', ...
                    trackID, bestTrajIdx, minAvgDist);
        else
            fprintf('⚠ Track %d: No close ground truth found (min dist: %.2f m > %.1f m)\n', ...
                    trackID, minAvgDist, DISTANCE_THRESHOLD);
        end
    else
        fprintf('⚠ targetTrajectories variable not available!\n');
    end
    
    % 用颜色表示时间/机动状态
    scatter(estPos(validEstIdx, 1), estPos(validEstIdx, 2), 50, t(validEstIdx)', 'filled', ...
            'DisplayName', sprintf('Estimated Track %d', trackID));
    colorbar;
    colormap(jet);
    caxis([min(t), max(t)]);
    ylabel(colorbar, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 11);
    
    % 标注机动段
    for k = 1:length(maneuverStarts)
        maneuverIdx = maneuverStarts(k):maneuverEnds(k);
        validManeuverIdx = maneuverIdx(validEstIdx(maneuverIdx));
        if ~isempty(validManeuverIdx)
            plot(estPos(validManeuverIdx, 1), estPos(validManeuverIdx, 2), 'ro', ...
                 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
        end
    end
    
    xlabel('X Position (m)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontName', 'Times New Roman', 'FontSize', 12);
    if matchedTruth
        title(sprintf('Track %d - Trajectory Comparison (Avg Speed: %.1f m/s)', trackID, avgSpeed), ...
              'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    else
        title(sprintf('Track %d - Estimated Trajectory Only (Avg Speed: %.1f m/s)', trackID, avgSpeed), ...
              'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    end
    legend('Location', 'best', 'FontName', 'Times New Roman', 'FontSize', 11);
    grid on;
    axis equal;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
    hold off;
end

%% 统计分析
fprintf('\n=== Adaptive Process Noise Statistical Analysis ===\n');
fprintf('(Filtering static tracks with avg speed < %.1f m/s)\n\n', VELOCITY_THRESHOLD);

for iTrack = 1:numTracks
    trackID = trackIDs(iTrack);
    modelProb = squeeze(modelProbabilities(:, :, iTrack));
    
    % 跳过无效数据
    if all(isnan(modelProb(:)))
        continue;
    end
    
    % 速度过滤检查
    vx = squeeze(targetStateEstimates(2, :, iTrack));
    vy = squeeze(targetStateEstimates(4, :, iTrack));
    validVelIdx = ~isnan(vx) & ~isnan(vy);
    if any(validVelIdx)
        speeds = sqrt(vx(validVelIdx).^2 + vy(validVelIdx).^2);
        avgSpeed = mean(speeds);
        if avgSpeed < VELOCITY_THRESHOLD
            continue;  % 跳过静止航迹的统计
        end
    else
        continue;
    end
    
    maneuverProb = modelProb(2, :) + modelProb(3, :);
    maneuverFrames = sum(maneuverProb > 0.4);
    totalFrames = sum(~isnan(maneuverProb));
    
    fprintf('Track %d (Avg Speed: %.2f m/s):\n', trackID, avgSpeed);
    fprintf('  Total frames: %d\n', totalFrames);
    fprintf('  Maneuver frames: %d (%.1f%%)\n', maneuverFrames, 100*maneuverFrames/totalFrames);
    fprintf('  Average CV probability: %.3f\n', mean(modelProb(1, ~isnan(modelProb(1, :)))));
    fprintf('  Average CT probability: %.3f\n', mean(modelProb(2, ~isnan(modelProb(2, :)))));
    fprintf('  Average CA probability: %.3f\n', mean(modelProb(3, ~isnan(modelProb(3, :)))));
    fprintf('  Max maneuver probability: %.3f\n', max(maneuverProb));
    fprintf('\n');
end
