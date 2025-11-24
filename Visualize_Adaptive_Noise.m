% 自适应过程噪声效果可视化
% 显示IMM模型概率、机动检测和Q增强系数的时间演化

% 加载跟踪结果
load('results/trackingResults.mat');

% 为每个跟踪目标创建可视化
numTracks = length(trackIDs);

for iTrack = 1:numTracks
    trackID = trackIDs(iTrack);
    
    % 提取该航迹的数据
    modelProb = squeeze(modelProbabilities(:, :, iTrack));
    
    % 检查是否有有效数据
    validIdx = ~all(isnan(modelProb), 1);
    if ~any(validIdx)
        fprintf('航迹 %d 无有效数据，跳过\n', trackID);
        continue;
    end
    
    % 重构机动概率和增强系数（基于保存的模型概率重新计算）
    maneuverProb = modelProb(2, :) + modelProb(3, :);  % CT + CA
    
    % 创建图形
    figure('Name', sprintf('航迹 %d - 自适应过程噪声分析', trackID), ...
           'Position', [100, 100, 1200, 800]);
    
    % === 子图1: 三模型概率演化 ===
    subplot(3, 1, 1);
    hold on;
    plot(t, modelProb(1, :), 'b-', 'LineWidth', 2, 'DisplayName', 'CV模型');
    plot(t, modelProb(2, :), 'r-', 'LineWidth', 2, 'DisplayName', 'CT模型');
    plot(t, modelProb(3, :), 'g-', 'LineWidth', 2, 'DisplayName', 'CA模型');
    
    % 标注机动区域（CT+CA > 阈值）
    maneuverThreshold = 0.4;
    maneuverRegion = maneuverProb > maneuverThreshold;
    
    % 找到机动区间
    maneuverStarts = find(diff([0, maneuverRegion]) == 1);
    maneuverEnds = find(diff([maneuverRegion, 0]) == -1);
    
    ylim([0, 1]);
    yLimits = ylim;
    for k = 1:length(maneuverStarts)
        tStart = t(maneuverStarts(k));
        tEnd = t(maneuverEnds(k));
        fill([tStart, tEnd, tEnd, tStart], ...
             [yLimits(1), yLimits(1), yLimits(2), yLimits(2)], ...
             [1, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
             'DisplayName', '机动检测区域');
    end
    
    xlabel('时间 (s)');
    ylabel('模型概率');
    title(sprintf('航迹 %d - IMM模型概率演化', trackID));
    legend('Location', 'best');
    grid on;
    hold off;
    
    % === 子图2: 机动概率和阈值 ===
    subplot(3, 1, 2);
    hold on;
    plot(t, maneuverProb, 'k-', 'LineWidth', 2.5, 'DisplayName', '机动概率 (CT+CA)');
    yline(maneuverThreshold, 'r--', 'LineWidth', 2, 'DisplayName', '触发阈值');
    yline(maneuverThreshold * 0.7, 'm--', 'LineWidth', 1.5, 'DisplayName', '退出阈值（迟滞）');
    
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
    
    xlabel('时间 (s)');
    ylabel('概率');
    title('机动检测逻辑');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % === 子图3: 轨迹对比 ===
    subplot(3, 1, 3);
    hold on;
    
    % 提取真实轨迹（如果可用）
    if exist('targets', 'var') && iTrack <= length(targets.Trajectories)
        trueTraj = targets.Trajectories{iTrack};
        truePos = zeros(length(t), 2);
        for k = 1:length(t)
            [pos, ~, ~] = lookupPose(trueTraj, t(k));
            truePos(k, :) = pos(1:2);
        end
        plot(truePos(:, 1), truePos(:, 2), 'k--', 'LineWidth', 2, 'DisplayName', '真实轨迹');
    end
    
    % 提取估计轨迹
    estPos = squeeze(targetStateEstimates([1, 3], :, iTrack))';
    validEstIdx = ~isnan(estPos(:, 1));
    
    % 用颜色表示时间/机动状态
    scatter(estPos(validEstIdx, 1), estPos(validEstIdx, 2), 50, t(validEstIdx)', 'filled', ...
            'DisplayName', '估计轨迹');
    colorbar;
    colormap(jet);
    caxis([min(t), max(t)]);
    ylabel(colorbar, '时间 (s)');
    
    % 标注机动段
    for k = 1:length(maneuverStarts)
        maneuverIdx = maneuverStarts(k):maneuverEnds(k);
        plot(estPos(maneuverIdx, 1), estPos(maneuverIdx, 2), 'ro', ...
             'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    
    xlabel('X 位置 (m)');
    ylabel('Y 位置 (m)');
    title('轨迹对比（红圈标注机动段）');
    legend('Location', 'best');
    grid on;
    axis equal;
    hold off;
    
    % 整体标题
    sgtitle(sprintf('航迹 %d - 自适应过程噪声效果分析', trackID), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

%% 统计分析
fprintf('\n=== 自适应过程噪声统计分析 ===\n');
for iTrack = 1:numTracks
    trackID = trackIDs(iTrack);
    modelProb = squeeze(modelProbabilities(:, :, iTrack));
    
    % 跳过无效数据
    if all(isnan(modelProb(:)))
        continue;
    end
    
    maneuverProb = modelProb(2, :) + modelProb(3, :);
    maneuverFrames = sum(maneuverProb > 0.4);
    totalFrames = sum(~isnan(maneuverProb));
    
    fprintf('航迹 %d:\n', trackID);
    fprintf('  总帧数: %d\n', totalFrames);
    fprintf('  机动帧数: %d (%.1f%%)\n', maneuverFrames, 100*maneuverFrames/totalFrames);
    fprintf('  平均CV概率: %.3f\n', mean(modelProb(1, ~isnan(modelProb(1, :)))));
    fprintf('  平均CT概率: %.3f\n', mean(modelProb(2, ~isnan(modelProb(2, :)))));
    fprintf('  平均CA概率: %.3f\n', mean(modelProb(3, ~isnan(modelProb(3, :)))));
    fprintf('  最大机动概率: %.3f\n', max(maneuverProb));
    fprintf('\n');
end
