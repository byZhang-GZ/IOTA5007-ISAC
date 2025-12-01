%% ========== Channel Simulation and Sensing Data Processing ==========
% 此脚本需要先运行配置脚本: ISAC_Scenario.m 和 FiveG_Waveform_Config.m
% 如果变量不存在，将自动运行这些脚本

% 添加必要的路径
addpath('ISACUsing5GWaveformExample');
addpath('Supporting_Functions');

% 检查必需的变量是否存在
if ~exist('numRxAntennas', 'var') || ~exist('waveformConfig', 'var')
    fprintf('Missing configuration variables detected, running config scripts...\n');
    run('ISAC_Scenario.m');
    fprintf('✓ ISAC scenario configured\n');
    run('FiveG_Waveform_Config.m');
    fprintf('✓ 5G waveform configured\n\n');
end

%  总共仿真10个PDSCH帧
numSensingFrames = 10;

% Time step between simulated PDSCH frames (s)
dt = 0.4;

% 时间向量，表示每次仿真的时刻点，从0秒到3.6秒
t = (0:numSensingFrames-1)*dt;


% 处理接收机到达角 (AoA) 
numAoA = ceil(2*pi*numRxAntennas);% 计算到达角(AoA)估计的角度网格点数，ceil函数将计算结果向上取整
aoaGrid = linspace(-90, 90, numAoA); % 生成numAoA个等间距的角度点
rxSteeringVector = phased.SteeringVector(SensorArray=channel.ReceiveArray);% 导向矢量计算器，用于计算该阵列在不同方向上的相位响应
rxsv = rxSteeringVector(carrierFrequency, aoaGrid);
% 一次性计算出在特定载波频率下，aoaGrid中每一个角度点对应的导向矢量
% 结果 rxsv 是一个复数矩阵，其每一列都代表一个特定到达角度的导向矢量

% 处理接收机发射角 (AoD)
numAoD = ceil(2*pi*numTxAntennas);                         % AoD grid size
aodGrid = linspace(-90, 90, numAoD);                       % AoD grid
txSteeringVector = phased.SteeringVector(SensorArray=channel.TransmitArray);
txsv = txSteeringVector(carrierFrequency, aodGrid);




% Limit the maximum bistatic range of interest to 100 meters
maxRange = 100;
rangeGrid = -20:rangeResolution/2:maxRange; % 距离搜索网格rangeGrid
numRange = numel(rangeGrid); % 计算rangeGrid中元素的总数，即距离网格的点数。

% Ensure targets struct is available even if scenario script was not run in this session
if ~exist('targets', 'var') || ~isstruct(targets) || ~isfield(targets, 'Trajectories')
    trajectories = helperGetTargetTrajectories();
    targets = struct();
    targets.Trajectories = trajectories;
    targets.ReflectionCoefficients = exp(1i*(2*pi*rand(1, numel(trajectories))));
end

% Baseline distance between Tx and Rx
baseline = vecnorm(txPosition - rxPosition); % 计算发射机和接收机之间的直线距离（基线距离）

% Fast-time steering vectors 
rangeFFTBins = (rangeGrid+baseline)/(2*rangeResolution);
numSubcarriers = waveformConfig.Carrier.NSizeGrid*12;
% 计算5G信号实际使用的总子载波数 NSizeGrid是资源块（RB）的数量，每个RB固定有12个子载波。
k = (0:numSubcarriers-1).'; % 所有子载波的索引。
rngsv = (exp(2*pi*1i*k*rangeFFTBins/numSubcarriers)/numSubcarriers);
% 用于距离处理的离散傅里叶变换（DFT）矩阵


%检测与跟踪准备
% 计算CFAR检测器在距离维度上所需的保护单元数量
numGuardCellsRange = ceil(rangeResolution/(rangeGrid(2)-rangeGrid(1))) + 1;

% 计算接收天线阵列的角度分辨率
aoaResolution = ap2beamwidth((numRxAntennas-1)*channel.ReceiveArray.ElementSpacing, wavelength);
% 计算CFAR检测器在角度维度上所需的保护单元数量
numGuardCellsAoA = ceil(aoaResolution/(aodGrid(2)-aodGrid(1))) + 1;

% 根据参数创建一个二维恒虚警率检测器2-D CFAR
cfar2D = phased.CFARDetector2D('GuardBandSize',[numGuardCellsRange numGuardCellsAoA], 'TrainingBandSize', [4 6],...
    'ProbabilityFalseAlarm', 1e-3, 'OutputFormat', 'CUT result', 'ThresholdOutputPort', false);



% Compute indices of the cells under test
offsetIdxs = cfar2D.TrainingBandSize + cfar2D.GuardBandSize;

rangeIdxs = offsetIdxs(1)+1:numel(rangeGrid)-offsetIdxs(1);    
angleIdxs = offsetIdxs(2)+1:numel(aoaGrid)-offsetIdxs(2);

[sumRangeIdxs, aoaIdxs] = meshgrid(rangeIdxs, angleIdxs);
cutidx = [sumRangeIdxs(:).'; aoaIdxs(:).'];

clusterer = clusterDBSCAN();
% 聚类器对象。它将被用来将CFAR检测产生的密集检测点聚合成单个目标


tracker = helperConfigureTracker(txPosition, rxPosition, txOrientationAxes, rxOrientationAxes, rangeResolution, aoaResolution);
% Preallocate space to store estimated target state vectors [x; vx; y; vy]
maxNumTracks = 10;
targetStateEstimates = nan(4, numSensingFrames, maxNumTracks);
% 保存IMM模型概率 (3个模型: CV、CT和CA)
modelProbabilities = nan(3, numSensingFrames, maxNumTracks);
trackIDList = nan(1, maxNumTracks);
activeTrackCount = 0;

% === 自适应过程噪声参数 ===
% 为每个航迹维护自适应状态
adaptiveNoiseParams = struct();
adaptiveNoiseParams.maneuverThreshold = 0.6;      % 机动检测阈值（CT+CA概率之和）- 提高以减少误报
adaptiveNoiseParams.boostFactor = 2.5;            % Q矩阵增强系数 - 降低以减少抖动
adaptiveNoiseParams.smoothingAlpha = 0.15;        % EWMA平滑系数（0-1，越小越平滑）- 降低以过滤噪声
adaptiveNoiseParams.cooldownFrames = 3;           % 冷却帧数，避免频繁切换
adaptiveNoiseParams.minBoostDuration = 2;         % 最小增强持续帧数

% 为每个航迹存储自适应状态
for iTrack = 1:maxNumTracks
    adaptiveNoiseParams.trackStates(iTrack).isManeuver = false;       % 当前是否处于机动状态
    adaptiveNoiseParams.trackStates(iTrack).maneuverProb = 0;         % 平滑后的机动概率
    adaptiveNoiseParams.trackStates(iTrack).cooldownCounter = 0;      % 冷却计数器
    adaptiveNoiseParams.trackStates(iTrack).boostCounter = 0;         % 增强持续计数器
    adaptiveNoiseParams.trackStates(iTrack).currentBoostFactor = 1.0; % 当前实际的增强系数（平滑过渡）
end

% Produce visualizations
generateVisualizations = true; %是否显示实时动画
% generateVisualizations = false; %是否显示实时动画


if generateVisualizations
    scenarioCFARResultsFigure = figure;
    tl = tiledlayout(scenarioCFARResultsFigure, 1, 2, 'Padding', 'tight', 'TileSpacing', 'tight')
    
    scenarioPlotter = helperTheaterPlotter(tl);
    scenarioPlotter.plotTxAndRx(txPosition, rxPosition);
    scenarioPlotter.plotTrajectories(targets.Trajectories, t);
    scenarioPlotter.plotScatterers(scatterers.Positions);
    title(scenarioPlotter, 'ISAC Scenario');

    cfarResultsVisualizer = helperCFARResultsVisualizer(tl);
    xlabel(cfarResultsVisualizer, 'AoA (degrees)');
    ylabel(cfarResultsVisualizer, 'Bistatic Range (m)');
    title(cfarResultsVisualizer, 'CFAR Results');
end

% 计算每一帧5G信号包含的OFDM符号总数
numSymbolsPerFrame = waveformInfo.SlotsPerFrame*waveformInfo.SymbolsPerSlot;

% Transmit and process one PDSCH frame at a time
for i = 1:numSensingFrames
    fprintf('\nSimulating frame at time %g s\n', t(i));
    
    % 执行单帧5G信号的收发仿真，是感知处理的原始数据
    % 子载波数 × 符号数 × 接收天线数 × 发射天线数（numSubcarriers x numSymbolsPerFrame x numRxAntennas x numTxAntennas）
    % helperSimulateLinkSingleFrame函数内部会生成一帧5G PDSCH波形 通过 channel 对象模拟其在包含 scatterers 和 targets 的环境中的传播，最后在 receiver 处接收并估计出信道矩阵。
    sensingCSI = helperSimulateLinkSingleFrame(transmitter, receiver, channel, waveformConfig, scatterers, targets, t(i));

    % 重置channel对象的内部状态。
    reset(channel);

    % To filter the static scatterers out, put a zero in the first Doppler
    X = fft(sensingCSI, [], 2); %第2维度（即OFDM符号维度，也称为"慢时间"维度）进行FFT
    % X 是一个与 sensingCSI 同样大小的4维矩阵，但它的第2维现在代表的是多普勒频移
    X(:, 1, :, :) = 0; % 将零多普勒分量强制设为0，我们就可以有效地滤除来自这些静止物体的干扰（杂波）
    x = ifft(X, [], 2); % 其中由静态物体产生的信号分量已经被大幅削弱。

    
    
    % 第一次维度重排列——执行发射角（AoD）处理，也称为发射端波束形成或空间滤波
    x = permute(x, [4 1 3 2]); %这个新顺序将矩阵变为 [1:发射天线, 2:子载波, 3:接收天线, 4:符号]。
    x = txsv'*reshape(x, numTxAntennas, []); % 将4维矩阵 x "拉平"成一个2维矩阵，其行数为 numTxAntennas（8），列数由其余维度（子载波、接收天线、符号）的乘积决定
    % x的行维度现在代表的是发射角（AoD）

    
    % 第二次维度重排——执行距离处理
    x = permute(reshape(x, numAoA, numSubcarriers, numRxAntennas, numSymbolsPerFrame), [2 3 1 4]); 
    % 将当前数据x重塑回4维，其维度现在代表 [AoD, 子载波, 接收天线, 符号]，然后重排维度，新顺序为 [子载波, 接收天线, AoD, 符号]
    x = rngsv.'*reshape(x, numSubcarriers, []); % 执行一次逆傅里叶变换，将信号从"子载波域"（频域）转换到"距离域"
    % 矩阵x的行维度现在代表的是距离



    % 第三次进行维度重排，为到达角（AoA）处理做准备
    x = permute(reshape(x, numRange, numRxAntennas, numAoD, numSymbolsPerFrame), [2 1 3 4]);
    x = rxsv'*reshape(x, numRxAntennas, []);
    % 矩阵 x 的行维度现在代表的是到达角（AoA）
    
    % 重塑为 [AoA, Range, AoD, Symbols] 以便进行多普勒处理
    x_4D = reshape(x, numAoA, numRange, numAoD, numSymbolsPerFrame);
    
    % 在符号维度（慢时间）进行FFT以获取多普勒信息
    x_R_A_D = fft(x_4D, [], 4);  % 第4维是符号维，FFT后变为多普勒维
    x_R_A_D = fftshift(x_R_A_D, 4);  % 将零频移到中心
    
    % 生成距离-角度图（在AoD和多普勒上积分）
    rangeAoAMap = sqrt(sum(abs(x_R_A_D).^2, [3 4])).';
    
    % Perform 2-D CFAR detection over range-AoA map
    cutResult = cfar2D(rangeAoAMap, cutidx); % rangeAoAMap 是待检测的图像
    detectionIdxs = cutidx(:, cutResult); % 提取出被检测为目标的单元的索引
    detectionValues = [rangeGrid(detectionIdxs(1,:)); aoaGrid(detectionIdxs(2,:))];
    
    % === 创新：为每个检测提取多普勒信息 ===
    numDetections = size(detectionIdxs, 2);
    dopplerValues = zeros(1, numDetections);
    
    % 多普勒网格（对应速度）
    % 符号周期 = 符号长度（样本数）/ 采样率
    symbolDuration = waveformInfo.SymbolLengths(1) / waveformInfo.SampleRate;
    % 多普勒分辨率 = 1 / (符号周期 * 符号数)
    dopplerResolution = 1 / (numSymbolsPerFrame * symbolDuration);
    maxDoppler = dopplerResolution * numSymbolsPerFrame / 2;
    dopplerGrid = linspace(-maxDoppler, maxDoppler, numSymbolsPerFrame);
    velocityGrid = dopplerGrid * wavelength / 2;  % 转换为速度 (m/s)
    
    for d = 1:numDetections
        rangeIdx = detectionIdxs(1, d);
        aoaIdx = detectionIdxs(2, d);
        
        % 提取该检测点的多普勒切片（在AoD上求和）
        dopplerSlice = squeeze(sqrt(sum(abs(x_R_A_D(aoaIdx, rangeIdx, :, :)).^2, 3)));
        
        % 找到峰值多普勒索引
        [~, dopplerIdx] = max(dopplerSlice);
        
        % 转换为实际径向速度
        dopplerValues(d) = velocityGrid(dopplerIdx);
    end
    
    % 将多普勒信息添加到检测值中
    detectionValues = [detectionValues; dopplerValues];  % 现在是 3 x N 矩阵
    
    % Cluster CFAR detections (使用3D检测值)
    [~, clusterids] = clusterer(detectionValues.'); % 对CFAR检测出的所有点进行聚类
    uniqClusterIds = unique(clusterids); % 找出 clusterids 中所有独一无二的簇ID

    m = numel(uniqClusterIds); %获取独立簇的数量
    clusteredDetections = zeros(3, m);  % 现在是3行（距离、角度、多普勒）
    %计算每个簇的中心位置
    for j = 1:m
        idxs = clusterids == uniqClusterIds(j);
        d = detectionValues(:, idxs); 
        clusteredDetections(:, j) = mean(d, 2);
    end

    % Format detections so that they can be processed by the tracker
    detections = helperFormatDetectionsForTracker(clusteredDetections, t(i), rangeResolution, aoaResolution);
    
    % === 自适应过程噪声调整 ===
    % 在调用tracker之前，根据上一帧的模型概率调整过程噪声
    if i > 1  % 从第二帧开始才有历史数据
        for iTrack = 1:activeTrackCount
            trackID = trackIDList(iTrack);
            
            % 获取上一帧的模型概率
            prevModelProb = modelProbabilities(:, i-1, iTrack);
            
            % 检查是否有有效的概率数据
            if ~any(isnan(prevModelProb))
                % 计算机动模型（CT + CA）的概率之和
                maneuverProbRaw = prevModelProb(2) + prevModelProb(3);  % CT概率 + CA概率
                
                % 使用EWMA平滑机动概率，避免突变
                trackState = adaptiveNoiseParams.trackStates(iTrack);
                trackState.maneuverProb = adaptiveNoiseParams.smoothingAlpha * maneuverProbRaw + ...
                                          (1 - adaptiveNoiseParams.smoothingAlpha) * trackState.maneuverProb;
                
                % 机动状态判断逻辑
                if ~trackState.isManeuver
                    % 当前非机动状态，检查是否进入机动
                    if trackState.maneuverProb > adaptiveNoiseParams.maneuverThreshold && ...
                       trackState.cooldownCounter == 0
                        % 触发机动状态
                        trackState.isManeuver = true;
                        trackState.boostCounter = 0;
                        fprintf('  [Adaptive] Track %d: Maneuver detected (prob=%.3f), increasing process noise\n', ...
                                trackID, trackState.maneuverProb);
                    end
                else
                    % 当前处于机动状态，检查是否退出机动
                    trackState.boostCounter = trackState.boostCounter + 1;
                    
                    % 只有在持续了最小增强时长且概率回落后才退出
                    if trackState.boostCounter >= adaptiveNoiseParams.minBoostDuration && ...
                       trackState.maneuverProb < adaptiveNoiseParams.maneuverThreshold * 0.7  % 退出阈值稍低，形成迟滞
                        % 退出机动状态
                        trackState.isManeuver = false;
                        trackState.cooldownCounter = adaptiveNoiseParams.cooldownFrames;
                        fprintf('  [Adaptive] Track %d: Maneuver ended (prob=%.3f), restoring process noise\n', ...
                                trackID, trackState.maneuverProb);
                    end
                end
                
                % 更新冷却计数器
                if trackState.cooldownCounter > 0
                    trackState.cooldownCounter = trackState.cooldownCounter - 1;
                end
                
                % 平滑过渡增强系数（避免突变导致的jitter）
                targetBoostFactor = 1.0;
                if trackState.isManeuver
                    targetBoostFactor = adaptiveNoiseParams.boostFactor;
                end
                
                % 使用指数平滑过渡
                transitionSpeed = 0.4;  % 过渡速度（0-1，越大越快）
                trackState.currentBoostFactor = transitionSpeed * targetBoostFactor + ...
                                                (1 - transitionSpeed) * trackState.currentBoostFactor;
                
                % 应用自适应过程噪声调整
                if abs(trackState.currentBoostFactor - 1.0) > 0.01  % 只有在显著偏离1.0时才调整
                    try
                        % 获取当前滤波器
                        filter = getTrackFilterProperties(tracker, trackID, 'Filter');
                        
                        % IMM滤波器包含多个子滤波器，我们主要调整CV模型
                        if isa(filter, 'trackingIMM')
                            % 获取CV滤波器（第一个子滤波器）
                            cvFilter = filter.TrackingFilters{1};
                            
                            % 获取原始过程噪声
                            Q_original = cvFilter.ProcessNoise;
                            
                            % 应用增强系数
                            Q_adaptive = Q_original * trackState.currentBoostFactor;
                            
                            % 更新过程噪声
                            cvFilter.ProcessNoise = Q_adaptive;
                            
                            % 将修改后的滤波器写回
                            filter.TrackingFilters{1} = cvFilter;
                            setTrackFilterProperties(tracker, trackID, 'Filter', filter);
                            
                            % 可选：也可以调整CT和CA模型，但通常CV是主要的
                            % 因为机动时我们希望CV能快速适应，而CT/CA已经是机动模型
                        end
                    catch ME
                        % 如果访问失败，静默跳过（可能航迹刚初始化）
                        % fprintf('  [Adaptive] Track %d: 无法调整过程噪声 (%s)\n', trackID, ME.message);
                    end
                end
                
                % 保存更新后的状态
                adaptiveNoiseParams.trackStates(iTrack) = trackState;
            end
        end
    end
    
    % Pass detections to the tracker
    tracks = tracker(detections, t(i)); % GNN跟踪器需要时间戳

    % Store target state estimates
    for it = 1:numel(tracks)
        id = tracks(it).TrackID;
        idx = find(trackIDList(1:activeTrackCount) == id, 1);
        if isempty(idx)
            if activeTrackCount >= maxNumTracks
                warning('Maximum number of track slots (%d) reached. Skipping track ID %d.', maxNumTracks, id);
                continue;
            end
            activeTrackCount = activeTrackCount + 1;
            trackIDList(activeTrackCount) = id;
            idx = activeTrackCount;
        end

        % 从IMM滤波器提取状态（取前4维：x, vx, y, vy）
        trackState = tracks(it).State;
        targetStateEstimates(:, i, idx) = NaN;
        if numel(trackState) >= 4
            targetStateEstimates(1:4, i, idx) = trackState(1:4);
        else
            targetStateEstimates(1:numel(trackState), i, idx) = trackState;
        end
        
        % 保存IMM模型概率
        modelProbabilities(:, i, idx) = NaN;
        try
            % 使用getTrackFilterProperties获取模型概率
            mp = getTrackFilterProperties(tracker, id, 'ModelProbabilities');
            
            % 处理返回值：可能是cell数组或数值数组
            if iscell(mp) && ~isempty(mp)
                mp = mp{1};  % 提取cell内容
            end
            
            if ~isempty(mp) && isnumeric(mp) && numel(mp) == 3
                modelProbabilities(:, i, idx) = mp(:);
            end
        catch
            % 静默失败，保持NaN
            modelProbabilities(:, i, idx) = NaN;
        end
    end    
    
    % Visualization
    if generateVisualizations
        % Plot true target positions
        targetPositions = zeros(numTargets, 3);
        for it = 1:numTargets
            targetPositions(it, :) = lookupPose(targets.Trajectories{it}, t(i));
        end
        scenarioPlotter.plotTargetPositions(targetPositions);
       
        % Plot target positions estimated from the detections
        % 只使用前2维（距离和角度）进行笛卡尔转换
        clusteredDetections2D = clusteredDetections(1:2, :);
        measuredPositions = helperGetCartesianMeasurement(clusteredDetections2D, txPosition, rxPosition, rxOrientationAxes);
        scenarioPlotter.plotDetections(measuredPositions);
        scenarioPlotter.plotTracks(tracks);
    
        % Visualize CFAR results 
        [trueRrx, trueAoA] = rangeangle(targetPositions.', rxPosition, rxOrientationAxes);
        [trueRtx, trueAoD] = rangeangle(targetPositions.', txPosition, txOrientationAxes);
        trueBistaticRange = trueRrx + trueRtx - baseline;
    
        cfarResultsVisualizer.plotCFARImage(aoaGrid, rangeGrid, rangeAoAMap);
        cfarResultsVisualizer.plotTruth([trueBistaticRange; trueAoA(1, :)]);
        cfarResultsVisualizer.plotClusteredDetections(clusteredDetections2D);

        sgtitle(scenarioCFARResultsFigure, sprintf('Frame %d, Simulation Time %.1f s', i, t(i)), 'FontSize' ,10);
    end
% 将跟踪结果保存到结果目录，供后续性能评估脚本使用
scriptDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(scriptDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
resultFile = fullfile(resultsDir, 'trackingResults.mat');
trackIDs = trackIDList(1:activeTrackCount);

% 保存自适应过程噪声的状态历史（用于后续分析）
adaptiveHistory = struct();
for iTrack = 1:activeTrackCount
    adaptiveHistory.trackStates{iTrack} = adaptiveNoiseParams.trackStates(iTrack);
end

% 保存目标真实轨迹供可视化使用
targetTrajectories = helperGetTargetTrajectories();

save(resultFile, 'targetStateEstimates', 'modelProbabilities', 't', 'trackIDs', 'adaptiveHistory', 'targetTrajectories');
fprintf('\nTracking results saved to %s\n', resultFile);

end