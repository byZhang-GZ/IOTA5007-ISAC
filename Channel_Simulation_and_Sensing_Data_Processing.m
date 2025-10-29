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
% Preallocate space for 10 tracks
targetStateEstimates = zeros(4, numSensingFrames, 10);
trackIDs = [];

% Produce visualizations
generateVisualizations = true; %是否显示实时动画

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

    % Average over AoDs and symbols
    x = reshape(x, numAoA, numRange, []);
    sumRangeAoAMap = sqrt(sum(abs(x).^2, 3)).';
    % 生成最终的距离-角度图，也称为距离-角度热力图

    % Perform 2-D CFAR detection over range-AoA map
    cutResult = cfar2D(sumRangeAoAMap, cutidx); % sumRangeAoAMap 是待检测的图像，cutidx 是一个包含了所有待测单元坐标的索引列表
    detectionIdxs = cutidx(:, cutResult); % 提取出被检测为目标的单元的索引。是一个2×N的矩阵，其中N是检测到的点的数量。每一列[range_index; angle_index]都是一个检测点的索引坐标
    detectionValues = [rangeGrid(detectionIdxs(1,:)); aoaGrid(detectionIdxs(2,:))];% 将检测点的索引坐标转换为物理值    

    % Cluster CFAR detections
    [~, clusterids] = clusterer(detectionValues.'); % 对CFAR检测出的所有点进行聚类
    uniqClusterIds = unique(clusterids); % 找出 clusterids 中所有独一无二的簇ID


    m = numel(uniqClusterIds); %获取独立簇的数量
    clusteredDetections = zeros(2, m);%2行m列的矩阵，用于存储每个簇的中心位置
    %计算每个簇的中心位置
    for j = 1:m
        idxs = clusterids == uniqClusterIds(j);
        d = detectionValues(:, idxs); 
        clusteredDetections(:, j) = mean(d, 2);
    end

    % Format detections so that they can be processed by the tracker
    detections = helperFormatDetectionsForTracker(clusteredDetections, t(i), rangeResolution, aoaResolution);
    
    % Pass detections to the tracker
    tracks = tracker(detections); % 执行跟踪步骤。这是整个循环中最高层的处理步骤之一

    % Store target state estimates
    for it = 1:numel(tracks)
        id = tracks(it).TrackID;
        trackIDs = union(trackIDs, id);        
        targetStateEstimates(:, i, id) = tracks(it).State;
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
        measuredPositions = helperGetCartesianMeasurement(clusteredDetections, txPosition, rxPosition, rxOrientationAxes);
        scenarioPlotter.plotDetections(measuredPositions);
        scenarioPlotter.plotTracks(tracks);
    
        % Visualize CFAR results 
        [trueRrx, trueAoA] = rangeangle(targetPositions.', rxPosition, rxOrientationAxes);
        [trueRtx, trueAoD] = rangeangle(targetPositions.', txPosition, txOrientationAxes);
        trueBistaticRange = trueRrx + trueRtx - baseline;
    
        cfarResultsVisualizer.plotCFARImage(aoaGrid, rangeGrid, sumRangeAoAMap);
        cfarResultsVisualizer.plotTruth([trueBistaticRange; trueAoA(1, :)]);
        cfarResultsVisualizer.plotClusteredDetections(clusteredDetections);

        sgtitle(scenarioCFARResultsFigure, sprintf('Frame %d, Simulation Time %.1f s', i, t(i)), 'FontSize' ,10);
    end
end