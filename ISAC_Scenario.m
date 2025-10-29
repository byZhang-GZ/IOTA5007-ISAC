rng('default');                             % 将随机数生成器重置到其默认启动设置
carrierFrequency = 60e9;                    % 载波频率（HZ）
wavelength = freq2wavelen(carrierFrequency);% 换算成波长

txPosition = [0; 0; 0]; %发射机（基站）位置

txPower = 46;           % 发射机功率
transmitter = phased.Transmitter('PeakPower', db2pow(txPower-30), 'Gain', 0);% 发射机的射频前端，设置发射机的峰值功率和增益

numTxAntennas = 8;      % 发射天线阵列中的天线单元数量 
element = phased.IsotropicAntennaElement('BackBaffled', true); % 创建天线单元对象，设置天线有后挡板
txArray = phased.ULA(numTxAntennas, wavelength/2, 'Element', element); %阵列中的单元数量，天线单元之间的间距，天线单元

txOrientationAxes = eye(3); %定义发射天线阵列的朝向，单位矩阵表示阵列的坐标系与全局坐标系完全对齐
% 阵列的法线（主波束方向）指向X轴正方向



%以下是接收机的设置
rxPosition = [50; 60; 0];

receiver = phased.Receiver('AddInputNoise', true, 'Gain', 0, 'NoiseMethod', 'Noise figure',...
    'NoiseFigure', 5, 'ReferenceTemperature', 290);%模拟接收机的射频前端和前置放大器。
% 在仿真中加入噪声，接收机增益，使用"噪声系数"的方法来计算噪声，设置噪声系数为5 dB，设置参考温度计算热噪声
numRxAntennas = 8;      % Number of antenna elements at the Rx array
rxArray = phased.ULA(numRxAntennas, wavelength/2, 'Element', element);
rxOrientationAxes = rotz(-90); % 绕Z轴逆时针旋转-90度。阵列的法线方向指向了全局坐标系的负Y轴方向



%以下是环境中散射体和目标的设置
regionOfInterest = [0 120; -80 80];        % 定义了一个感兴趣的区域范围
numScatterers = 40;                                        % Number of scatterers distributed with the region of interest

% Randomly distributed scatterers
[scatterers.Positions, scatterers.ReflectionCoefficients] = ...
    helperGenerateStaticScatterers(numScatterers, regionOfInterest);
% 它在指定的区域内随机生成 numScatterers 个点的位置，并为每个点赋予一个随机的反射系数
scatterers.Velocities = zeros(size(scatterers.Positions)); % 设置所有这些散射体的速度为零，确认它们是静态的



targets.Trajectories = helperGetTargetTrajectories(); % 生成两个移动目标的运动轨迹
numTargets = numel(targets.Trajectories);
targets.ReflectionCoefficients = exp(1i*(2*pi*rand(1, numTargets)));
% 每个目标随机生成一个复数形式的反射系数

channel = phased.ScatteringMIMOChannel('CarrierFrequency', carrierFrequency, 'TransmitArray', txArray,...
    'TransmitArrayPosition', txPosition, 'ReceiveArray', rxArray, 'ReceiveArrayPosition', rxPosition,...
    "TransmitArrayOrientationAxes", txOrientationAxes, "ReceiveArrayOrientationAxes", rxOrientationAxes,... , 
    'SimulateDirectPath', true, 'ScattererSpecificationSource', 'Input Port', ...
    'ChannelResponseOutputPort',true,'Polarization','None');
helperVisualizeScatteringMIMOChannel(channel, scatterers.Positions, targets.Trajectories);
title('ISAC Scenario');

% 用于模拟一个完整的多输入多输出（MIMO）散射信道。设置信道的载波频率，关联发射天线阵列
% 关联接收天线阵列，关联发射机和接收机的位置与朝向，仿真包含从发射机到接收机的直射路径，
% 定散射体（包括静态和动态目标）的信息将通过函数的输入端口在仿真时提供。
% 'ChannelResponseOutputPort', true: 极其关键！这个设置让信道对象在仿真时输出信道的响应（即信道矩阵），而不是只输出经过信道后的信号。
