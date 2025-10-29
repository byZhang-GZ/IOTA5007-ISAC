channelBandwidth = 50; % 设置5G系统的信道带宽为50 MHz
bandwidthOccupancy = 0.9; % 设置带宽占用率为90%。剩下的10%作为保护带，以减少对邻近频道的干扰
subcarrierSpacing = 60; %子载波间隔为60kHz

waveformConfig = helperGet5GWaveformConfiguration(channelBandwidth, bandwidthOccupancy, subcarrierSpacing);
waveformInfo = nrOFDMInfo(waveformConfig.Carrier);

transmissionBandwidth = subcarrierSpacing*1e3*waveformConfig.Carrier.NSizeGrid*12; % Bandwidth of resource blocks in Hz
fprintf("Transmission bandwidth: %d MHz\n",transmissionBandwidth);
%计算实际的传输带宽。公式是：子载波间隔 × 资源块(RB)内的子载波数(12) × 总RB数

channel.SampleRate = waveformInfo.SampleRate; % 信道对象的采样率设置为5G波形的采样率

waveformConfig.PDSCH.DMRS.DMRSTypeAPosition = 2;           % Mapping type A only. First DM-RS symbol position is 2
waveformConfig.PDSCH.DMRS.DMRSLength = 1;                  % Single-symbold DM-RS
waveformConfig.PDSCH.DMRS.DMRSAdditionalPosition = 2;      % Additional DM-RS symbol positions (max range 0...3)
waveformConfig.PDSCH.DMRS.NumCDMGroupsWithoutData = 1;     % Number of CDM groups without data
waveformConfig.PDSCH.DMRS.DMRSConfigurationType = 1;       % DM-RS configuration type (1,2)
% 设置DM-RS符号在时域上的起始位置，设置DM-RS占用1个OFDM符号，设置额外的DM-RS符号位置、增加时域上的导频密度
% 设置不传输数据的码分复用（CDM）组数量、可以提高导频功率，设置DM-RS在频域上的梳状结构（占用奇数或偶数个子载波）
% 这些参数共同决定了DM-RS导频在时频资源网格上的分布模式，直接影响了感知的最大无模糊范围和最大无模糊速度。



resourceGrid = nrResourceGrid(waveformConfig.Carrier, waveformConfig.PDSCH.NumLayers);
ind = nrPDSCHDMRSIndices(waveformConfig.Carrier, waveformConfig.PDSCH);
resourceGrid(ind) = nrPDSCHDMRS(waveformConfig.Carrier, waveformConfig.PDSCH);
helperVisualizeResourceGrid(abs(resourceGrid(1:12,1:14,1)));
title({'PDSCH DM-RS Resource Elements in the Carrier Resource Grid', '(single resource block)'});
% 这几行代码是为了可视化DM-RS在资源网格上的位置



% Compute maximum pilot signal separation in time
[~,pdschInfo] = nrPDSCHIndices(waveformConfig.Carrier,waveformConfig.PDSCH);
Mt = max(diff(pdschInfo.DMRSSymbolSet));



% 计算一个OFDM符号的持续时间
ofdmSymbolDuration = mode(waveformInfo.SymbolLengths)/waveformInfo.SampleRate;
% 基于奈奎斯特采样定理的无模糊测量的最大频率
maxDoppler = 1/(2*Mt*ofdmSymbolDuration);
fprintf('Maximum unambiguous Doppler shift: %.2f kHz\n', maxDoppler*1e-3);

% 将最大多普勒频移转换为最大无模糊速度
maxVelocity = dop2speed(maxDoppler, wavelength)/2;
fprintf('Maximum unambiguous velocity: %.2f m/s\n', maxVelocity);

% Compute pilot signal separation in frequency
Mf = min(diff(waveformConfig.PDSCH.DMRS.DMRSSubcarrierLocations));
maxDelay = 1/(2*Mf*subcarrierSpacing*1e3);
fprintf('Maximum time delay: %.2fe-6 s\n', maxDelay*1e6);
% 计算DM-RS导频在频率维度上的间隔 

maxRange = time2range(maxDelay);
fprintf('Maximum unambiguous range: %.2f m\n', maxRange);
% 将最大时延转换为最大无模糊（双基地）距离

% Range resolution
rangeResolution = bw2rangeres(transmissionBandwidth);
fprintf('Range resolution: %.2f m\n', rangeResolution);
% 根据实际的传输带宽计算系统的距离分辨率。带宽越宽，分辨率越高