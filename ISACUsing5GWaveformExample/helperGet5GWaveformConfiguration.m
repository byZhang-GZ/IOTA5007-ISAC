function waveformConfig = helperGet5GWaveformConfiguration(channelBandwidth, bandwidthOccupancy, subcarrierSpacing)

    %   Copyright 2025 The MathWorks, Inc.

    % Calculate the transmission bandwidth and required number of resource blocks given the bandwidth occupancy.
    numResourceBlocks = floor((channelBandwidth*1e6/(subcarrierSpacing*1e3)*bandwidthOccupancy)/12); % 12 subcarriers per RB
    
    waveformConfig = struct();   % Clear simParameters variable to contain all key simulation parameters
    waveformConfig.NFrames = 1;  % Number of 10 ms frames to simulate for each SNR
    waveformConfig.DisplaySimulationInformation = false; % Display detailed simulation information
    
    % Set waveform type and PDSCH numerology (SCS and CP type)
    waveformConfig.Carrier = nrCarrierConfig;   % Carrier resource grid configuration
    waveformConfig.Carrier.NSizeGrid = numResourceBlocks;
    waveformConfig.Carrier.SubcarrierSpacing = subcarrierSpacing;    % 15, 30, 60, 120 (kHz)
    waveformConfig.Carrier.CyclicPrefix = 'Normal';   % 'Normal' or 'Extended' (Extended CP is relevant for 60 kHz SCS only)
    waveformConfig.Carrier.NCellID = 1;               % Cell identity
    
    % PDSCH/DL-SCH parameters
    waveformConfig.PDSCH = nrPDSCHConfig;      % This PDSCH definition is the basis for all PDSCH transmissions in the BLER simulation
    waveformConfig.PDSCHExtension = struct();  % This structure is to hold additional simulation parameters for the DL-SCH and PDSCH
    
    % Define PDSCH time-frequency resource allocation per slot to be full grid (single full grid BWP)
    waveformConfig.PDSCH.PRBSet = 0:waveformConfig.Carrier.NSizeGrid-1;                 % PDSCH PRB allocation
    waveformConfig.PDSCH.SymbolAllocation = [0,waveformConfig.Carrier.SymbolsPerSlot];  % Starting symbol and number of symbols of each PDSCH allocation
    waveformConfig.PDSCH.MappingType = 'A';     % PDSCH mapping type ('A'(slot-wise),'B'(non slot-wise))
    
    % Scrambling identifiers
    waveformConfig.PDSCH.NID = waveformConfig.Carrier.NCellID;
    waveformConfig.PDSCH.RNTI = 1;
    
    % PDSCH resource block mapping (TS 38.211 Section 7.3.1.6)
    waveformConfig.PDSCH.VRBToPRBInterleaving = 0; % Disable interleaved resource mapping
    waveformConfig.PDSCH.VRBBundleSize = 4;
    
    % Define the number of transmission layers to be used
    waveformConfig.PDSCH.NumLayers = 1;            % Number of PDSCH transmission layers
    
    % Define codeword modulation and target coding rate
    % The number of codewords is directly dependent on the number of layers so ensure that
    % layers are set first before getting the codeword number
    if waveformConfig.PDSCH.NumCodewords > 1                             % Multicodeword transmission (when number of layers being > 4)
        waveformConfig.PDSCH.Modulation = {'16QAM','16QAM'};             % 'QPSK', '16QAM', '64QAM', '256QAM'
        waveformConfig.PDSCHExtension.TargetCodeRate = [490 490]/1024;   % Code rate used to calculate transport block sizes
    else
        waveformConfig.PDSCH.Modulation = '16QAM';                       % 'QPSK', '16QAM', '64QAM', '256QAM'
        waveformConfig.PDSCHExtension.TargetCodeRate = 490/1024;         % Code rate used to calculate transport block sizes
    end
    
    % DM-RS and antenna port configuration (TS 38.211 Section 7.4.1.1)
    waveformConfig.PDSCH.DMRS.DMRSPortSet = 0:waveformConfig.PDSCH.NumLayers-1; % DM-RS ports to use for the layers
    waveformConfig.PDSCH.DMRS.DMRSTypeAPosition = 2;      % Mapping type A only. First DM-RS symbol position (2,3)
    
    waveformConfig.PDSCH.DMRS.DMRSLength = 1;             % Number of front-loaded DM-RS symbols (1(single symbol),2(double symbol))
    waveformConfig.PDSCH.DMRS.DMRSAdditionalPosition = 2; % Additional DM-RS symbol positions (max range 0...3)
    waveformConfig.PDSCH.DMRS.NumCDMGroupsWithoutData = 1;% Number of CDM groups without data
    
    waveformConfig.PDSCH.DMRS.DMRSConfigurationType = 1;  % DM-RS configuration type (1,2)
    waveformConfig.PDSCH.DMRS.NIDNSCID = 1;               % Scrambling identity (0...65535)
    waveformConfig.PDSCH.DMRS.NSCID = 0;                  % Scrambling initialization (0,1)
    
    % PT-RS configuration (TS 38.211 Section 7.4.1.2)
    waveformConfig.PDSCH.EnablePTRS = 0;                  % Enable or disable PT-RS (1 or 0)
    waveformConfig.PDSCH.PTRS.TimeDensity = 1;            % PT-RS time density (L_PT-RS) (1, 2, 4)
    waveformConfig.PDSCH.PTRS.FrequencyDensity = 2;       % PT-RS frequency density (K_PT-RS) (2 or 4)
    waveformConfig.PDSCH.PTRS.REOffset = '00';            % PT-RS resource element offset ('00', '01', '10', '11')
    waveformConfig.PDSCH.PTRS.PTRSPortSet = [];           % PT-RS antenna port, subset of DM-RS port set. Empty corresponds to lower DM-RS port number
    
    % Reserved PRB patterns, if required (for CORESETs, forward compatibility etc)
    waveformConfig.PDSCH.ReservedPRB{1}.SymbolSet = [];   % Reserved PDSCH symbols
    waveformConfig.PDSCH.ReservedPRB{1}.PRBSet = [];      % Reserved PDSCH PRBs
    waveformConfig.PDSCH.ReservedPRB{1}.Period = [];      % Periodicity of reserved resources
    
    % Additional simulation and DL-SCH related parameters
    %
    % PDSCH PRB bundling (TS 38.214 Section 5.1.2.3)
    waveformConfig.PDSCHExtension.PRGBundleSize = [];     % 2, 4, or [] to signify "wideband"
    
    % HARQ process and rate matching/TBS parameters
    waveformConfig.PDSCHExtension.XOverhead = 6*waveformConfig.PDSCH.EnablePTRS; % Set PDSCH rate matching overhead for TBS (Xoh) to 6 when PT-RS is enabled, otherwise 0
    waveformConfig.PDSCHExtension.NHARQProcesses = 16;    % Number of parallel HARQ processes to use
    waveformConfig.PDSCHExtension.EnableHARQ = true;      % Enable retransmissions for each process, using RV sequence [0,2,3,1]
    
    % LDPC decoder parameters
    % Available algorithms: 'Belief propagation', 'Layered belief propagation', 'Normalized min-sum', 'Offset min-sum'
    waveformConfig.PDSCHExtension.LDPCDecodingAlgorithm = 'Normalized min-sum';
    waveformConfig.PDSCHExtension.MaximumLDPCIterationCount = 6;
    
    % Define data type ('single' or 'double') for resource grids and waveforms
    waveformConfig.DataType = 'double';
end