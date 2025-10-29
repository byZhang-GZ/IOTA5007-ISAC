function timeSynchronizedChannelMatrix = helperSimulateLinkSingleFrame(transmitter, receiver, channel, waveformConfig, scatterers, targets, currentTime)  
    
    %   Copyright 2025 The MathWorks, Inc.

    waveformInfo = nrOFDMInfo(waveformConfig.Carrier); % Get information about the baseband waveform after OFDM modulation step

    % Maximum expected channel delay in samples after which we discard frames
    maxChDelay = 1e3;
    
    % Set up redundancy version (RV) sequence for all HARQ processes
    if waveformConfig.PDSCHExtension.EnableHARQ
        % In the final report of RAN WG1 meeting #91 (R1-1719301), it was
        % observed in R1-1717405 that if performance is the priority, [0 2 3 1]
        % should be used. If self-decodability is the priority, it should be
        % taken into account that the upper limit of the code rate at which
        % each RV is self-decodable is in the following order: 0>3>2>1
        rvSeq = [0 2 3 1];
    else
        % HARQ disabled - single transmission with RV=0, no retransmissions
        rvSeq = 0; 
    end
    
    % Create DL-SCH encoder system object to perform transport channel encoding
    encodeDLSCH = nrDLSCH;
    encodeDLSCH.MultipleHARQProcesses = true;
    encodeDLSCH.TargetCodeRate = waveformConfig.PDSCHExtension.TargetCodeRate;
    
    % Create DL-SCH decoder system object to perform transport channel decoding
    % Use layered belief propagation for LDPC decoding, with half the number of
    % iterations as compared to the default for belief propagation decoding
    decodeDLSCH = nrDLSCHDecoder;
    decodeDLSCH.MultipleHARQProcesses = true;
    decodeDLSCH.TargetCodeRate = waveformConfig.PDSCHExtension.TargetCodeRate;
    decodeDLSCH.LDPCDecodingAlgorithm = waveformConfig.PDSCHExtension.LDPCDecodingAlgorithm;
    decodeDLSCH.MaximumLDPCIterationCount = waveformConfig.PDSCHExtension.MaximumLDPCIterationCount;   
       
    % Take full copies of the simulation-level parameter structures so that they are not 
    % PCT broadcast variables when using parfor 
    simLocal = waveformConfig;
    waveinfoLocal = waveformInfo;

    % Take copies of channel-level parameters to simplify subsequent parameter referencing 
    carrier = simLocal.Carrier;
    pdsch = simLocal.PDSCH;
    pdschextra = simLocal.PDSCHExtension;
    decodeDLSCHLocal = decodeDLSCH;  % Copy of the decoder handle to help PCT classification of variable
    decodeDLSCHLocal.reset();        % Reset decoder at the start of each SNR point

    numTxAntennas = getNumElements(channel.TransmitArray);
    numRxAntennas = getNumElements(channel.ReceiveArray);

    validateNumLayers(pdsch.NumLayers, numTxAntennas, numRxAntennas);

    % Specify the fixed order in which we cycle through the HARQ process IDs
    harqSequence = 0:pdschextra.NHARQProcesses-1;

    % Initialize the state of all HARQ processes
    harqEntity = HARQEntity(harqSequence,rvSeq,pdsch.NumCodewords);
    
    % Total number of slots in the simulation period
    NSlots = simLocal.NFrames * carrier.SlotsPerFrame;

    % Use a simple initial precoding matrix
    newWtx = 1/sqrt(numTxAntennas) * ones(pdsch.NumLayers,numTxAntennas);

    % Timing offset, updated in every slot for perfect synchronization and
    % when the correlation is strong for practical synchronization
    offset = 0;

    numSubcarriers = waveformConfig.Carrier.NSizeGrid*12;
    numSymbolsPerFrame = waveformInfo.SlotsPerFrame*waveformInfo.SymbolsPerSlot;
    timeSynchronizedChannelMatrix = zeros(numSubcarriers, numSymbolsPerFrame, numRxAntennas, numTxAntennas);

    % Loop over the entire waveform length
    for nslot = 0:NSlots-1
        carrier.NSlot = nslot;

        % Calculate the transport block sizes for the transmission in the slot
        [pdschIndices,pdschIndicesInfo] = nrPDSCHIndices(carrier,pdsch);
        trBlkSizes = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet),pdschIndicesInfo.NREPerPRB,pdschextra.TargetCodeRate,pdschextra.XOverhead);
    
        % HARQ processing
        for cwIdx = 1:pdsch.NumCodewords
            
            % If new data for current process and codeword then create a new DL-SCH transport block
            if harqEntity.NewData(cwIdx) 
                trBlk = randi([0 1],trBlkSizes(cwIdx),1);
                setTransportBlock(encodeDLSCH,trBlk,cwIdx-1,harqEntity.HARQProcessID);
            
                % If new data because of previous RV sequence time out then flush decoder soft buffer explicitly
                if harqEntity.SequenceTimeout(cwIdx)
                    resetSoftBuffer(decodeDLSCHLocal,cwIdx-1,harqEntity.HARQProcessID);
                end
            end
        end
                                
        % Encode the DL-SCH transport blocks
        codedTrBlocks = encodeDLSCH(pdsch.Modulation,pdsch.NumLayers, ...
            pdschIndicesInfo.G,harqEntity.RedundancyVersion,harqEntity.HARQProcessID);

        % Get precoding matrix (wtx) calculated in previous slot
        wtx = newWtx;
    
        % Create resource grid for a slot
        pdschGrid = nrResourceGrid(carrier,numTxAntennas,OutputDataType=simLocal.DataType);
        
        % PDSCH modulation and precoding
        pdschSymbols = nrPDSCH(carrier,pdsch,codedTrBlocks);
        [pdschAntSymbols,pdschAntIndices] = nrPDSCHPrecode(carrier,pdschSymbols,pdschIndices,wtx);
        
        % PDSCH mapping in grid associated with PDSCH transmission period
        pdschGrid(pdschAntIndices) = pdschAntSymbols;
        
        % PDSCH DM-RS precoding and mapping
        dmrsSymbols = nrPDSCHDMRS(carrier,pdsch);
        dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);
        [dmrsAntSymbols,dmrsAntIndices] = nrPDSCHPrecode(carrier,dmrsSymbols,dmrsIndices,wtx);
        pdschGrid(dmrsAntIndices) = dmrsAntSymbols;
    
        % PDSCH PT-RS precoding and mapping
        ptrsSymbols = nrPDSCHPTRS(carrier,pdsch);
        ptrsIndices = nrPDSCHPTRSIndices(carrier,pdsch);
        [ptrsAntSymbols,ptrsAntIndices] = nrPDSCHPrecode(carrier,ptrsSymbols,ptrsIndices,wtx);
        pdschGrid(ptrsAntIndices) = ptrsAntSymbols;
    
        % OFDM modulation
        [txWaveform,ofdmInfo] = nrOFDMModulate(carrier,pdschGrid);

        % Scale signal amplitude to account for FFT occupancy factor such
        % that the total power is 1W
        powerScalingFactor = sqrt(ofdmInfo.Nfft^2/(12*carrier.NSizeGrid));
        txWaveform = txWaveform*powerScalingFactor;

        % Pass data through channel model. Append zeros at the end of the
        % transmitted waveform to flush channel content. These zeros take
        % into account any delay introduced in the channel. This is a mix
        % of multipath delay and implementation delay. This value may 
        % change depending on the sampling rate, delay profile and delay
        % spread
        txWaveform = [txWaveform; zeros(maxChDelay,size(txWaveform,2))]; %#ok<AGROW>

        rxWaveform = zeros(height(txWaveform),numRxAntennas);
        Ns = [waveinfoLocal.SymbolLengths waveinfoLocal.SymbolLengths(1)];

        numTargets = numel(targets.Trajectories);
        for s = 1:waveinfoLocal.SymbolsPerSlot+1
            symDuration = Ns(s)/ofdmInfo.SampleRate;
            
            % Update target positions
            targetPositions = zeros(numTargets, 3);
            targetVelocities = zeros(numTargets, 3);

            for it = 1:numTargets
                [targetPositions(it, :), ~, targetVelocities(it, :)] = lookupPose(targets.Trajectories{it},currentTime);
            end

            symIdx = sum([0 Ns(1:s-1)])+(1:Ns(s));
            symIdx(symIdx>height(txWaveform)) = []; % If not enough samples just use what we can
            symtxWaveform = txWaveform(symIdx, :);

            symrxWaveform = channel(transmitter(symtxWaveform),...
                [scatterers.Positions targetPositions.'],...
                [scatterers.Velocities targetVelocities.'],...
                [scatterers.ReflectionCoefficients targets.ReflectionCoefficients]); 

            rxWaveform(symIdx,:) = receiver(symrxWaveform);

            currentTime = currentTime + symDuration;
        end
   
        % Practical synchronization. Correlate the received waveform
        % with the PDSCH DM-RS to give timing offset estimate 't' and
        % correlation magnitude 'mag'. The function
        % hSkipWeakTimingOffset is used to update the receiver timing
        % offset. If the correlation peak in 'mag' is weak, the current
        % timing estimate 't' is ignored and the previous estimate
        % 'offset' is used
        [t,mag] = nrTimingEstimate(carrier,rxWaveform,dmrsIndices,dmrsSymbols); 
        offset = hSkipWeakTimingOffset(offset,t,mag);
        % Display a warning if the estimated timing offset exceeds the
        % maximum channel delay
        if offset > maxChDelay
            warning(['Estimated timing offset (%d) is greater than the maximum channel delay (%d).' ...
                ' This will result in a decoding failure. This may be caused by low SNR,' ...
                ' or not enough DM-RS symbols to synchronize successfully.'],offset,maxChDelay);
        end
        rxWaveform = rxWaveform(1+offset:end,:);

        % Perform OFDM demodulation on the received data to recreate the
        % resource grid, including padding in the event that practical
        % synchronization results in an incomplete slot being demodulated
        rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
        [K,~,R] = size(rxGrid);
        rxGrid = resize(rxGrid,[K carrier.SymbolsPerSlot R]);

        % Practical channel estimation between the received grid and
        % each transmission layer, using the PDSCH DM-RS for each
        % layer. This channel estimate includes the effect of
        % transmitter precoding
        [estChannelGridPorts,noiseEst] = hSubbandChannelEstimate(carrier,rxGrid,dmrsIndices,dmrsSymbols,pdschextra.PRGBundleSize,'CDMLengths',pdsch.DMRS.CDMLengths); 
        
        % Average noise estimate across PRGs and layers
        noiseEst = mean(noiseEst,'all');

        % Get PDSCH resource elements from the received grid and
        % channel estimate
        [pdschRx,pdschHest] = nrExtractResources(pdschIndices,rxGrid,estChannelGridPorts);

        % Remove precoding from estChannelGridPorts to get channel
        % estimate w.r.t. antennas
        estChannelGridAnts = precodeChannelEstimate(carrier,estChannelGridPorts,conj(wtx));

        % Equalization
        [pdschEq,csi] = nrEqualizeMMSE(pdschRx,pdschHest,noiseEst);

        % Common phase error (CPE) compensation
        if ~isempty(ptrsIndices)
            % Initialize temporary grid to store equalized symbols
            tempGrid = nrResourceGrid(carrier,pdsch.NumLayers);
    
            % Extract PT-RS symbols from received grid and estimated
            % channel grid
            [ptrsRx,ptrsHest,~,~,ptrsHestIndices,ptrsLayerIndices] = nrExtractResources(ptrsIndices,rxGrid,estChannelGridAnts,tempGrid);
            ptrsHest = nrPDSCHPrecode(carrier,ptrsHest,ptrsHestIndices,permute(wtx,[2 1 3]));
    
            % Equalize PT-RS symbols and map them to tempGrid
            ptrsEq = nrEqualizeMMSE(ptrsRx,ptrsHest,noiseEst);
            tempGrid(ptrsLayerIndices) = ptrsEq;
    
            % Estimate the residual channel at the PT-RS locations in
            % tempGrid
            cpe = nrChannelEstimate(tempGrid,ptrsIndices,ptrsSymbols);
    
            % Sum estimates across subcarriers, receive antennas, and
            % layers. Then, get the CPE by taking the angle of the
            % resultant sum
            cpe = angle(sum(cpe,[1 3 4]));
    
            % Map the equalized PDSCH symbols to tempGrid
            tempGrid(pdschIndices) = pdschEq;
    
            % Correct CPE in each OFDM symbol within the range of reference
            % PT-RS OFDM symbols
            symLoc = pdschIndicesInfo.PTRSSymbolSet(1)+1:pdschIndicesInfo.PTRSSymbolSet(end)+1;
            tempGrid(:,symLoc,:) = tempGrid(:,symLoc,:).*exp(-1i*cpe(symLoc));
    
            % Extract PDSCH symbols
            pdschEq = tempGrid(pdschIndices);
        end
    
        % Decode PDSCH physical channel
        [dlschLLRs,rxSymbols] = nrPDSCHDecode(carrier,pdsch,pdschEq,noiseEst);
    
        % Scale LLRs by CSI
        csi = nrLayerDemap(csi); % CSI layer demapping
        for cwIdx = 1:pdsch.NumCodewords
            Qm = length(dlschLLRs{cwIdx})/length(rxSymbols{cwIdx}); % bits per symbol
            csi{cwIdx} = repmat(csi{cwIdx}.',Qm,1);                 % expand by each bit per symbol
            dlschLLRs{cwIdx} = dlschLLRs{cwIdx} .* csi{cwIdx}(:);   % scale by CSI
        end
    
        % Decode the DL-SCH transport channel
        decodeDLSCHLocal.TransportBlockLength = trBlkSizes;
        [~,blkerr] = decodeDLSCHLocal(dlschLLRs,pdsch.Modulation,pdsch.NumLayers,harqEntity.RedundancyVersion,harqEntity.HARQProcessID);

        % Store values to calculate throughput
        simThroughput = sum(~blkerr .* trBlkSizes);
        maxThroughput = sum(trBlkSizes);
    
        % Update current process with CRC error and advance to next process
        procstatus = updateAndAdvance(harqEntity,blkerr,trBlkSizes,pdschIndicesInfo.G);
        if (simLocal.DisplaySimulationInformation)
            fprintf('\n(%3.2f%%) NSlot=%d, %s',100*(nslot+1)/NSlots,nslot,procstatus);
        end
    
        % Get precoding matrix for next slot
        newWtx = hSVDPrecoders(carrier,pdsch,estChannelGridAnts,pdschextra.PRGBundleSize);

        % Adjust channel estimate for each slot to account for synchrnozied
        % integer number of samples
        k = (1:numSubcarriers)'+(waveformInfo.Nfft/2) - (numSubcarriers/2);
        a = exp(-1i*-2*pi*k*-offset/(waveformInfo.Nfft));
        timeSynchronizedChannelMatrix(:, nslot*waveformInfo.SymbolsPerSlot+1:(nslot+1)*waveformInfo.SymbolsPerSlot, :, :) = estChannelGridAnts .*a;
    end

    % Display the results dynamically in the command window
    if (simLocal.DisplaySimulationInformation)
        fprintf('\n');
    end
    fprintf('\nThroughput(Mbps) for %d frame(s) = %.4f\n',simLocal.NFrames,1e-6*simThroughput/(simLocal.NFrames*10e-3));
    fprintf('Throughput(%%) for %d frame(s) = %.4f\n',simLocal.NFrames,simThroughput*100/maxThroughput);
end

function validateNumLayers(numLayers, numTxAntennas, numRxAntennas)
% Validate the number of layers, relative to the antenna geometry
    antennaDescription = sprintf('min(NTxAnts,NRxAnts) = min(%d,%d) = %d',numTxAntennas,numRxAntennas,min(numTxAntennas,numRxAntennas));
    if numLayers > min(numTxAntennas,numRxAntennas)
        error('The number of layers (%d) must satisfy NumLayers <= %s', ...
            numLayers,antennaDescription);
    end
    
    % Display a warning if the maximum possible rank of the channel equals
    % the number of layers
    if (numLayers > 2) && (numLayers == min(numTxAntennas,numRxAntennas))
        warning(['The maximum possible rank of the channel, given by %s, is equal to NumLayers (%d).' ...
            ' This may result in a decoding failure under some channel conditions.' ...
            ' Try decreasing the number of layers or increasing the channel rank' ...
            ' (use more transmit or receive antennas).'],antennaDescription,numLayers); %#ok<SPWRN>
    end

end

function estChannelGrid = precodeChannelEstimate(carrier,estChannelGrid,W)
% Apply precoding matrix W to the last dimension of the channel estimate

    [K,L,R,P] = size(estChannelGrid);
    estChannelGrid = reshape(estChannelGrid,[K*L R P]);
    estChannelGrid = nrPDSCHPrecode(carrier,estChannelGrid,reshape(1:numel(estChannelGrid),[K*L R P]),W);
    estChannelGrid = reshape(estChannelGrid,K,L,R,[]);

end

%hSkipWeakTimingOffset skip timing offset estimates with weak correlation
%   OFFSET = hSkipWeakTimingOffset(OFFSET,T,MAG) manages receiver timing
%   offset OFFSET, using the current timing estimate T and correlation 
%   magnitude MAG. 

%   Copyright 2019-2025 The MathWorks, Inc.

function offset = hSkipWeakTimingOffset(offset,t,mag)

    % Combine receive antennas in 'mag'
    mag = sum(mag,2);
    
    % Empirically determine threshold based on mean and standard deviation
    % of the correlation magnitude. Exclude values near zero.
    zero = max(mag)*sqrt(eps);
    magz = mag(mag>zero);
    threshold = median(magz) + 7*std(magz);
    
    % If the maximum correlation magnitude equals or exceeds the threshold,
    % accept the current timing estimate 't' as the timing offset
    if (max(magz) >= threshold)
        offset = t;
    end
    
end

%hSubbandChannelEstimate practical subband channel estimation
%   [HEST,NVAR] = hSubbandChannelEstimate(CARRIER,RXGRID,REFIND,REFSYM,BUNDLESIZE)
%   performs channel estimation returning channel estimate H and noise
%   variance estimates NVAR. H is a K-by-N-by-R-by-P array where K is the
%   number of subcarriers, N is the number of OFDM symbols, R is the number
%   of receive antennas and P is the number of reference signal ports. NVAR
%   is an NSB-by-1 vector indicating the measured variance of additive
%   white Gaussian noise on the received reference symbols for every
%   subband.
%
%   CARRIER is a carrier configuration object, <a 
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only this
%   object property is relevant for this function:
%
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%
%   RXGRID is an array of size K-by-L-by-R. K is the number of subcarriers,
%   given by CARRIER.NSizeGrid * 12. L is the number of OFDM symbols in one
%   slot, given by CARRIER.SymbolsPerSlot.
%
%   REFIND and REFSYM are the reference signal indices and symbols,
%   respectively. REFIND is an array of 1-based linear indices addressing a
%   K-by-L-by-P resource array. P is the number of reference signal ports
%   and is inferred from the range of values in REFIND. Only nonzero
%   elements in REFSYM are considered. Any zero-valued elements in REFSYM
%   and their associated indices in REFIND are ignored.
%
%   BUNDLESIZE is the PRG bundle size (2, 4, or [] to signify 'wideband').
%
%   [H,NVAR,INFO] = hSubbandChannelEstimate(...,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs:
%
%   'CDMLengths'      - A 2-element row vector [FD TD] specifying the 
%                       length of FD-CDM and TD-CDM despreading to perform.
%                       A value of 1 for an element indicates no CDM and a
%                       value greater than 1 indicates the length of the
%                       CDM. For example, [2 1] indicates FD-CDM2 and no
%                       TD-CDM. The default is [1 1] (no orthogonal
%                       despreading)
%
%   'AveragingWindow' - A 2-element row vector [F T] specifying the number
%                       of adjacent reference symbols in the frequency
%                       domain F and time domain T over which to average
%                       prior to interpolation. F and T must be odd or
%                       zero. If F or T is zero, the averaging value is
%                       determined automatically from the estimated SNR
%                       (calculated using NVAR). The default is [0 0]
%

%  Copyright 2022-2025 The MathWorks, Inc.

function [Hest, noiseEst] = hSubbandChannelEstimate(carrier,rxGrid,refInd,refSym,bundleSize,varargin)

    % Dimensionality information for subband channel estimation
    K = carrier.NSizeGrid * 12;
    L = carrier.SymbolsPerSlot;
    R = size(rxGrid,3);
    P = size(refInd,2);
    
    % Get subcarrier indices 'k' used by the DM-RS, corresponding
    % PRG indices 'prg', and set of unique PRGs 'uprg'
    [k,~,~] = ind2sub([K L],refInd);
    [prg,uprg,prgInfo] = hPRGIndices(carrier,bundleSize,k(:,1));
    
    % Perform channel estimation for each PRG
    Hest = zeros([K L R P]);
    nVarPRGs = zeros(prgInfo.NPRG,1);
    for i = 1:numel(uprg)
    
        [HPRG,nVarPRGs(uprg(i))] = nrChannelEstimate(rxGrid,refInd(prg==uprg(i),:),refSym(prg==uprg(i),:),varargin{:});
        Hest = Hest + HPRG;
    
    end
    
    noiseEst = nVarPRGs(uprg);

end

function [prg,uprg,prgInfo] = hPRGIndices(carrier,bundleSize,k)
% Calculate PRG indices 'prg', and set of unique PRGs 'uprg' for subcarrier
% indices 'k'

    prgInfo = nrPRGInfo(carrier,bundleSize);
    rb = floor((k-1)/12);
    prg = prgInfo.PRGSet(rb+1);
    uprg = unique(prg).';

end

function [Wprg,Hprg] = hSVDPrecoders(carrier,pdsch,H,prgsize)
%hSVDPrecoders Get PRG precoders from a channel estimate using SVD

%   Copyright 2023-2025 The MathWorks, Inc.

    % Get PRG information
    prgInfo = nrPRGInfo(carrier,prgsize);

    % Average the channel estimate across all subcarriers in each RB and
    % across all OFDM symbols, extract allocated RBs, then permute to shape
    % R-by-P-by-NPRB where R is the number of receive antennas, P is the
    % number of transmit antennas and NPRB is the number of allocated RBs
    gridrbset = getGridRBSet(carrier,pdsch);
    [K,L,R,P] = size(H);
    H = reshape(H,[12 K/12 L R P]);
    H = mean(H,[1 3]);
    H = H(:,gridrbset + 1,:,:,:);
    H = permute(H,[4 5 2 1 3]);

    % For each PRG
    nu = pdsch.NumLayers;
    Wprg = zeros([nu P prgInfo.NPRG]);
    Hprg = zeros([R P prgInfo.NPRG]);
    pdschPRGs = prgInfo.PRGSet(gridrbset + 1);
    uprg = unique(pdschPRGs).';
    for i = uprg

        % Average the channel estimate across all allocated RBs in the PRG
        thisPRG = (pdschPRGs==i);
        Havg = mean(H(:,:,thisPRG),3);
        Hprg(:,:,i) = Havg;

        % Get SVD-based precoder for the PRG
        [~,~,V] = svd(Havg);
        W = permute(V(:,1:nu,:),[2 1 3]);
        W = W / sqrt(nu);
        Wprg(:,:,i) = W;
    end
end

% Get allocated RBs in the carrier resource grid i.e. relative to
% NStartGrid
function gridrbset = getGridRBSet(carrier,pdsch)

    if (pdsch.VRBToPRBInterleaving)
        [~,indinfo] = nrPDSCHIndices(carrier,pdsch);
        gridrbset = indinfo.PRBSet;
    else
        gridrbset = pdsch.PRBSet;
    end

    if (isempty(pdsch.NStartBWP))
        bwpOffset = 0;
    else
        bwpOffset = pdsch.NStartBWP - carrier.NStartGrid;
    end
    gridrbset = gridrbset + bwpOffset;
end
