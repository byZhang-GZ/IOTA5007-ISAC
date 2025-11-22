function metrics = run_ISAC_Simulation_Core(config, scenarioName)
%RUN_ISAC_SIMULATION_CORE Execute the AIM-UKF-JPDA ISAC simulation loop.
%   metrics = RUN_ISAC_SIMULATION_CORE(config, scenarioName) runs the full
%   sensing, detection, and tracking chain for the requested scenario and
%   algorithm configuration, and returns aggregate RMSE/track quality
%   metrics suitable for ablation experiments.
%
%   Inputs:
%     config       - struct with fields FilterType, MotionModels,
%                    TrackerType, AdaptiveQ (see defaults below)
%     scenarioName - name of trajectory set ('HighManeuver', 'Acceleration',
%                    or 'Crossing')
%
%   Outputs:
%     metrics      - struct with fields:
%                       posRMSE     (scalar mean position RMSE in meters)
%                       velRMSE     (scalar mean velocity RMSE in m/s)
%                       trackLosses (count of truth targets with no tracks)
%                       trackSwaps  (count of ID switches)
%
%   The core loop mirrors ISAC_Scenario.m but strips visualization and file
%   side-effects so it can be reused by batch experiments.

if nargin < 1 || isempty(config)
    config = struct();
end
if nargin < 2 || isempty(scenarioName)
    scenarioName = 'HighManeuver';
end

config = localApplyConfigDefaults(config);
disp('Config applied.');
scenario = localSetupScenario(scenarioName);
disp('Scenario setup complete.');
waveform = localConfigureWaveform(scenario);
disp('Waveform configured.');
metrics = localRunSimulationLoop(scenario, waveform, config);
disp('Simulation loop complete.');
end

function config = localApplyConfigDefaults(config)
if ~isfield(config, 'FilterType') || isempty(config.FilterType)
    config.FilterType = 'UKF';
end
if ~isfield(config, 'MotionModels') || isempty(config.MotionModels)
    config.MotionModels = {'CV', 'CT', 'CA'};
end
if ~isfield(config, 'TrackerType') || isempty(config.TrackerType)
    config.TrackerType = 'JPDA';
end
if ~isfield(config, 'AdaptiveQ') || isempty(config.AdaptiveQ)
    config.AdaptiveQ = true;
end
end

function scenario = localSetupScenario(scenarioName)
carrierFrequency = 60e9;
wavelength = freq2wavelen(carrierFrequency);

% Transmitter definition
scenario.txPosition = [0; 0; 0];
scenario.txPower = 46;
scenario.txOrientationAxes = eye(3);
scenario.numTxAntennas = 8;
scenario.txElement = phased.IsotropicAntennaElement('BackBaffled', true);
scenario.txArray = phased.ULA(scenario.numTxAntennas, wavelength/2, ...
    'Element', scenario.txElement);
scenario.transmitter = phased.Transmitter('PeakPower', db2pow(scenario.txPower-30), ...
    'Gain', 0);

% Receiver definition
scenario.rxPosition = [50; 60; 0];
scenario.rxOrientationAxes = rotz(-90);
scenario.numRxAntennas = 8;
scenario.rxArray = phased.ULA(scenario.numRxAntennas, wavelength/2, ...
    'Element', scenario.txElement);
scenario.receiver = phased.Receiver('AddInputNoise', true, 'Gain', 0, ...
    'NoiseMethod', 'Noise figure', 'NoiseFigure', 5, ...
    'ReferenceTemperature', 290);

% Static scatterers
regionOfInterest = [0 120; -80 80];
numScatterers = 40;
[scatterPos, scatterRef] = helperGenerateStaticScatterers(numScatterers, regionOfInterest);
scenario.scatterers.Positions = scatterPos;
scenario.scatterers.ReflectionCoefficients = scatterRef;
scenario.scatterers.Velocities = zeros(size(scatterPos));

% Target trajectories
scenario.targets = struct();
scenario.targets.Trajectories = helperGetTargetTrajectories(scenarioName);
numTargets = numel(scenario.targets.Trajectories);
scenario.targets.ReflectionCoefficients = exp(1i*(2*pi*rand(1, numTargets)));
scenario.numTargets = numTargets;

% Channel
disp('Creating ScatteringMIMOChannel...');
scenario.channel = phased.ScatteringMIMOChannel('CarrierFrequency', carrierFrequency, ...
    'TransmitArray', scenario.txArray, 'TransmitArrayPosition', scenario.txPosition, ...
    'ReceiveArray', scenario.rxArray, 'ReceiveArrayPosition', scenario.rxPosition, ...
    'TransmitArrayOrientationAxes', scenario.txOrientationAxes, ...
    'ReceiveArrayOrientationAxes', scenario.rxOrientationAxes, ...
    'SimulateDirectPath', true, 'ScattererSpecificationSource', 'Input Port', ...
    'ChannelResponseOutputPort', true, 'Polarization', 'None');
disp('Channel created.');

scenario.carrierFrequency = carrierFrequency;
scenario.wavelength = wavelength;
end

function waveform = localConfigureWaveform(scenario)
channelBandwidth = 50;
bandwidthOccupancy = 0.9;
subcarrierSpacing = 60;

waveform.waveformConfig = helperGet5GWaveformConfiguration(channelBandwidth, ...
    bandwidthOccupancy, subcarrierSpacing);
waveform.waveformInfo = nrOFDMInfo(waveform.waveformConfig.Carrier);
waveform.waveformConfig.PDSCH.DMRS.DMRSTypeAPosition = 2;
waveform.waveformConfig.PDSCH.DMRS.DMRSLength = 1;
waveform.waveformConfig.PDSCH.DMRS.DMRSAdditionalPosition = 2;
waveform.waveformConfig.PDSCH.DMRS.NumCDMGroupsWithoutData = 1;
waveform.waveformConfig.PDSCH.DMRS.DMRSConfigurationType = 1;

scenario.channel.SampleRate = waveform.waveformInfo.SampleRate;
waveform.numSymbolsPerFrame = waveform.waveformInfo.SlotsPerFrame * ...
    waveform.waveformInfo.SymbolsPerSlot;
waveform.numSubcarriers = waveform.waveformConfig.Carrier.NSizeGrid * 12;
waveform.transmissionBandwidth = subcarrierSpacing*1e3*waveform.waveformConfig.Carrier.NSizeGrid*12;
waveform.rangeResolution = bw2rangeres(waveform.transmissionBandwidth);
waveform.maxRange = time2range(1/(2*min(diff(waveform.waveformConfig.PDSCH.DMRS.DMRSSubcarrierLocations))*subcarrierSpacing*1e3));
end

function metrics = localRunSimulationLoop(scenario, waveform, config)
disp('Entering localRunSimulationLoop...');
numSensingFrames = 10;
framePeriod = 0.4;
assignmentThreshold = 120;

rng('shuffle');

% Convenience aliases
carrierFrequency = scenario.carrierFrequency;
wavelength = scenario.wavelength;
numTxAntennas = scenario.numTxAntennas;
numRxAntennas = scenario.numRxAntennas;

waveformConfig = waveform.waveformConfig;
waveformInfo = waveform.waveformInfo;
rangeResolution = waveform.rangeResolution;
maxRange = min(100, waveform.maxRange);
numSymbolsPerFrame = waveform.numSymbolsPerFrame;
numSubcarriers = waveform.numSubcarriers;

% Steering vectors and grids
disp('Initializing steering vectors...');
numAoA = ceil(2*pi*numRxAntennas);
aoaGrid = linspace(-90, 90, numAoA);
rxSteeringVector = phased.SteeringVector(SensorArray=scenario.rxArray);
rxsv = rxSteeringVector(carrierFrequency, aoaGrid);
disp('Rx steering vector initialized.');

numAoD = ceil(2*pi*numTxAntennas);
aodGrid = linspace(-90, 90, numAoD);
txSteeringVector = phased.SteeringVector(SensorArray=scenario.txArray);
txsv = txSteeringVector(carrierFrequency, aodGrid);
disp('Tx steering vector initialized.');

rangeGrid = -20:rangeResolution/2:maxRange;
baseline = vecnorm(scenario.txPosition - scenario.rxPosition);
rangeFFTBins = (rangeGrid + baseline)/(2*rangeResolution);
k = (0:numSubcarriers-1).';
rngsv = (exp(2*pi*1i*k*rangeFFTBins/numSubcarriers)/numSubcarriers);

% CFAR configuration
disp('Configuring CFAR...');
numGuardCellsRange = ceil(rangeResolution/(rangeGrid(2)-rangeGrid(1))) + 1;
aoaResolution = ap2beamwidth((numRxAntennas-1)*scenario.rxArray.ElementSpacing, wavelength);
numGuardCellsAoA = ceil(aoaResolution/(aoaGrid(2)-aoaGrid(1))) + 1;
cfar2D = phased.CFARDetector2D('GuardBandSize',[numGuardCellsRange numGuardCellsAoA], ...
    'TrainingBandSize', [4 6], 'ProbabilityFalseAlarm', 1e-3, ...
    'OutputFormat', 'CUT result', 'ThresholdOutputPort', false);
disp('CFAR configured.');

offsetIdxs = cfar2D.TrainingBandSize + cfar2D.GuardBandSize;
rangeIdxs = offsetIdxs(1)+1:numel(rangeGrid)-offsetIdxs(1);
angleIdxs = offsetIdxs(2)+1:numel(aoaGrid)-offsetIdxs(2);
[sumRangeIdxs, aoaIdxs] = meshgrid(rangeIdxs, angleIdxs);
cutidx = [sumRangeIdxs(:).'; aoaIdxs(:).'];

disp('Configuring Tracker...');
clusterer = clusterDBSCAN();
tracker = helperConfigureTracker(scenario.txPosition, scenario.rxPosition, ...
    scenario.txOrientationAxes, scenario.rxOrientationAxes, rangeResolution, ...
    aoaResolution, config);
disp('Tracker configured.');

posErrors = nan(numSensingFrames, 1);
velErrors = nan(numSensingFrames, 1);
trackLosses = 0;
trackSwaps = 0;
assignmentMap = containers.Map('KeyType', 'double', 'ValueType', 'double');

t = (0:numSensingFrames-1)*framePeriod;

symbolDuration = waveformInfo.SymbolLengths(1)/waveformInfo.SampleRate;
dopplerResolution = 1/(numSymbolsPerFrame * symbolDuration);
maxDoppler = dopplerResolution * numSymbolsPerFrame / 2;
dopplerGrid = linspace(-maxDoppler, maxDoppler, numSymbolsPerFrame);
velocityGrid = dopplerGrid * wavelength / 2;

for frameIdx = 1:numSensingFrames
    fprintf('  Processing Frame %d/%d...\n', frameIdx, numSensingFrames);
    
    try
        sensingCSI = helperSimulateLinkSingleFrame(scenario.transmitter, scenario.receiver, ...
            scenario.channel, waveformConfig, scenario.scatterers, scenario.targets, t(frameIdx));
        fprintf('  Frame %d simulated. Processing...\n', frameIdx);
    catch ME
        fprintf('  Error in helperSimulateLinkSingleFrame at frame %d: %s\n', frameIdx, ME.message);
        fprintf('  Stack trace:\n');
        for k = 1:length(ME.stack)
            fprintf('    %s:%d\n', ME.stack(k).name, ME.stack(k).line);
        end
        rethrow(ME); 
    end
    reset(scenario.channel);

    X = fft(sensingCSI, [], 2);
    X(:, 1, :, :) = 0;
    x = ifft(X, [], 2);

    x = permute(x, [4 1 3 2]);
    x = txsv' * reshape(x, numTxAntennas, []);
    x = permute(reshape(x, numAoA, numSubcarriers, numRxAntennas, numSymbolsPerFrame), [2 3 1 4]);
    x = rngsv.' * reshape(x, numSubcarriers, []);
    x = permute(reshape(x, numel(rangeGrid), numRxAntennas, numAoD, numSymbolsPerFrame), [2 1 3 4]);
    x = rxsv' * reshape(x, numRxAntennas, []);
    x4d = reshape(x, numAoA, numel(rangeGrid), numAoD, numSymbolsPerFrame);

    x_R_A_D = fftshift(fft(x4d, [], 4), 4);
    rangeAoAMap = sqrt(sum(abs(x_R_A_D).^2, [3 4])).';

    cutResult = cfar2D(rangeAoAMap, cutidx);
    detectionIdxs = cutidx(:, cutResult);
    detectionValues = [rangeGrid(detectionIdxs(1, :)); aoaGrid(detectionIdxs(2, :))];

    numDetections = size(detectionIdxs, 2);
    dopplerValues = zeros(1, numDetections);
    for d = 1:numDetections
        rangeIdx = detectionIdxs(1, d);
        aoaIdx = detectionIdxs(2, d);
        dopplerSlice = squeeze(sqrt(sum(abs(x_R_A_D(aoaIdx, rangeIdx, :, :)).^2, 3)));
        [~, dopplerIdx] = max(dopplerSlice);
        dopplerValues(d) = velocityGrid(dopplerIdx);
    end
    detectionValues = [detectionValues; dopplerValues];

    if numDetections > 0
        [~, clusterids] = clusterer(detectionValues.');
        uniqClusterIds = unique(clusterids);
        clusteredDetections = zeros(3, numel(uniqClusterIds));
        for j = 1:numel(uniqClusterIds)
            idxs = clusterids == uniqClusterIds(j);
            clusteredDetections(:, j) = mean(detectionValues(:, idxs), 2);
        end
    else
        clusteredDetections = zeros(3, 0);
    end

    detections = helperFormatDetectionsForTracker(clusteredDetections, t(frameIdx), ...
        rangeResolution, aoaResolution);

    tracks = tracker(detections, t(frameIdx));
    if config.AdaptiveQ
        tracks = helperAdaptiveProcessNoise(tracks, struct());
    end

    % Ground truth positions/velocities
    truePos = zeros(scenario.numTargets, 2);
    trueVel = zeros(scenario.numTargets, 2);
    for it = 1:scenario.numTargets
        pose = lookupPose(scenario.targets.Trajectories{it}, t(frameIdx));
        truePos(it, :) = pose(1:2);
        if frameIdx == 1
            trueVel(it, :) = [0 0];
        else
            prevPose = lookupPose(scenario.targets.Trajectories{it}, t(frameIdx-1));
            trueVel(it, :) = (pose(1:2) - prevPose(1:2))/framePeriod;
        end
    end

    trackPos = zeros(numel(tracks), 2);
    trackVel = zeros(numel(tracks), 2);
    for it = 1:numel(tracks)
        [p, v] = localExtractStateComponents(tracks(it).State);
        trackPos(it, :) = p;
        trackVel(it, :) = v;
    end

    [assignments, unassignedTruth] = localAssignTracks(truePos, trackPos, assignmentThreshold);
    trackLosses = trackLosses + numel(unassignedTruth);

    if ~isempty(assignments)
        posErrTmp = zeros(size(assignments, 1), 1);
        velErrTmp = zeros(size(assignments, 1), 1);
        for a = 1:size(assignments, 1)
            truthIdx = assignments(a, 1);
            trackIdx = assignments(a, 2);
            posDiff = trackPos(trackIdx, :) - truePos(truthIdx, :);
            velDiff = trackVel(trackIdx, :) - trueVel(truthIdx, :);
            posErrTmp(a) = norm(posDiff);
            velErrTmp(a) = norm(velDiff);

            trackID = tracks(trackIdx).TrackID;
            if isKey(assignmentMap, trackID)
                prevTruth = assignmentMap(trackID);
                if prevTruth ~= truthIdx
                    trackSwaps = trackSwaps + 1;
                end
            end
            assignmentMap(trackID) = truthIdx;
        end
        posErrors(frameIdx) = sqrt(mean(posErrTmp.^2));
        velErrors(frameIdx) = sqrt(mean(velErrTmp.^2));
    end
end

metrics = struct();
metrics.posRMSE = mean(posErrors, 'omitnan');
metrics.velRMSE = mean(velErrors, 'omitnan');
metrics.trackLosses = trackLosses;
metrics.trackSwaps = trackSwaps;
end

function [assignments, unassignedTruth] = localAssignTracks(truePos, trackPos, maxDist)
if isempty(truePos)
    assignments = zeros(0, 2);
    unassignedTruth = [];
    return;
end
if isempty(trackPos)
    assignments = zeros(0, 2);
    unassignedTruth = (1:size(truePos, 1)).';
    return;
end

numTruth = size(truePos, 1);
numTracks = size(trackPos, 1);
truthAssigned = false(numTruth, 1);
trackAssigned = false(numTracks, 1);
assignments = zeros(0, 2);

while true
    bestDist = inf;
    bestTruth = 0;
    bestTrack = 0;
    for ti = 1:numTruth
        if truthAssigned(ti)
            continue;
        end
        for tr = 1:numTracks
            if trackAssigned(tr)
                continue;
            end
            dist = norm(truePos(ti, :) - trackPos(tr, :));
            if dist < bestDist
                bestDist = dist;
                bestTruth = ti;
                bestTrack = tr;
            end
        end
    end
    if bestDist <= maxDist
        assignments(end+1, :) = [bestTruth, bestTrack]; %#ok<AGROW>
        truthAssigned(bestTruth) = true;
        trackAssigned(bestTrack) = true;
    else
        break;
    end
    if all(truthAssigned) || all(trackAssigned)
        break;
    end
end

unassignedTruth = find(~truthAssigned);
end

function [posXY, velXY] = localExtractStateComponents(state)
n = numel(state);
if n >= 6
    posXY = [state(1) state(4)];
    velXY = [state(2) state(5)];
elseif n == 5
    posXY = [state(1) state(3)];
    velXY = [state(2) state(4)];
elseif n == 4
    posXY = [state(1) state(3)];
    velXY = [state(2) state(4)];
elseif n >= 2
    posXY = [state(1) 0];
    if n >= 3
        posXY(2) = state(3);
    end
    velXY = [state(2) 0];
    if n >= 4
        velXY(2) = state(4);
    end
else
    posXY = [0 0];
    velXY = [0 0];
    if n >= 1
        posXY(1) = state(1);
    end
end
end
