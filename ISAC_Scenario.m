function simResult = ISAC_Scenario(algorithmConfig, scenarioName, simOptions)
%ISAC_SCENARIO Run the full AIM-UKF-JPDA ISAC simulation pipeline.
%   simResult = ISAC_Scenario(algorithmConfig, scenarioName, simOptions)
%   executes the bistatic sensing, detection, and tracking chain using the
%   specified algorithm configuration and scenario. It returns detailed
%   tracking metrics (RMSE, track losses, etc.) required by the experiment
%   framework and can optionally generate the legacy visualizations.

if nargin < 1 || isempty(algorithmConfig)
    algorithmConfig = struct();
end
if nargin < 2 || isempty(scenarioName)
    scenarioName = 'HighManeuver';
end
if nargin < 3
    simOptions = struct();
end

algorithmConfig = localApplyAlgorithmDefaults(algorithmConfig);
simOptions = localApplySimOptions(simOptions);

scenario = localSetupScenario(scenarioName, simOptions);
waveform = localConfigureWaveform(scenario, simOptions);

simResult = localRunIsacLoop(scenario, waveform, algorithmConfig, scenarioName, simOptions);

if simOptions.SaveTrackingResults
    localPersistTrackingResults(simResult, simOptions);
end

if nargout == 0
    localDisplaySummary(simResult);
end
end

function algorithmConfig = localApplyAlgorithmDefaults(algorithmConfig)
if ~isfield(algorithmConfig, 'FilterType') || isempty(algorithmConfig.FilterType)
    algorithmConfig.FilterType = 'UKF';
end
if ~isfield(algorithmConfig, 'MotionModels') || isempty(algorithmConfig.MotionModels)
    algorithmConfig.MotionModels = {'CV', 'CT', 'CA'};
end
if ~isfield(algorithmConfig, 'TrackerType') || isempty(algorithmConfig.TrackerType)
    algorithmConfig.TrackerType = 'JPDA';
end
if ~isfield(algorithmConfig, 'AdaptiveQ') || isempty(algorithmConfig.AdaptiveQ)
    algorithmConfig.AdaptiveQ = false;
end
end

function simOptions = localApplySimOptions(simOptions)
defaults.NumSensingFrames = 10;
defaults.FramePeriod = 0.4;
defaults.GenerateVisualizations = true;
defaults.SaveTrackingResults = false;
defaults.ResultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
defaults.Seed = 'shuffle';
defaults.AssignmentThreshold = 120;
defaults.MaxRange = 100;
defaults.MaxNumTracks = 10;
defaults.ShowProgress = false;
defaults.AdaptiveConfig = struct();

if isempty(simOptions)
    simOptions = defaults;
else
    f = fieldnames(defaults);
    for i = 1:numel(f)
        if ~isfield(simOptions, f{i}) || isempty(simOptions.(f{i}))
            simOptions.(f{i}) = defaults.(f{i});
        end
    end
end

if ~isfield(simOptions.AdaptiveConfig, 'Enable')
    simOptions.AdaptiveConfig.Enable = true;
end
if ~isfield(simOptions.AdaptiveConfig, 'ManeuverThreshold')
    simOptions.AdaptiveConfig.ManeuverThreshold = 0.5;
end
if ~isfield(simOptions.AdaptiveConfig, 'QBoostFactor')
    simOptions.AdaptiveConfig.QBoostFactor = 10;
end
if ~isfield(simOptions.AdaptiveConfig, 'BoostDuration')
    simOptions.AdaptiveConfig.BoostDuration = 3;
end

if ischar(simOptions.Seed) || isstring(simOptions.Seed)
    switch lower(string(simOptions.Seed))
        case "default"
            rng('default');
        case "shuffle"
            rng('shuffle');
        otherwise
            rng(str2double(string(simOptions.Seed)), 'twister');
    end
else
    rng(simOptions.Seed, 'twister');
end
state = rng;
simOptions.ActualSeed = state.Seed;

if simOptions.SaveTrackingResults && ~exist(simOptions.ResultsDir, 'dir')
    mkdir(simOptions.ResultsDir);
end
end

function scenario = localSetupScenario(scenarioName, simOptions)
carrierFrequency = 60e9;
wavelength = freq2wavelen(carrierFrequency);

txPosition = [0; 0; 0];
txPower = 46;
transmitter = phased.Transmitter('PeakPower', db2pow(txPower-30), 'Gain', 0);

numTxAntennas = 8;
element = phased.IsotropicAntennaElement('BackBaffled', true);
txArray = phased.ULA(numTxAntennas, wavelength/2, 'Element', element);
txOrientationAxes = eye(3);

rxPosition = [50; 60; 0];
receiver = phased.Receiver('AddInputNoise', true, 'Gain', 0, 'NoiseMethod', 'Noise figure', ...
    'NoiseFigure', 5, 'ReferenceTemperature', 290);
numRxAntennas = 8;
rxArray = phased.ULA(numRxAntennas, wavelength/2, 'Element', element);
rxOrientationAxes = rotz(-90);

regionOfInterest = [0 120; -80 80];
numScatterers = 40;
[scatterers.Positions, scatterers.ReflectionCoefficients] = ...
    helperGenerateStaticScatterers(numScatterers, regionOfInterest);
scatterers.Velocities = zeros(size(scatterers.Positions));

targets = struct();
targets.Trajectories = helperGetTargetTrajectories(scenarioName);
numTargets = numel(targets.Trajectories);
targets.ReflectionCoefficients = exp(1i*(2*pi*rand(1, numTargets)));

channel = phased.ScatteringMIMOChannel('CarrierFrequency', carrierFrequency, ...
    'TransmitArray', txArray, 'TransmitArrayPosition', txPosition, ...
    'ReceiveArray', rxArray, 'ReceiveArrayPosition', rxPosition, ...
    'TransmitArrayOrientationAxes', txOrientationAxes, ...
    'ReceiveArrayOrientationAxes', rxOrientationAxes, ...
    'SimulateDirectPath', true, 'ScattererSpecificationSource', 'Input Port', ...
    'ChannelResponseOutputPort', true, 'Polarization', 'None');

if simOptions.GenerateVisualizations
    helperVisualizeScatteringMIMOChannel(channel, scatterers.Positions, targets.Trajectories);
    title('ISAC Scenario');
end

scenario.carrierFrequency = carrierFrequency;
scenario.wavelength = wavelength;
scenario.txPosition = txPosition;
scenario.rxPosition = rxPosition;
scenario.txOrientationAxes = txOrientationAxes;
scenario.rxOrientationAxes = rxOrientationAxes;
scenario.transmitter = transmitter;
scenario.receiver = receiver;
scenario.txArray = txArray;
scenario.rxArray = rxArray;
scenario.numTxAntennas = numTxAntennas;
scenario.numRxAntennas = numRxAntennas;
scenario.channel = channel;
scenario.scatterers = scatterers;
scenario.targets = targets;
scenario.numTargets = numTargets;
end

function waveform = localConfigureWaveform(scenario, simOptions)
channelBandwidth = 50;
bandwidthOccupancy = 0.9;
subcarrierSpacing = 60;

waveformConfig = helperGet5GWaveformConfiguration(channelBandwidth, bandwidthOccupancy, subcarrierSpacing);
waveformInfo = nrOFDMInfo(waveformConfig.Carrier);

transmissionBandwidth = subcarrierSpacing*1e3*waveformConfig.Carrier.NSizeGrid*12;
scenario.channel.SampleRate = waveformInfo.SampleRate;

waveformConfig.PDSCH.DMRS.DMRSTypeAPosition = 2;
waveformConfig.PDSCH.DMRS.DMRSLength = 1;
waveformConfig.PDSCH.DMRS.DMRSAdditionalPosition = 2;
waveformConfig.PDSCH.DMRS.NumCDMGroupsWithoutData = 1;
waveformConfig.PDSCH.DMRS.DMRSConfigurationType = 1;

if simOptions.GenerateVisualizations
    resourceGrid = nrResourceGrid(waveformConfig.Carrier, waveformConfig.PDSCH.NumLayers);
    ind = nrPDSCHDMRSIndices(waveformConfig.Carrier, waveformConfig.PDSCH);
    resourceGrid(ind) = nrPDSCHDMRS(waveformConfig.Carrier, waveformConfig.PDSCH);
    helperVisualizeResourceGrid(abs(resourceGrid(1:12,1:14,1)));
    title({'PDSCH DM-RS Resource Grid', '(single resource block)'});
end

[~, pdschInfo] = nrPDSCHIndices(waveformConfig.Carrier, waveformConfig.PDSCH);
Mt = max(diff(pdschInfo.DMRSSymbolSet));
ofdmSymbolDuration = mode(waveformInfo.SymbolLengths)/waveformInfo.SampleRate;
maxDoppler = 1/(2*Mt*ofdmSymbolDuration);
maxVelocity = dop2speed(maxDoppler, scenario.wavelength)/2;
Mf = min(diff(waveformConfig.PDSCH.DMRS.DMRSSubcarrierLocations));
maxDelay = 1/(2*Mf*subcarrierSpacing*1e3);
maxRange = time2range(maxDelay);
rangeResolution = bw2rangeres(transmissionBandwidth);

waveform.waveformConfig = waveformConfig;
waveform.waveformInfo = waveformInfo;
waveform.transmissionBandwidth = transmissionBandwidth;
waveform.rangeResolution = rangeResolution;
waveform.maxRange = maxRange;
waveform.maxVelocity = maxVelocity;
waveform.numSubcarriers = waveformConfig.Carrier.NSizeGrid*12;
waveform.numSymbolsPerFrame = waveformInfo.SlotsPerFrame * waveformInfo.SymbolsPerSlot;
end

function simResult = localRunIsacLoop(scenario, waveform, algorithmConfig, scenarioName, simOptions)
numSensingFrames = simOptions.NumSensingFrames;
dt = simOptions.FramePeriod;
t = (0:numSensingFrames-1)*dt;

txPosition = scenario.txPosition;
rxPosition = scenario.rxPosition;
txOrientationAxes = scenario.txOrientationAxes;
rxOrientationAxes = scenario.rxOrientationAxes;
transmitter = scenario.transmitter;
receiver = scenario.receiver;
channel = scenario.channel;
scatterers = scenario.scatterers;
targets = scenario.targets;
numTargets = scenario.numTargets;

carrierFrequency = scenario.carrierFrequency;
wavelength = scenario.wavelength;
numTxAntennas = scenario.numTxAntennas;
numRxAntennas = scenario.numRxAntennas;

waveformConfig = waveform.waveformConfig;
waveformInfo = waveform.waveformInfo;
rangeResolution = waveform.rangeResolution;
maxRange = min(simOptions.MaxRange, waveform.maxRange);
numSubcarriers = waveform.numSubcarriers;
numSymbolsPerFrame = waveform.numSymbolsPerFrame;

numAoA = ceil(2*pi*numRxAntennas);
aoaGrid = linspace(-90, 90, numAoA);
rxSteeringVector = phased.SteeringVector(SensorArray=scenario.rxArray);
rxsv = rxSteeringVector(carrierFrequency, aoaGrid);

numAoD = ceil(2*pi*numTxAntennas);
aodGrid = linspace(-90, 90, numAoD);
txSteeringVector = phased.SteeringVector(SensorArray=scenario.txArray);
txsv = txSteeringVector(carrierFrequency, aodGrid);

rangeGrid = -20:rangeResolution/2:maxRange;
baseline = vecnorm(txPosition - rxPosition);
rangeFFTBins = (rangeGrid+baseline)/(2*rangeResolution);
k = (0:numSubcarriers-1).';
rngsv = (exp(2*pi*1i*k*rangeFFTBins/numSubcarriers)/numSubcarriers);

aoaResolution = ap2beamwidth((numRxAntennas-1)*scenario.rxArray.ElementSpacing, wavelength);
numGuardCellsRange = ceil(rangeResolution/(rangeGrid(2)-rangeGrid(1))) + 1;
numGuardCellsAoA = ceil(aoaResolution/(aoaGrid(2)-aoaGrid(1))) + 1;

cfar2D = phased.CFARDetector2D('GuardBandSize', [numGuardCellsRange numGuardCellsAoA], ...
    'TrainingBandSize', [4 6], 'ProbabilityFalseAlarm', 1e-3, ...
    'OutputFormat', 'CUT result', 'ThresholdOutputPort', false);

offsetIdxs = cfar2D.TrainingBandSize + cfar2D.GuardBandSize;
rangeIdxs = offsetIdxs(1)+1:numel(rangeGrid)-offsetIdxs(1);
angleIdxs = offsetIdxs(2)+1:numel(aoaGrid)-offsetIdxs(2);
[sumRangeIdxs, aoaIdxs] = meshgrid(rangeIdxs, angleIdxs);
cutidx = [sumRangeIdxs(:).'; aoaIdxs(:).'];

clusterer = clusterDBSCAN();
tracker = helperConfigureTracker(txPosition, rxPosition, txOrientationAxes, rxOrientationAxes, ...
    rangeResolution, aoaResolution, algorithmConfig);

stateDim = 6;
maxNumTracks = simOptions.MaxNumTracks;
numModels = numel(algorithmConfig.MotionModels);
targetStateEstimates = nan(stateDim, numSensingFrames, maxNumTracks);
modelProbabilities = nan(numModels, numSensingFrames, maxNumTracks);
trackIDList = nan(1, maxNumTracks);
activeTrackCount = 0;

posErrors = nan(numSensingFrames, 1);
velErrors = nan(numSensingFrames, 1);
frameDurations = nan(numSensingFrames, 1);
trackLosses = 0;
trackSwaps = 0;

truthPositionsXYZ = nan(numTargets, 3, numSensingFrames);
truthPositionsXY = nan(numTargets, 2, numSensingFrames);
truthVelocitiesXY = zeros(numTargets, 2, numSensingFrames);

generateVisualizations = simOptions.GenerateVisualizations;
if generateVisualizations
    scenarioCFARResultsFigure = figure('Name', 'ISAC Scenario Visualization');
    tl = tiledlayout(scenarioCFARResultsFigure, 1, 2, 'Padding', 'tight', 'TileSpacing', 'tight');
    scenarioPlotter = helperTheaterPlotter(tl);
    scenarioPlotter.plotTxAndRx(txPosition, rxPosition);
    scenarioPlotter.plotTrajectories(targets.Trajectories, t);
    scenarioPlotter.plotScatterers(scatterers.Positions);
    title(scenarioPlotter, 'ISAC Scenario');

    cfarResultsVisualizer = helperCFARResultsVisualizer(tl);
    xlabel(cfarResultsVisualizer, 'AoA (degrees)');
    ylabel(cfarResultsVisualizer, 'Bistatic Range (m)');
    title(cfarResultsVisualizer, 'CFAR Results');
else
    scenarioCFARResultsFigure = [];
    scenarioPlotter = [];
    cfarResultsVisualizer = [];
end

symbolDuration = waveformInfo.SymbolLengths(1) / waveformInfo.SampleRate;
dopplerResolution = 1 / (numSymbolsPerFrame * symbolDuration);
maxDoppler = dopplerResolution * numSymbolsPerFrame / 2;
dopplerGrid = linspace(-maxDoppler, maxDoppler, numSymbolsPerFrame);
velocityGrid = dopplerGrid * wavelength / 2;

assignmentMap = containers.Map('KeyType', 'double', 'ValueType', 'double');

for i = 1:numSensingFrames
    frameTimer = tic;
    if simOptions.ShowProgress
        fprintf('Simulating frame %d/%d at time %.2f s\n', i, numSensingFrames, t(i));
    end

    sensingCSI = helperSimulateLinkSingleFrame(transmitter, receiver, channel, ...
        waveformConfig, scatterers, targets, t(i));
    reset(channel);

    X = fft(sensingCSI, [], 2);
    X(:, 1, :, :) = 0;
    x = ifft(X, [], 2);

    x = permute(x, [4 1 3 2]);
    x = txsv' * reshape(x, numTxAntennas, []);

    x = permute(reshape(x, numAoA, numSubcarriers, numRxAntennas, numSymbolsPerFrame), [2 3 1 4]);
    x = rngsv.' * reshape(x, numSubcarriers, []);

    x = permute(reshape(x, numel(rangeGrid), numRxAntennas, numAoD, numSymbolsPerFrame), [2 1 3 4]);
    x = rxsv' * reshape(x, numRxAntennas, []);
    x_4D = reshape(x, numAoA, numel(rangeGrid), numAoD, numSymbolsPerFrame);

    x_R_A_D = fft(x_4D, [], 4);
    x_R_A_D = fftshift(x_R_A_D, 4);
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
        m = numel(uniqClusterIds);
        clusteredDetections = zeros(3, m);
        for j = 1:m
            idxs = clusterids == uniqClusterIds(j);
            dvals = detectionValues(:, idxs);
            clusteredDetections(:, j) = mean(dvals, 2);
        end
    else
        clusteredDetections = zeros(3, 0);
    end

    detections = helperFormatDetectionsForTracker(clusteredDetections, t(i), rangeResolution, aoaResolution);
    tracks = tracker(detections, t(i));
    if algorithmConfig.AdaptiveQ
        tracks = helperAdaptiveProcessNoise(tracks, simOptions.AdaptiveConfig);
    end

    for it = 1:numel(tracks)
        id = tracks(it).TrackID;
        idx = find(trackIDList(1:activeTrackCount) == id, 1);
        if isempty(idx)
            if activeTrackCount >= maxNumTracks
                warning('ISAC_Scenario:MaxTracksReached', ...
                    'Maximum number of track slots (%d) reached. Track %d skipped.', maxNumTracks, id);
                continue;
            end
            activeTrackCount = activeTrackCount + 1;
            trackIDList(activeTrackCount) = id;
            idx = activeTrackCount;
        end

        trackState = tracks(it).State;
        targetStateEstimates(:, i, idx) = NaN;
        numStatesToStore = min(numel(trackState), size(targetStateEstimates, 1));
        targetStateEstimates(1:numStatesToStore, i, idx) = trackState(1:numStatesToStore);

        mp = nan(numModels, 1);
        try
            probs = getTrackFilterProperties(tracker, id, 'ModelProbabilities');
            mp(1:min(numModels, numel(probs))) = probs(1:min(numModels, numel(probs)));
        catch
            % Leave NaNs if unavailable
        end
        modelProbabilities(:, i, idx) = mp;
    end

    targetPositions = zeros(numTargets, 3);
    for it = 1:numTargets
        targetPositions(it, :) = lookupPose(targets.Trajectories{it}, t(i));
    end
    truthPositionsXYZ(:, :, i) = targetPositions;
    truthPositionsXY(:, :, i) = targetPositions(:, 1:2);
    if i == 1
        truthVelocitiesXY(:, :, i) = zeros(numTargets, 2);
    else
        truthVelocitiesXY(:, :, i) = (truthPositionsXY(:, :, i) - truthPositionsXY(:, :, i-1)) / dt;
    end

    trackPositions2D = zeros(numel(tracks), 2);
    trackVelocities2D = zeros(numel(tracks), 2);
    for it = 1:numel(tracks)
        [posXY, velXY] = localExtractStateComponents(tracks(it).State);
        trackPositions2D(it, :) = posXY;
        trackVelocities2D(it, :) = velXY;
    end

    truePos2D = reshape(truthPositionsXY(:, :, i), numTargets, 2);
    trueVel2D = reshape(truthVelocitiesXY(:, :, i), numTargets, 2);
    [assignments, unassignedTruth] = localAssignTracks(truePos2D, trackPositions2D, simOptions.AssignmentThreshold);
    trackLosses = trackLosses + numel(unassignedTruth);

    if ~isempty(assignments)
        posErrTmp = zeros(size(assignments, 1), 1);
        velErrTmp = zeros(size(assignments, 1), 1);
        for aIdx = 1:size(assignments, 1)
            truthIdx = assignments(aIdx, 1);
            trackIdx = assignments(aIdx, 2);
            posDiff = trackPositions2D(trackIdx, :) - truePos2D(truthIdx, :);
            velDiff = trackVelocities2D(trackIdx, :) - trueVel2D(truthIdx, :);
            posErrTmp(aIdx) = norm(posDiff);
            velErrTmp(aIdx) = norm(velDiff);

            if trackIdx <= numel(tracks)
                trackID = tracks(trackIdx).TrackID;
                if isKey(assignmentMap, trackID)
                    prevTruth = assignmentMap(trackID);
                    if ~isnan(prevTruth) && prevTruth ~= truthIdx
                        trackSwaps = trackSwaps + 1;
                    end
                end
                assignmentMap(trackID) = truthIdx;
            end
        end
        posErrors(i) = sqrt(mean(posErrTmp.^2));
        velErrors(i) = sqrt(mean(velErrTmp.^2));
    else
        posErrors(i) = NaN;
        velErrors(i) = NaN;
    end

    if generateVisualizations
        scenarioPlotter.plotTargetPositions(targetPositions);
        clusteredDetections2D = clusteredDetections(1:min(2, size(clusteredDetections, 1)), :);
        if ~isempty(clusteredDetections2D)
            measuredPositions = helperGetCartesianMeasurement(clusteredDetections2D, txPosition, rxPosition, rxOrientationAxes);
            scenarioPlotter.plotDetections(measuredPositions);
        end
        scenarioPlotter.plotTracks(tracks);

        if numTargets > 0
            [trueRrx, trueAoA] = rangeangle(targetPositions.', rxPosition, rxOrientationAxes);
            [trueRtx, ~] = rangeangle(targetPositions.', txPosition, txOrientationAxes);
            trueBistaticRange = trueRrx + trueRtx - baseline;
        else
            trueBistaticRange = zeros(1, 0);
            trueAoA = zeros(2, 0);
        end

        cfarResultsVisualizer.plotCFARImage(aoaGrid, rangeGrid, rangeAoAMap);
        if ~isempty(trueBistaticRange)
            cfarResultsVisualizer.plotTruth([trueBistaticRange; trueAoA(1, :)]);
        end
        if ~isempty(clusteredDetections2D)
            cfarResultsVisualizer.plotClusteredDetections(clusteredDetections2D);
        end
        sgtitle(scenarioCFARResultsFigure, sprintf('Frame %d, Simulation Time %.1f s', i, t(i)), 'FontSize', 10);
    end

    frameDurations(i) = toc(frameTimer);
end

simResult = struct();
simResult.posRMSE = posErrors;
simResult.velRMSE = velErrors;
simResult.trackLosses = trackLosses;
simResult.trackSwaps = trackSwaps;
simResult.avgProcessingTime = mean(frameDurations, 'omitnan') * 1e3;
simResult.targetStateEstimates = targetStateEstimates;
simResult.modelProbabilities = modelProbabilities;
simResult.trackIDs = trackIDList(1:activeTrackCount);
simResult.timeVector = t;
simResult.truePositionsXYZ = truthPositionsXYZ;
simResult.truePositionsXY = truthPositionsXY;
simResult.trueVelocitiesXY = truthVelocitiesXY;
simResult.scenarioName = scenarioName;
simResult.algorithmConfig = algorithmConfig;
simResult.seed = simOptions.ActualSeed;
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

function localPersistTrackingResults(simResult, simOptions)
if ~exist(simOptions.ResultsDir, 'dir')
    mkdir(simOptions.ResultsDir);
end
resultFile = fullfile(simOptions.ResultsDir, 'trackingResults.mat');
targetStateEstimates = simResult.targetStateEstimates;
modelProbabilities = simResult.modelProbabilities;
t = simResult.timeVector;
trackIDs = simResult.trackIDs;
truePositions = simResult.truePositionsXYZ;
save(resultFile, 'targetStateEstimates', 'modelProbabilities', 't', 'trackIDs', 'truePositions');
fprintf('\nTracking results saved to %s\n', resultFile);
end

function localDisplaySummary(simResult)
fprintf('\n=== ISAC Scenario Summary ===\n');
fprintf('Scenario: %s\n', simResult.scenarioName);
fprintf('Algorithm: %s | Models: %s\n', simResult.algorithmConfig.FilterType, strjoin(simResult.algorithmConfig.MotionModels, '/'));
fprintf('Tracker: %s | AdaptiveQ: %d\n', string(simResult.algorithmConfig.TrackerType), simResult.algorithmConfig.AdaptiveQ);
fprintf('Mean Position RMSE: %.2f m\n', mean(simResult.posRMSE, 'omitnan'));
fprintf('Mean Velocity RMSE: %.2f m/s\n', mean(simResult.velRMSE, 'omitnan'));
fprintf('Total Track Losses: %d | Track Swaps: %d\n', simResult.trackLosses, simResult.trackSwaps);
fprintf('Avg Processing Time: %.2f ms\n', simResult.avgProcessingTime);
fprintf('===============================\n\n');
end
