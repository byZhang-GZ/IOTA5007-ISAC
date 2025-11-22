%% AIM-UKF-JPDA Experiment Framework
% This script implements ablation studies comparing five tracking algorithms:
%   1. Baseline-CV:    Single CV-EKF model
%   2. Baseline-IMM:   IMM(CV/CT)-EKF + GNN
%   3. Proposed-A:     IMM(CV/CT/CA)-UKF + GNN
%   4. Proposed-B:     IMM(CV/CT/CA)-UKF + Adaptive-Q + GNN
%   5. Proposed-C:     IMM(CV/CT/CA)-UKF + Adaptive-Q + JPDA (AIM-UKF-JPDA)
%
% Scenarios tested:
%   - HighManeuver:   High-speed turns
%   - Acceleration:   Acceleration/deceleration maneuvers
%   - Crossing:       Target crossing with clutter
%
% Outputs: Performance metrics (RMSE, track quality) and visualization plots

clear; close all; clc;

%% Configuration
config = struct();
config.NumMonteCarloRuns = 2;         % Number of Monte Carlo simulations
config.Scenarios = {'HighManeuver', 'Acceleration', 'Crossing'};
config.SaveResults = true;              % Save results to .mat file
config.GeneratePlots = true;            % Generate comparison plots
config.ResultsDir = 'Experiment_Results';

% Create results directory
if config.SaveResults && ~exist(config.ResultsDir, 'dir')
    mkdir(config.ResultsDir);
end

%% Algorithm Configurations
algorithms = localDefineAlgorithms();

%% Run Experiments
fprintf('\n========================================\n');
fprintf('AIM-UKF-JPDA Ablation Study\n');
fprintf('========================================\n');
fprintf('Monte Carlo Runs: %d\n', config.NumMonteCarloRuns);
fprintf('Scenarios: %s\n', strjoin(config.Scenarios, ', '));
fprintf('Algorithms: %d\n', numel(algorithms));
fprintf('========================================\n\n');

% Initialize results storage
results = struct();

for scenIdx = 1:numel(config.Scenarios)
    scenarioName = config.Scenarios{scenIdx};
    fprintf('\n>>> Processing Scenario: %s\n', scenarioName);
    
    results.(scenarioName) = struct();
    
    for algIdx = 1:numel(algorithms)
        algName = algorithms(algIdx).Name;
        fprintf('  Algorithm [%d/%d]: %s ... ', algIdx, numel(algorithms), algName);
        tic;
        
        % Run Monte Carlo simulations
        mcResults = localRunMonteCarloSimulation(algorithms(algIdx), ...
            scenarioName, config.NumMonteCarloRuns);
        
        results.(scenarioName).(algName) = mcResults;
        fprintf('Done (%.2f s)\n', toc);
    end
end

%% Generate Performance Summary
fprintf('\n>>> Generating Performance Summary...\n');
summary = localGeneratePerformanceSummary(results, algorithms, config.Scenarios);

% Display summary table
disp(summary);

%% Save Results
if config.SaveResults
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = fullfile(config.ResultsDir, sprintf('AIM_UKF_JPDA_Results_%s.mat', timestamp));
    save(filename, 'results', 'summary', 'algorithms', 'config');
    fprintf('Results saved to: %s\n', filename);
end

%% Generate Plots
if config.GeneratePlots
    fprintf('\n>>> Generating Comparison Plots...\n');
    localGenerateComparisonPlots(results, algorithms, config);
end

fprintf('\n========================================\n');
fprintf('Experiment Completed Successfully!\n');
fprintf('========================================\n\n');

%% ========================================================================
%  Local Functions
%  ========================================================================

function algorithms = localDefineAlgorithms()
%LOCALDEFINEALGORITHMS Define the five comparison algorithms.

algorithms = struct('Name', {}, 'Config', {});

% 1. Baseline-CV: Single CV model with EKF
algorithms(1).Name = 'Baseline_CV';
algorithms(1).Config = struct();
algorithms(1).Config.FilterType = 'EKF';
algorithms(1).Config.MotionModels = {'CV'};
algorithms(1).Config.TrackerType = 'GNN';
algorithms(1).Config.AdaptiveQ = false;

% 2. Baseline-IMM: IMM(CV/CT) with EKF + GNN
algorithms(2).Name = 'Baseline_IMM';
algorithms(2).Config = struct();
algorithms(2).Config.FilterType = 'EKF';
algorithms(2).Config.MotionModels = {'CV', 'CT'};
algorithms(2).Config.TrackerType = 'GNN';
algorithms(2).Config.AdaptiveQ = false;

% 3. Proposed-A: IMM(CV/CT/CA) with UKF + GNN
algorithms(3).Name = 'Proposed_A';
algorithms(3).Config = struct();
algorithms(3).Config.FilterType = 'UKF';
algorithms(3).Config.MotionModels = {'CV', 'CT', 'CA'};
algorithms(3).Config.TrackerType = 'GNN';
algorithms(3).Config.AdaptiveQ = false;

% 4. Proposed-B: IMM(CV/CT/CA) with UKF + Adaptive-Q + GNN
algorithms(4).Name = 'Proposed_B';
algorithms(4).Config = struct();
algorithms(4).Config.FilterType = 'UKF';
algorithms(4).Config.MotionModels = {'CV', 'CT', 'CA'};
algorithms(4).Config.TrackerType = 'GNN';
algorithms(4).Config.AdaptiveQ = true;

% 5. Proposed-C: AIM-UKF-JPDA (full system)
algorithms(5).Name = 'Proposed_C_AIM_UKF_JPDA';
algorithms(5).Config = struct();
algorithms(5).Config.FilterType = 'UKF';
algorithms(5).Config.MotionModels = {'CV', 'CT', 'CA'};
algorithms(5).Config.TrackerType = 'JPDA';
algorithms(5).Config.AdaptiveQ = true;
end

function mcResults = localRunMonteCarloSimulation(algorithm, scenarioName, numRuns)
%LOCALRUNMONTECARLOSIMULATION Run Monte Carlo simulation for one algorithm.

posRMSE = [];
velRMSE = [];
trackLosses = 0;
trackSwaps = 0;
processingTimes = [];
exampleRun = [];

for run = 1:numRuns
    try
        seedValue = localDeriveSeed(algorithm.Name, scenarioName, run);
        runResult = localRunSingleSimulation(algorithm, scenarioName, seedValue);
        
        posRMSE = [posRMSE; runResult.posRMSE(:).']; %#ok<AGROW>
        velRMSE = [velRMSE; runResult.velRMSE(:).']; %#ok<AGROW>
        trackLosses = trackLosses + runResult.trackLosses;
        trackSwaps = trackSwaps + runResult.trackSwaps;
        processingTimes = [processingTimes; runResult.avgProcessingTime]; %#ok<AGROW>
        if isempty(exampleRun)
            exampleRun = runResult;
        end
    catch ME
        warning('Run %d failed: %s', run, ME.message);
    end
end

mcResults = struct();
mcResults.posRMSE_mean = mean(posRMSE, 'all', 'omitnan');
mcResults.posRMSE_std = std(posRMSE(:), 0, 'omitnan');
mcResults.velRMSE_mean = mean(velRMSE, 'all', 'omitnan');
mcResults.velRMSE_std = std(velRMSE(:), 0, 'omitnan');
mcResults.trackLosses_total = trackLosses;
mcResults.trackSwaps_total = trackSwaps;
mcResults.avgProcessingTime = mean(processingTimes, 'omitnan');
mcResults.posRMSE_timeSeries = posRMSE;
mcResults.velRMSE_timeSeries = velRMSE;
mcResults.exampleRun = exampleRun;
end

function runResult = localRunSingleSimulation(algorithm, scenarioName, seedValue)
%LOCALRUNSINGLESIMULATION Execute the full ISAC simulation for one run.

if nargin < 3 || isempty(seedValue)
    seedValue = 'shuffle';
end

try
    metrics = run_ISAC_Simulation_Core(algorithm.Config, scenarioName);
catch ME
    warning('localRunSingleSimulation:Failure', ...
        'Simulation failed for %s/%s (seed=%s): %s', ...
        algorithm.Name, scenarioName, num2str(seedValue), ME.message);
    metrics = struct('posRMSE', NaN, 'velRMSE', NaN, ...
        'trackLosses', NaN, 'trackSwaps', NaN);
end

runResult = struct();
runResult.posRMSE = metrics.posRMSE;
runResult.velRMSE = metrics.velRMSE;
runResult.trackLosses = metrics.trackLosses;
runResult.trackSwaps = metrics.trackSwaps;
runResult.avgProcessingTime = NaN;
runResult.modelProbabilities = [];
runResult.timeVector = [];
runResult.trackIDs = [];
runResult.algorithmConfig = algorithm.Config;
runResult.seed = seedValue;
end

function seedValue = localDeriveSeed(algorithmName, scenarioName, runIdx)
%LOCALDERIVESEED Create deterministic seeds per algorithm/scenario/run.

if nargin < 3
    runIdx = 1;
end
nameHash = sum(double(char(algorithmName)))*7919;
scenarioHash = sum(double(char(scenarioName)))*104729;
seedValue = double(mod(nameHash + scenarioHash + runIdx*1013, 2^32 - 1));
if seedValue <= 0
    seedValue = runIdx;
end
end

function summary = localGeneratePerformanceSummary(results, algorithms, scenarios)
%LOCALGENERATEPERFORMANCESUMMARY Create summary table of all results.

numAlgs = numel(algorithms);
numScen = numel(scenarios);

% Initialize table arrays
AlgorithmName = cell(numAlgs, 1);
for i = 1:numAlgs
    AlgorithmName{i} = algorithms(i).Name;
end

% Pre-allocate columns for each scenario
varNames = {'Algorithm'};
varTypes = {'string'};

for s = 1:numScen
    scenName = scenarios{s};
    varNames = [varNames, {sprintf('%s_PosRMSE', scenName), sprintf('%s_VelRMSE', scenName)}]; %#ok<AGROW>
    varTypes = [varTypes, {'double', 'double'}]; %#ok<AGROW>
end

% Add global metrics
varNames = [varNames, {'TotalTrackLosses', 'TotalTrackSwaps', 'AvgProcessingTime_ms'}];
varTypes = [varTypes, {'double', 'double', 'double'}];

% Create table
summary = table('Size', [numAlgs, numel(varNames)], ...
    'VariableTypes', varTypes, ...
    'VariableNames', varNames);

% Fill table
for i = 1:numAlgs
    algName = algorithms(i).Name;
    summary.Algorithm(i) = string(algName);
    
    totalLosses = 0;
    totalSwaps = 0;
    allTimes = [];
    
    for s = 1:numScen
        scenName = scenarios{s};
        if isfield(results, scenName) && isfield(results.(scenName), algName)
            res = results.(scenName).(algName);
            summary.(sprintf('%s_PosRMSE', scenName))(i) = res.posRMSE_mean;
            summary.(sprintf('%s_VelRMSE', scenName))(i) = res.velRMSE_mean;
            totalLosses = totalLosses + res.trackLosses_total;
            totalSwaps = totalSwaps + res.trackSwaps_total;
            allTimes = [allTimes; res.avgProcessingTime]; %#ok<AGROW>
        end
    end
    
    summary.TotalTrackLosses(i) = totalLosses;
    summary.TotalTrackSwaps(i) = totalSwaps;
    summary.AvgProcessingTime_ms(i) = mean(allTimes);
end
end

function localGenerateComparisonPlots(results, algorithms, config)
%LOCALGENERATECOMPARISONPLOTS Generate comparison plots for paper.

% Plot 1: RMSE vs Time for HighManeuver scenario
if ismember('HighManeuver', config.Scenarios)
    figure('Name', 'RMSE Comparison - High Maneuver Scenario');
    localPlotRMSEComparison(results.HighManeuver, algorithms, 'HighManeuver');
    if config.SaveResults
        saveas(gcf, fullfile(config.ResultsDir, 'Fig1_RMSE_HighManeuver.png'));
    end
end

% Plot 2: Model Probabilities (for Proposed-C in Acceleration scenario)
if ismember('Acceleration', config.Scenarios)
    figure('Name', 'Model Probabilities - Acceleration Scenario');
    localPlotModelProbabilities(results.Acceleration, 'Acceleration');
    if config.SaveResults
        saveas(gcf, fullfile(config.ResultsDir, 'Fig2_ModelProbs_Acceleration.png'));
    end
end

% Plot 3: Ablation Study RMSE across all scenarios
figure('Name', 'Ablation Study - All Scenarios');
localPlotAblationStudy(results, algorithms, config.Scenarios);
if config.SaveResults
    saveas(gcf, fullfile(config.ResultsDir, 'Fig3_AblationStudy.png'));
end
end

function localPlotRMSEComparison(scenarioResults, ~, scenarioName)
%LOCALPLOTRMSECOMPARISON Plot RMSE time series for selected algorithms.

% Select key algorithms to plot: Baseline-CV, Baseline-IMM, Proposed-C
selectedAlgs = {'Baseline_CV', 'Baseline_IMM', 'Proposed_C_AIM_UKF_JPDA'};
colors = lines(numel(selectedAlgs));

hold on; grid on;
for i = 1:numel(selectedAlgs)
    algName = selectedAlgs{i};
    if isfield(scenarioResults, algName)
        rmseData = scenarioResults.(algName).posRMSE_timeSeries;
        if ~isempty(rmseData)
            timeSeries = mean(rmseData, 1, 'omitnan');  % Average across Monte Carlo runs
            plot(1:numel(timeSeries), timeSeries, 'LineWidth', 2, ...
                'Color', colors(i,:), 'DisplayName', strrep(algName, '_', '-'));
        end
    end
end
xlabel('Time Step');
ylabel('Position RMSE (m)');
title(sprintf('Position RMSE Comparison - %s', scenarioName));
legend('Location', 'best');
hold off;
end

function localPlotModelProbabilities(scenarioResults, scenarioName)
%LOCALPLOTMODELPROBABILITIES Plot IMM model probabilities over time.

algName = 'Proposed_C_AIM_UKF_JPDA';
if ~isfield(scenarioResults, algName)
    localPlotModelProbabilitiesPlaceholder();
    return;
end

algResult = scenarioResults.(algName);
if ~isfield(algResult, 'exampleRun') || isempty(algResult.exampleRun)
    localPlotModelProbabilitiesPlaceholder();
    return;
end

runData = algResult.exampleRun;
mpData = runData.modelProbabilities;
if isempty(mpData)
    localPlotModelProbabilitiesPlaceholder();
    return;
end

numTracks = size(mpData, 3);
trackIdx = [];
for k = 1:max(1, numTracks)
    if any(isfinite(mpData(:, :, k)), 'all')
        trackIdx = k;
        break;
    end
end
if isempty(trackIdx)
    localPlotModelProbabilitiesPlaceholder();
    return;
end

modelProbSeries = mpData(:, :, trackIdx);
timeVector = runData.timeVector;
if isempty(modelProbSeries) || isempty(timeVector)
    localPlotModelProbabilitiesPlaceholder();
    return;
end

modelNames = runData.algorithmConfig.MotionModels;
colors = lines(size(modelProbSeries, 1));
hold on; grid on;
for m = 1:size(modelProbSeries, 1)
    plot(timeVector, modelProbSeries(m, :), 'LineWidth', 2, 'Color', colors(m, :), ...
        'DisplayName', sprintf('P(%s)', modelNames{m}));
end
xlabel('Time (s)');
ylabel('Model Probability');
ylim([0 1]);
title(sprintf('IMM Model Probabilities - %s Scenario', scenarioName));
legend('Location', 'best');
hold off;
end

function localPlotModelProbabilitiesPlaceholder()
subplot(1,1,1);
hold on; grid on;
text(0.5, 0.5, 'Model Probability Data Unavailable', ...
    'HorizontalAlignment', 'center', 'FontSize', 12);
xlabel('Time (s)');
ylabel('Model Probability');
title('IMM Model Probabilities');
hold off;
end

function localPlotAblationStudy(results, algorithms, scenarios)
%LOCALPLOTABLATIONSTUDY Plot ablation study showing progressive improvements.

numAlgs = numel(algorithms);
numScen = numel(scenarios);

% Extract mean position RMSE for each algorithm and scenario
rmseMatrix = zeros(numAlgs, numScen);

for a = 1:numAlgs
    algName = algorithms(a).Name;
    for s = 1:numScen
        scenName = scenarios{s};
        if isfield(results, scenName) && isfield(results.(scenName), algName)
            rmseMatrix(a, s) = results.(scenName).(algName).posRMSE_mean;
        end
    end
end

% Plot grouped bar chart
bar(rmseMatrix');
set(gca, 'XTickLabel', scenarios);
xlabel('Scenario');
ylabel('Mean Position RMSE (m)');
title('Ablation Study: Performance Progression');
legend(cellfun(@(x) strrep(x, '_', '-'), {algorithms.Name}, 'UniformOutput', false), ...
    'Location', 'best');
grid on;
end
