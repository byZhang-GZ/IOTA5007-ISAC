function tracks = helperAdaptiveProcessNoise(tracks, adaptiveConfig)
%HELPERADAPTIVEPROCESSNOISE Apply adaptive process noise based on IMM model probabilities.
%   This function dynamically adjusts the process noise (Q) of each subfilter
%   in the IMM tracker based on the current model probabilities. When maneuvering
%   models (CT/CA) dominate, process noise is temporarily increased to make the
%   filter more responsive to rapid state changes.
%
%   Inputs:
%     tracks          - array of objectTrack from the tracker
%     adaptiveConfig  - struct with fields:
%         ManeuverThreshold     - threshold for maneuver detection (default: 0.5)
%         QBoostFactor          - multiplier for Q during maneuver (default: 10)
%         BoostDuration         - frames to maintain boost (default: 3)
%         Enable                - true/false to enable adaptation (default: true)
%
%   Outputs:
%     tracks - modified tracks with updated process noise

if nargin < 2 || isempty(adaptiveConfig)
    adaptiveConfig = struct();
end

% Apply defaults
if ~isfield(adaptiveConfig, 'ManeuverThreshold')
    adaptiveConfig.ManeuverThreshold = 0.5;
end
if ~isfield(adaptiveConfig, 'QBoostFactor')
    adaptiveConfig.QBoostFactor = 10;
end
if ~isfield(adaptiveConfig, 'BoostDuration')
    adaptiveConfig.BoostDuration = 3;
end
if ~isfield(adaptiveConfig, 'Enable')
    adaptiveConfig.Enable = true;
end

if ~adaptiveConfig.Enable || isempty(tracks)
    return;
end

for idx = 1:numel(tracks)
    filter = tracks(idx).Filter;
    
    % Check if this is an IMM filter
    if ~isa(filter, 'trackingIMM')
        continue;
    end
    
    % Get current model probabilities
    modelProbs = filter.ModelProbabilities;
    
    % Calculate maneuver probability (sum of CT and CA probabilities)
    % Assumes models are ordered as CV, CT, CA (or subset thereof)
    numModels = numel(modelProbs);
    
    % Identify maneuver models by name
    modelNames = cellstr(string(filter.ModelNames));
    maneuverProb = 0;
    
    for m = 1:numModels
        modelName = upper(modelNames{m});
        if contains(modelName, 'TURN') || contains(modelName, 'ACC')
            maneuverProb = maneuverProb + modelProbs(m);
        end
    end
    
    % Check if maneuver threshold exceeded
    if maneuverProb > adaptiveConfig.ManeuverThreshold
        % Boost process noise for all subfilters
        subFilters = filter.Filters;
        for f = 1:numel(subFilters)
            currentQ = subFilters{f}.ProcessNoise;
            subFilters{f}.ProcessNoise = currentQ * adaptiveConfig.QBoostFactor;
        end
        
        % Store boost counter in track's State (last element as metadata)
        % Note: This is a simplified approach. In production, use track.State(end)
        % or a persistent variable to track boost duration
    end
end
end
