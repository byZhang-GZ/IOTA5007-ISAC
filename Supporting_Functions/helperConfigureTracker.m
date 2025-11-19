function tracker = helperConfigureTracker(txPosition, rxPosition, txOrientation, rxOrientation, rangeResolution, aoaResolution, algorithmConfig)
%HELPERCONFIGURETRACKER Configure multi-target tracker with flexible algorithm options.
%   Supports both trackerGNN and trackerJPDA with configurable IMM settings.
%
%   algorithmConfig is an optional struct with fields:
%     TrackerType          - "GNN" (default) or "JPDA"
%     FilterType           - "UKF" or "EKF"
%     MotionModels         - cell array {"CV", "CT"} or {"CV", "CT", "CA"}
%     NoiseIntensity       - struct with noise settings
%     TransitionProbabilities, InitialModelProbabilities, etc.

if nargin < 7
    algorithmConfig = struct();
end

% Apply defaults
if ~isfield(algorithmConfig, 'TrackerType') || isempty(algorithmConfig.TrackerType)
    algorithmConfig.TrackerType = "GNN";
end

% Create filter initialization function handle that captures algorithmConfig
initFcn = @(detection) helperInitIMM(detection, txPosition, rxPosition, ...
    txOrientation, rxOrientation, algorithmConfig);

% Select tracker type
trackerType = upper(string(algorithmConfig.TrackerType));

switch trackerType
    case "GNN"
        tracker = trackerGNN('FilterInitializationFcn', initFcn, ...
            'AssignmentThreshold', [200 inf], ...
            'ConfirmationThreshold', [3 5], ...
            'DeletionThreshold', [5 5], ...
            'HasCostMatrixInput', false, ...
            'MaxNumTracks', 10, ...
            'MaxNumSensors', 1);
    case "JPDA"
        tracker = trackerJPDA('FilterInitializationFcn', initFcn, ...
            'AssignmentThreshold', [200 inf], ...
            'ConfirmationThreshold', [3 5], ...
            'DeletionThreshold', [5 5], ...
            'DetectionProbability', 0.9, ...
            'ClutterDensity', 1e-6, ...
            'MaxNumTracks', 10, ...
            'MaxNumSensors', 1);
    otherwise
        error('helperConfigureTracker:UnsupportedTrackerType', ...
            'Unsupported tracker type "%s".', trackerType);
end
end
