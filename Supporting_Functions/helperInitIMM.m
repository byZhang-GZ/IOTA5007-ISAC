function filter = helperInitIMM(detection, txPos, rxPos, txOrientationAxes, rxOrientationAxes, algorithmConfig)
%HELPERINITIMM Initialise an IMM filter for new tracks with configurable models.
%   This helper creates a collection of subfilters (CV/CT/CA) based on the
%   requested motion models and wraps them in a trackingIMM instance.
%
%   detection          objectDetection containing the first measurement
%   txPos, rxPos       transmitter / receiver Cartesian positions
%   txOrientationAxes  orientation axes of the transmitter (unused but kept
%                      for compatibility with existing tooling)
%   rxOrientationAxes  orientation axes of the receiver (3-by-3 matrix)
%   algorithmConfig    optional struct overriding default IMM settings
%
%   The algorithmConfig struct supports the following fields:
%     SampleTime              - scalar sample period (s)
%     FilterType              - "UKF" (default) or "EKF"
%     MotionModels            - cellstr / string array of motion models
%                                (subset of {"CV","CT","CA"})
%     NoiseIntensity          - struct with optional fields per model
%     TransitionProbabilities - Markov transition matrix
%     InitialModelProbabilities - column vector of initial model probs
%
%   Returns a trackingIMM object ready to be used by trackerJPDA or similar.

%#ok<*NASGU> txOrientationAxes is unused but kept for signature compatibility

if nargin < 6
    algorithmConfig = struct();
end
algorithmConfig = localApplyConfigDefaults(algorithmConfig);

filterType = upper(string(algorithmConfig.FilterType));
dt = algorithmConfig.SampleTime;
motionModels = string(algorithmConfig.MotionModels(:));
numModels = numel(motionModels);

% Convert the bistatic measurement into an initial Cartesian estimate.
range = detection.Measurement(1);
aoa = detection.Measurement(2);
meassph = [aoa; 0; range];
meassphGlobal = local2globalcoord(meassph, 'ss', [0; 0; 0], rxOrientationAxes);

% Use the Radar Toolbox helper to triangulate the bistatic Cartesian point.
meascart = bistaticposest(range, meassphGlobal(1:2), eps, [eps; eps], ...
    txPos, rxPos, 'RangeMeasurement', 'BistaticRange');

x0 = meascart(1);
y0 = meascart(2);

if numel(detection.Measurement) >= 3 && ~isempty(detection.Measurement(3))
    rangeRate = detection.Measurement(3);
    dx_rx = x0 - rxPos(1);
    dy_rx = y0 - rxPos(2);
    rangeRx = hypot(dx_rx, dy_rx);
    if rangeRx > 1e-3
        vx0 = 0.5 * rangeRate * dx_rx / rangeRx;
        vy0 = 0.5 * rangeRate * dy_rx / rangeRx;
    else
        vx0 = 0;
        vy0 = 0;
    end
else
    vx0 = 0;
    vy0 = 0;
end

if isempty(detection.MeasurementNoise)
    R = diag([25, deg2rad(1)^2, 1]);
else
    R = detection.MeasurementNoise;
end

Qstruct = helperProcessNoiseMatrices(dt, algorithmConfig.NoiseIntensity);

subFilters = cell(numModels, 1);
modelNames = cell(numModels, 1);

for idx = 1:numModels
    modelName = upper(strtrim(motionModels(idx)));
    switch modelName
        case "CV"
            state = [x0; vx0; y0; vy0];
            P0 = diag([100, 25, 100, 25]);
            processNoise = Qstruct.CV;
            transitionFcn = @(stateVec, dtInput) cvTransition(stateVec, dtInput);
            transitionJac = @(stateVec, dtInput) cvTransitionJacobian(dtInput);
            measurementFcn = @(stateVec) isacBistaticMeasurementFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            measurementJac = @(stateVec) isacBistaticMeasurementJacobianFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            subFilters{idx} = localCreateFilter(filterType, state, P0, processNoise, R, transitionFcn, transitionJac, measurementFcn, measurementJac);
            modelNames{idx} = 'constvel';
        case "CT"
            state = [x0; vx0; y0; vy0; 0];
            P0 = diag([100, 25, 100, 25, deg2rad(30)^2]);
            processNoise = Qstruct.CT;
            transitionFcn = @(stateVec, dtInput) coordinatedTurnTransition(stateVec, dtInput);
            transitionJac = @(stateVec, dtInput) coordinatedTurnJacobian(stateVec, dtInput);
            measurementFcn = @(stateVec) isacBistaticMeasurementFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            measurementJac = @(stateVec) isacBistaticMeasurementJacobianFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            subFilters{idx} = localCreateFilter(filterType, state, P0, processNoise, R, transitionFcn, transitionJac, measurementFcn, measurementJac);
            modelNames{idx} = 'constturn';
        case "CA"
            state = [x0; vx0; 0; y0; vy0; 0];
            P0 = diag([100, 25, 9, 100, 25, 9]);
            processNoise = Qstruct.CA;
            transitionFcn = @(stateVec, dtInput) caTransition(stateVec, dtInput);
            transitionJac = @(stateVec, dtInput) caTransitionJacobian(dtInput);
            measurementFcn = @(stateVec) isacBistaticMeasurementFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            measurementJac = @(stateVec) isacBistaticMeasurementJacobianFcn(stateVec, txPos, rxPos, rxOrientationAxes);
            subFilters{idx} = localCreateFilter(filterType, state, P0, processNoise, R, transitionFcn, transitionJac, measurementFcn, measurementJac);
            modelNames{idx} = 'constacc';
        otherwise
            error('helperInitIMM:UnsupportedMotionModel', ...
                'Unsupported motion model "%s".', modelName);
    end
end

filter = trackingIMM(subFilters, ...
    'ModelNames', modelNames, ...
    'ModelProbabilities', algorithmConfig.InitialModelProbabilities(:), ...
    'TransitionProbabilities', algorithmConfig.TransitionProbabilities);
end

% -------------------------------------------------------------------------
function filter = localCreateFilter(filterType, state, P0, Q, R, transitionFcn, transitionJacFcn, measurementFcn, measurementJacFcn)
switch filterType
    case "UKF"
        filter = trackingUKF('State', state, ...
            'StateCovariance', P0, ...
            'ProcessNoise', Q, ...
            'MeasurementNoise', R, ...
            'StateTransitionFcn', transitionFcn, ...
            'MeasurementFcn', measurementFcn);
    case "EKF"
        filter = trackingEKF('State', state, ...
            'StateCovariance', P0, ...
            'StateTransitionFcn', transitionFcn, ...
            'StateTransitionJacobianFcn', transitionJacFcn, ...
            'ProcessNoise', Q, ...
            'MeasurementFcn', measurementFcn, ...
            'MeasurementJacobianFcn', measurementJacFcn, ...
            'MeasurementNoise', R);
    otherwise
        error('helperInitIMM:UnsupportedFilterType', ...
            'Unsupported filter type "%s".', filterType);
end
end

% -------------------------------------------------------------------------
function config = localApplyConfigDefaults(config)
defaults = localDefaultAlgorithmConfig();

% Apply individual field defaults
if ~isfield(config, 'SampleTime') || isempty(config.SampleTime)
    config.SampleTime = defaults.SampleTime;
end
if ~isfield(config, 'FilterType') || isempty(config.FilterType)
    config.FilterType = defaults.FilterType;
end
if ~isfield(config, 'MotionModels') || isempty(config.MotionModels)
    config.MotionModels = defaults.MotionModels;
end
if ~isfield(config, 'NoiseIntensity') || isempty(config.NoiseIntensity)
    config.NoiseIntensity = defaults.NoiseIntensity;
end

config.MotionModels = cellstr(string(config.MotionModels));
numModels = numel(config.MotionModels);

% Generate transition probabilities based on actual number of models
if ~isfield(config, 'TransitionProbabilities') || isempty(config.TransitionProbabilities)
    config.TransitionProbabilities = eye(numModels) * 0.9 + ...
        (ones(numModels) - eye(numModels)) * (0.1 / max(numModels - 1, 1));
elseif size(config.TransitionProbabilities, 1) ~= numModels
    config.TransitionProbabilities = localNormalizeTransitionMatrix(config.TransitionProbabilities, numModels);
end

if ~isfield(config, 'InitialModelProbabilities') || isempty(config.InitialModelProbabilities)
    config.InitialModelProbabilities = ones(numModels, 1) / numModels;
elseif numel(config.InitialModelProbabilities) ~= numModels
    config.InitialModelProbabilities = ones(numModels, 1) / numModels;
end

if ~isstruct(config.NoiseIntensity)
    config.NoiseIntensity = struct();
end
end

% -------------------------------------------------------------------------
function defaults = localDefaultAlgorithmConfig()
defaults = struct();
defaults.SampleTime = 0.4;
defaults.FilterType = "UKF";
defaults.MotionModels = {"CV", "CT"};
defaults.NoiseIntensity = struct('CV', 5, 'CA', 1, 'CTTurnRate', deg2rad(5));
defaults.TransitionProbabilities = [0.95 0.05; 0.05 0.95];
defaults.InitialModelProbabilities = [0.5; 0.5];
end

% -------------------------------------------------------------------------
function P = localNormalizeTransitionMatrix(P, numModels)
if isempty(P)
    P = eye(numModels) * 0.9 + (ones(numModels) - eye(numModels)) * (0.1 / max(numModels - 1, 1));
    return;
end

P = P(:, 1:min(size(P, 2), numModels));
if size(P, 1) ~= numModels
    P = repmat(P(1, :), numModels, 1);
end

rowSums = sum(P, 2);
rowSums(rowSums == 0) = 1;
P = P ./ rowSums;
end

% -------------------------------------------------------------------------
function newState = cvTransition(state, dt)
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];
newState = F * state;
end

function F = cvTransitionJacobian(dt)
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];
end

function newState = caTransition(state, dt)
newState = state;
newState(1) = state(1) + state(2) * dt + 0.5 * state(3) * dt^2;
newState(2) = state(2) + state(3) * dt;
newState(4) = state(4) + state(5) * dt + 0.5 * state(6) * dt^2;
newState(5) = state(5) + state(6) * dt;
end

function F = caTransitionJacobian(dt)
F = [1 dt 0.5*dt^2 0 0 0;
     0 1  dt      0 0 0;
     0 0  1       0 0 0;
     0 0  0       1 dt 0.5*dt^2;
     0 0  0       0 1  dt;
     0 0  0       0 0  1];
end

function newState = coordinatedTurnTransition(state, dt)
x = state(1);
vx = state(2);
y = state(3);
vy = state(4);
omega = state(5);

if abs(omega) < 1e-6
    newState = [x + vx * dt;
                vx;
                y + vy * dt;
                vy;
                omega];
else
    sinw = sin(omega * dt);
    cosw = cos(omega * dt);
    newState = [
        x + (vx * sinw + vy * (cosw - 1)) / omega;
        vx * cosw - vy * sinw;
        y + (vx * (1 - cosw) + vy * sinw) / omega;
        vx * sinw + vy * cosw;
        omega];
end
end

function J = coordinatedTurnJacobian(state, dt)
vx = state(2);
vy = state(4);
omega = state(5);

if abs(omega) < 1e-6
    J = [1 dt 0 0 0;
         0 1  0 0 0;
         0 0  1 dt 0;
         0 0  0 1  0;
         0 0  0 0  1];
else
    sinw = sin(omega * dt);
    cosw = cos(omega * dt);
    omega2 = omega^2;
    J = zeros(5, 5);
    J(1, 1) = 1;
    J(1, 2) = sinw / omega;
    J(1, 4) = (cosw - 1) / omega;
    J(1, 5) = (vx * (omega * dt * cosw - sinw) + vy * (-omega * dt * sinw - cosw + 1)) / omega2;

    J(2, 2) = cosw;
    J(2, 4) = -sinw;
    J(2, 5) = -dt * (vx * sinw + vy * cosw);

    J(3, 2) = (1 - cosw) / omega;
    J(3, 3) = 1;
    J(3, 4) = sinw / omega;
    J(3, 5) = (vx * (-omega * dt * sinw - cosw + 1) + vy * (omega * dt * cosw - sinw)) / omega2;

    J(4, 2) = sinw;
    J(4, 4) = cosw;
    J(4, 5) = dt * (vx * cosw - vy * sinw);

    J(5, 5) = 1;
end
end
