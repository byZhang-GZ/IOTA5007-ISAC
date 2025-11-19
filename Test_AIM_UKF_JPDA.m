%% AIM-UKF-JPDA Quick Validation Test
% This script performs a quick validation of the AIM-UKF-JPDA implementation
% without running the full Monte Carlo experiments.

clear; close all; clc;

fprintf('\n========================================\n');
fprintf('AIM-UKF-JPDA Quick Validation Test\n');
fprintf('========================================\n\n');

%% Test 1: helperInitIMM with Different Configurations
fprintf('[Test 1] Testing helperInitIMM with different configurations...\n');

% Create a dummy detection
detection = objectDetection(0, [100; 45; -5], ...  % [range; azimuth; rangeRate]
    'MeasurementNoise', diag([25, deg2rad(1)^2, 1]));

txPos = [0; 0; 0];
rxPos = [100; 0; 0];
rxOrient = eye(3);

% Test 1a: UKF with CV/CT/CA
fprintf('  1a. UKF with CV/CT/CA models... ');
config1 = struct();
config1.FilterType = 'UKF';
config1.MotionModels = {'CV', 'CT', 'CA'};
try
    filter1 = helperInitIMM(detection, txPos, rxPos, rxOrient, rxOrient, config1);
    fprintf('PASS (IMM with %d models)\n', numel(filter1.Filters));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

% Test 1b: EKF with CV/CT
fprintf('  1b. EKF with CV/CT models... ');
config2 = struct();
config2.FilterType = 'EKF';
config2.MotionModels = {'CV', 'CT'};
try
    filter2 = helperInitIMM(detection, txPos, rxPos, rxOrient, rxOrient, config2);
    fprintf('PASS (IMM with %d models)\n', numel(filter2.Filters));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

%% Test 2: helperConfigureTracker with GNN and JPDA
fprintf('\n[Test 2] Testing helperConfigureTracker...\n');

% Test 2a: GNN Tracker
fprintf('  2a. GNN Tracker... ');
config3 = struct();
config3.TrackerType = 'GNN';
config3.FilterType = 'UKF';
config3.MotionModels = {'CV', 'CT'};
try
    tracker1 = helperConfigureTracker(txPos, rxPos, rxOrient, rxOrient, 1.5, 1.0, config3);
    fprintf('PASS (%s)\n', class(tracker1));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

% Test 2b: JPDA Tracker
fprintf('  2b. JPDA Tracker... ');
config4 = struct();
config4.TrackerType = 'JPDA';
config4.FilterType = 'UKF';
config4.MotionModels = {'CV', 'CT', 'CA'};
try
    tracker2 = helperConfigureTracker(txPos, rxPos, rxOrient, rxOrient, 1.5, 1.0, config4);
    fprintf('PASS (%s)\n', class(tracker2));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

%% Test 3: helperGetTargetTrajectories with Different Scenarios
fprintf('\n[Test 3] Testing helperGetTargetTrajectories...\n');

scenarios = {'HighManeuver', 'Acceleration', 'Crossing'};
for s = 1:numel(scenarios)
    fprintf('  3%c. Scenario: %s... ', 'a' + s - 1, scenarios{s});
    try
        traj = helperGetTargetTrajectories(scenarios{s});
        fprintf('PASS (%d trajectories)\n', numel(traj));
    catch ME
        fprintf('FAIL: %s\n', ME.message);
    end
end

%% Test 4: Measurement Functions with CA State
fprintf('\n[Test 4] Testing measurement functions with CA state...\n');

% Test 4a: CV state (4-D)
fprintf('  4a. CV state (4-D)... ');
try
    stateCV = [50; 10; 30; 5];
    z = isacBistaticMeasurementFcn(stateCV, txPos, rxPos, rxOrient);
    H = isacBistaticMeasurementJacobianFcn(stateCV, txPos, rxPos, rxOrient);
    fprintf('PASS (z: %dx1, H: %dx%d)\n', numel(z), size(H,1), size(H,2));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

% Test 4b: CT state (5-D)
fprintf('  4b. CT state (5-D)... ');
try
    stateCT = [50; 10; 30; 5; 0.1];
    z = isacBistaticMeasurementFcn(stateCT, txPos, rxPos, rxOrient);
    H = isacBistaticMeasurementJacobianFcn(stateCT, txPos, rxPos, rxOrient);
    fprintf('PASS (z: %dx1, H: %dx%d)\n', numel(z), size(H,1), size(H,2));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

% Test 4c: CA state (6-D)
fprintf('  4c. CA state (6-D)... ');
try
    stateCA = [50; 10; 2; 30; 5; -1];
    z = isacBistaticMeasurementFcn(stateCA, txPos, rxPos, rxOrient);
    H = isacBistaticMeasurementJacobianFcn(stateCA, txPos, rxPos, rxOrient);
    fprintf('PASS (z: %dx1, H: %dx%d)\n', numel(z), size(H,1), size(H,2));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

%% Test 5: Process Noise Matrices
fprintf('\n[Test 5] Testing helperProcessNoiseMatrices...\n');

fprintf('  5a. Default noise intensity... ');
try
    Q = helperProcessNoiseMatrices(0.4, struct());
    fprintf('PASS (CV: %dx%d, CT: %dx%d, CA: %dx%d)\n', ...
        size(Q.CV,1), size(Q.CV,2), size(Q.CT,1), size(Q.CT,2), size(Q.CA,1), size(Q.CA,2));
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

fprintf('  5b. Custom noise intensity... ');
try
    noiseConfig = struct('CV', 10, 'CA', 2, 'CTTurnRate', deg2rad(10));
    Q = helperProcessNoiseMatrices(0.4, noiseConfig);
    fprintf('PASS\n');
catch ME
    fprintf('FAIL: %s\n', ME.message);
end

%% Summary
fprintf('\n========================================\n');
fprintf('Validation Tests Completed!\n');
fprintf('========================================\n');
fprintf('\nNext steps:\n');
fprintf('  1. Run ISAC_Scenario.m to test full simulation\n');
fprintf('  2. Run AIM_UKF_JPDA_Experiment.m for ablation study\n');
fprintf('\n');
