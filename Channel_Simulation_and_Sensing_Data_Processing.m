%% Channel Simulation and Sensing Data Processing Wrapper
% This script is kept for backward compatibility and now defers to the
% functional ISAC_Scenario implementation so that the experiment framework
% and ad-hoc visualization share the same simulation core.

clear; close all; clc;

algorithmConfig = struct();
algorithmConfig.FilterType = 'UKF';
algorithmConfig.MotionModels = {'CV', 'CT', 'CA'};
algorithmConfig.TrackerType = 'JPDA';
algorithmConfig.AdaptiveQ = true;

simOptions = struct();
simOptions.GenerateVisualizations = true;
simOptions.SaveTrackingResults = true;
simOptions.ShowProgress = true;
simOptions.Seed = 'default';

scenarioName = 'HighManeuver';
simResult = ISAC_Scenario(algorithmConfig, scenarioName, simOptions); %#ok<NASGU>