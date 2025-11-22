fprintf('Current directory: %s\n', pwd);
try
    addpath(genpath(pwd));
    fprintf('Paths added.\n');
catch ME
    fprintf('Failed to add paths: %s\n', ME.message);
end

% try
%     fprintf('Testing ISAC_Scenario...\n');
%     % Run with visualization disabled for speed in test
%     simOptions = struct('GenerateVisualizations', false, 'NumSensingFrames', 5);
%     ISAC_Scenario(struct(), 'HighManeuver', simOptions);
%     fprintf('ISAC_Scenario passed.\n');
% catch ME
%     fprintf('ISAC_Scenario failed: %s\n', ME.message);
%     fprintf('Stack trace:\n');
%     for k = 1:length(ME.stack)
%         fprintf('  %s:%d\n', ME.stack(k).name, ME.stack(k).line);
%     end
% end

try
    fprintf('\nTesting AIM_UKF_JPDA_Experiment...\n');
    AIM_UKF_JPDA_Experiment;
    fprintf('AIM_UKF_JPDA_Experiment passed.\n');
catch ME
    fprintf('AIM_UKF_JPDA_Experiment failed: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s:%d\n', ME.stack(k).name, ME.stack(k).line);
    end
end
