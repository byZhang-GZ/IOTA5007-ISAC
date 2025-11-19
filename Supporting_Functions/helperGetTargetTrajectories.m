function trajectories = helperGetTargetTrajectories(scenarioType)
%HELPERGETTARGETTRAJECTORIES Generate target trajectories for different test scenarios.
%   Supports three scenarios designed to validate AIM-UKF-JPDA improvements:
%     'HighManeuver'   - High-speed turning (tests UKF vs EKF, CV/CT/CA)
%     'Acceleration'   - Acceleration/deceleration (tests CA model, adaptive Q)
%     'Crossing'       - Target crossing paths (tests JPDA vs GNN)
%
%   Syntax:
%     trajectories = helperGetTargetTrajectories()
%     trajectories = helperGetTargetTrajectories(scenarioType)

if nargin < 1 || isempty(scenarioType)
    scenarioType = 'HighManeuver';
end

scenarioType = char(scenarioType);

switch scenarioType
    case 'HighManeuver'
        % Scenario 1: High maneuvering turns (original scenario)
        % Target #1 - turning trajectory
        waypoints = [25 -5 0; 40 30 0; 60 40 0];
        timeOfArrival = [0 2.5 4];
        traj1 = waypointTrajectory(waypoints, timeOfArrival, 'Course', [0; 0; -30]);
        
        % Target #2 - opposite turning trajectory
        waypoints = [70 30 0; 80 10 0; 70 -5 0];
        timeOfArrival = [0 2.0 4];
        traj2 = waypointTrajectory(waypoints, timeOfArrival, 'Course', [0; 0; 30]);
        
        trajectories = {traj1, traj2};
        
    case 'Acceleration'
        % Scenario 2: Acceleration and deceleration maneuvers
        % Target #1 - accelerate then decelerate
        waypoints = [20 0 0;    % Start: stopped
                     40 0 0;     % Point 1: accelerating
                     70 0 0;     % Point 2: cruising
                     80 0 0];    % End: decelerating
        % Time intervals designed to create speed changes
        timeOfArrival = [0 3 5 8];  % Acceleration phase: 0-3s, Cruise: 3-5s, Deceleration: 5-8s
        traj1 = waypointTrajectory(waypoints, timeOfArrival);
        
        % Target #2 - constant speed for comparison
        waypoints = [20 20 0; 80 20 0];
        timeOfArrival = [0 6];
        traj2 = waypointTrajectory(waypoints, timeOfArrival);
        
        trajectories = {traj1, traj2};
        
    case 'Crossing'
        % Scenario 3: Crossing targets with close proximity
        % Target #1 - moving from left to right
        waypoints = [10 10 0; 50 30 0; 90 10 0];
        timeOfArrival = [0 2 4];
        traj1 = waypointTrajectory(waypoints, timeOfArrival);
        
        % Target #2 - moving from right to left (crossing path)
        waypoints = [90 30 0; 50 10 0; 10 30 0];
        timeOfArrival = [0 2 4];
        traj2 = waypointTrajectory(waypoints, timeOfArrival);
        
        trajectories = {traj1, traj2};
        
    otherwise
        error('helperGetTargetTrajectories:InvalidScenario', ...
            'Unknown scenario type: %s', scenarioType);
end

% Display trajectory information
displayTrajectoryInfo(trajectories, scenarioType);
end

function displayTrajectoryInfo(trajectories, scenarioType)
%DISPLAYTRAJECTORYINFO Helper to display trajectory summary table.

Trajectory = strings(numel(trajectories), 1);
Speed = zeros(numel(trajectories), 1);
Length = zeros(numel(trajectories), 1);
Start = cell(numel(trajectories), 1);
Stop = cell(numel(trajectories), 1);

for i = 1:numel(trajectories)
    Trajectory(i) = sprintf("Target %d", i);
    Start{i} = trajectories{i}.Waypoints(1, 1:2);
    Stop{i} = trajectories{i}.Waypoints(end, 1:2);
    Length(i) = vecnorm(trajectories{i}.Waypoints(1, :) - trajectories{i}.Waypoints(end, :));
    Speed(i) = max(trajectories{i}.GroundSpeed);
end

fprintf('\n=== Scenario: %s ===\n', scenarioType);
disp(table(Trajectory, Start, Stop, Length, Speed, 'VariableNames', ...
    {'Trajectory', 'Start [x,y]', 'End [x,y]', 'Length(m)', 'MaxSpeed(m/s)'}));
end