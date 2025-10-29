function trajectories = helperGetTargetTrajectories()

    % Target #1
    waypoints = [25 -5 0;
                 40 30 0];
    timeOfArrival = [0 4];
    traj1 = waypointTrajectory(waypoints,timeOfArrival);

    % Target #2
    waypoints = [70 30 0;
                 80 10 0];

    timeOfArrival = [0 4];
    traj2 = waypointTrajectory(waypoints,timeOfArrival);
    
    trajectories = {traj1, traj2};

    Trajectory = strings(numel(trajectories), 1);
    Speed = zeros(numel(trajectories), 1);
    Length = zeros(numel(trajectories), 1);
    Start = cell(numel(trajectories), 1);
    Stop = cell(numel(trajectories), 1);

    for i = 1:numel(trajectories)
        Trajectory(i) = i;
        Start{i} = trajectories{i}.Waypoints(1, 1:2);
        Stop{i} = trajectories{i}.Waypoints(end, 1:2);
        Length(i) = vecnorm(trajectories{i}.Waypoints(1, :) - trajectories{i}.Waypoints(end, :));
        Speed(i) = max(trajectories{i}.GroundSpeed);
    end

    table(Trajectory, Start, Stop, Length, Speed, 'VariableNames',...
        {'Trajectory', 'Start Position [x, y]', 'End Position [x, y]', 'Length (m)', 'Speed (m/s)'})
end