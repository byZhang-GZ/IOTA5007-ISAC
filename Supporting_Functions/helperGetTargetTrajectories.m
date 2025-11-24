function trajectories = helperGetTargetTrajectories()

    % Target #1 - 包含转弯的机动轨迹
    % 从起点直行，然后转弯
    waypoints = [25 -5 0;   % 起点
                 40 30 0;   % 中间点（直线段）
                 60 40 0];  % 终点（转弯段）
    timeOfArrival = [0 2.5 4];
    traj1 = waypointTrajectory(waypoints, timeOfArrival, 'Course', [0; 0; -30]);

    % Target #2 - 包含转弯的机动轨迹（相反方向）
    % 从起点直行，然后反方向转弯
    waypoints = [70 30 0;   % 起点
                 80 10 0;   % 中间点
                 70 -5 0];  % 终点（转弯）

    timeOfArrival = [0 2.0 4];
    traj2 = waypointTrajectory(waypoints, timeOfArrival, 'Course', [0; 0; 30]);
    
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