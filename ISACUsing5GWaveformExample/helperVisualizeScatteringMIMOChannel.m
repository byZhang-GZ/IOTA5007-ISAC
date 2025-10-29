function helperVisualizeScatteringMIMOChannel(scatteringMIMOChannel, scattererPositions, targetTrajectories)

    %   Copyright 2025 The MathWorks, Inc.
    
    figure;
    hold on;

    ax = gca;
    colors = ax.ColorOrder;

    xspan = [min(scattererPositions(1, :)) max(scattererPositions(1, :))];
    if ~isempty(xspan)
        dx = (xspan(2) - xspan(1))/10;
    else
        dx = 5;
    end

    yspan = [min(scattererPositions(2, :)) max(scattererPositions(2, :))];
    if ~isempty(yspan)
        dy = (yspan(2) - yspan(1))/10;
    else
        dy = 5;
    end
    
    txPosition = scatteringMIMOChannel.TransmitArrayPosition;
    txOrientationAxis = scatteringMIMOChannel.TransmitArrayOrientationAxes;
    plot(txPosition(1), txPosition(2), 'o', 'Color', colors(5, :), 'MarkerSize', 12, 'MarkerFaceColor', colors(5, :), 'LineWidth', 1.5, 'DisplayName', 'gNB');
    quiver(txPosition(1), txPosition(2), dx*txOrientationAxis(1,1), dx*txOrientationAxis(2, 1),...
        'Color', colors(5, :), 'LineWidth', 1.5, 'DisplayName', 'gNB array normal', 'MaxHeadSize', 1);

    rxPosition = scatteringMIMOChannel.ReceiveArrayPosition;
    rxOrientationAxis = scatteringMIMOChannel.ReceiveArrayOrientationAxes;    
    plot(rxPosition(1), rxPosition(2), 'v', 'Color', colors(3, :), 'MarkerSize', 10, 'MarkerFaceColor', colors(3, :), 'LineWidth', 1.5, 'DisplayName', 'UE');
    quiver(rxPosition(1), rxPosition(2), dy*rxOrientationAxis(1,1), dy*rxOrientationAxis(2, 1),...
        'Color', colors(3, :), 'LineWidth', 1.5, 'DisplayName', 'UE array normal', 'MaxHeadSize', 1);
    
    plot(scattererPositions(1, :), scattererPositions(2, :), '.', 'Color', colors(1, :), 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Static scatterers');

    T = targetTrajectories{1}.TimeOfArrival(end);
    t = linspace(0,T);
    numFrames = numel(t);
    for it = 1:numel(targetTrajectories)
        traj = clone(targetTrajectories{it});
        reset(traj);
        targetPositions = zeros(3, numFrames);
        for j = 1:numFrames
            targetPositions(:, j) = lookupPose(traj, t(j));
        end

        if it == 1
            plot(targetPositions(1, 1), targetPositions(2, 1), 'x', 'Color', colors(2, :), 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', ['Moving scatterers' newline '(Targets)']);
        else
            plot(targetPositions(1, 1), targetPositions(2, 1), 'x', 'Color', colors(2, :), 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
        end

        text(targetPositions(1, 1)+3, targetPositions(2, 1)+3, num2str(it));

        plot(targetPositions(1, :), targetPositions(2, :), '--', 'Color', colors(2, :), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    
    grid on;
    xlabel('x (m)');
    ylabel('y (m)');
    legend('Location', 'eastoutside', 'Orientation', 'horizontal', 'NumColumns', 1);
    % xlim([-25 25]);
    axis equal;
    box on;
end