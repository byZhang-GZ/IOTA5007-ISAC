function helperPlotSpeedEstimationResults(t, trajectories, targetStateEstimates, trackIDs)
    fig = figure;
    tl = tiledlayout(fig, 1, 2);
    speedAxes = nexttile(tl);
    hold(speedAxes, 'on');
    
    xlabel(speedAxes, 'Time (s)');
    ylabel(speedAxes, 'Speed (m/s)');
    grid(speedAxes, 'on');
    title(speedAxes, 'Speed');
    xlim(speedAxes, [t(1) t(end)]);
    lgd = legend(speedAxes, 'Location', 'southeast', 'Orientation', 'horizontal', 'NumColumns', 2);
    lgd.Layout.Tile = 'south';
    
    headingAxes = nexttile(tl);
    hold(headingAxes, 'on');
    xlabel(headingAxes, 'Time (s)');
    ylabel(headingAxes, 'Heading (deg)');
    grid(headingAxes, 'on');
    title(headingAxes, 'Heading');
    ylim(headingAxes, [-90 90]);    
    xlim(headingAxes, [t(1) t(end)]);

    sgtitle(fig, 'Targets Speed and Heading Esimates')
    
    for it = 1:numel(trackIDs)
        id = trackIDs(it);
        vx = targetStateEstimates(2, :, id);
        vy = targetStateEstimates(4, :, id);
    
        speedEstimate = sqrt(vx.^2 + vy.^2);
        headingEstimate = atan2d(vy, vx);
        plot(speedAxes, t, speedEstimate, 's--', 'LineWidth', 2, 'DisplayName', sprintf('Track %d estimate', trackIDs(it)));
        plot(headingAxes, t, headingEstimate, 's--', 'LineWidth', 2);
    end
    
    for it = 1:numel(trajectories)
        [~, ~, velocity] = lookupPose(trajectories{it}, t);
        trueSpeed = sqrt(velocity(:, 1).^2 + velocity(:, 2).^2);
        trueHeading = atan2d(velocity(:, 2), velocity(:, 1));
        
        plot(speedAxes, t, trueSpeed, '-', 'LineWidth', 2, 'DisplayName', sprintf('Target %d truth', it));
        plot(headingAxes, t, trueHeading, '-', 'LineWidth', 2);
    end
end