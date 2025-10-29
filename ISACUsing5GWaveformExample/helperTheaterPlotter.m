classdef helperTheaterPlotter

    %   Copyright 2025 The MathWorks, Inc.

    properties (Access = private)
        Axes
        TheaterPlot
        TrajectoryPlotter
        TargetPlotter
        DetectionPlotter
        ScattererPlotter
        TrackPlotter
        TxPlotter
        RxPlotter
        FrameBuffer
    end

    methods
        function obj = helperTheaterPlotter(tl)
            obj.Axes = nexttile(tl);
            colors = obj.Axes.ColorOrder;
            
            obj.Axes.ZDir = 'reverse';
            obj.TheaterPlot = theaterPlot('Parent', obj.Axes, 'AxesUnits', ["m","m","m"], ...
                'XLimits', [0 120], 'YLimits', [-80 80], 'ZLimits', [-1 1]);

            obj.TxPlotter = platformPlotter(obj.TheaterPlot, 'DisplayName', 'gNB', 'Marker',  'o',...
                            'MarkerFaceColor', colors(5, :), 'MarkerEdgeColor', colors(5, :), 'MarkerSize', 6);
          
            obj.RxPlotter = platformPlotter(obj.TheaterPlot, 'DisplayName', 'UE', 'Marker',  'v',...
                            'MarkerFaceColor', colors(3, :), 'MarkerEdgeColor', colors(3, :), 'MarkerSize', 6);

            obj.ScattererPlotter = platformPlotter(obj.TheaterPlot, 'DisplayName', 'Scatterers', 'Marker',  '.',...
                'MarkerFaceColor', colors(1, :), 'MarkerEdgeColor', colors(1, :));            
            
            obj.TargetPlotter = platformPlotter(obj.TheaterPlot, 'DisplayName', 'Targets', 'Marker',  'x',...
                            'MarkerFaceColor', colors(2, :), 'MarkerEdgeColor', colors(2, :), 'MarkerSize', 6);

            obj.TrajectoryPlotter = trajectoryPlotter(obj.TheaterPlot, 'DisplayName', 'Trajectories', 'LineWidth', 1, 'Color',  colors(2, :));            

            obj.DetectionPlotter = detectionPlotter(obj.TheaterPlot, 'DisplayName', 'Detections', 'Marker',  'o',...
                           'MarkerEdgeColor',  colors(4, :), 'MarkerSize', 5, 'HistoryDepth', 20);
        
            obj.TrackPlotter = trackPlotter(obj.TheaterPlot, 'DisplayName', 'Tracks', 'ConnectHistory', 'on', 'ColorizeHistory', 'on');

            lgd = legend(obj.Axes, 'NumColumns', 4, 'Orientation', 'horizontal');
            lgd.Layout.Tile = 'south';

            obj.FrameBuffer = {};
        end

        function plotTrajectories(obj, trajectories, t)
            n = numel(trajectories);
            trajectoryPositions = cell(1, n);

            for i = 1:n
                trajectoryPositions{i} = lookupPose(trajectories{i}, t);
            end
            plotTrajectory(obj.TrajectoryPlotter, trajectoryPositions);
        end

        function plotDetections(obj, meas)
            if ~isempty(meas)   
                obj.DetectionPlotter.plotDetection(meas.');
            end
        end

        function plotTracks(obj, tracks)
            n = numel(tracks);
            trackPositions = zeros(n, 3);
            trackIDs = zeros(n, 1);
            for i = 1:n
                trackPositions(i, 1) = tracks(i).State(1);
                trackPositions(i, 2) = tracks(i).State(3);
                trackIDs(i) = tracks(i).TrackID;
            end

            if ~isempty(trackPositions) 
                obj.TrackPlotter.plotTrack(trackPositions, trackIDs);
            end
        end

        function plotTargetPositions(obj, positions)
            labels = compose('%d',1:size(positions,1));
            obj.TargetPlotter.plotPlatform(positions, labels);
        end

        function plotScatterers(obj, positions)
            obj.ScattererPlotter.plotPlatform(positions.');
        end    

        function plotTxAndRx(obj, txPos, rxPos)
            obj.TxPlotter.plotPlatform(txPos.');
            obj.RxPlotter.plotPlatform(rxPos.');
        end

        function title(obj, str)
            title(obj.Axes, str);
        end
    end
end
