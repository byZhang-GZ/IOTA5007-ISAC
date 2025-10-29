classdef helperCFARResultsVisualizer < handle

    %   Copyright 2025 The MathWorks, Inc.

    properties(Access = private)
        Axes
        Image (1, 1) matlab.graphics.primitive.Image
        Truth (1, 1) matlab.graphics.chart.primitive.Line
        Detections (1, 1) matlab.graphics.chart.primitive.Line
        ClusteredDetections (1, 1) matlab.graphics.chart.primitive.Line
        FrameBuffer
    end

    methods
        function obj = helperCFARResultsVisualizer(tl)
            obj.Axes = nexttile(tl);
            colors = obj.Axes.ColorOrder;

            hold(obj.Axes, "on");
            obj.Image = imagesc(obj.Axes, NaN, 'HandleVisibility', 'off');
            obj.Truth = plot(obj.Axes, NaN, NaN, 'Color', colors(2, :),...
                'Marker', 'x', 'LineStyle', 'none', 'DisplayName', 'True positions', 'LineWidth', 2);
            obj.Detections = plot(obj.Axes, NaN, NaN, 'Marker', '.', 'LineStyle', 'none', 'DisplayName', 'CFAR detections');
            obj.ClusteredDetections = plot(obj.Axes, NaN, NaN, 'Color', colors(4, :),...
                'Marker', 'o', 'LineStyle', 'none', 'DisplayName', 'Clustered detections', 'LineWidth', 2);

            obj.Axes.YDir = 'normal';
            colormap(obj.Axes, 'gray');

            obj.FrameBuffer = {};
        end

        function plotCFARImage(obj, x, y, cfarImage)
            obj.Image.XData = x;
            obj.Image.YData = y;
            obj.Image.CData = cfarImage;

            xlim(obj.Axes, [min(x) max(x)]);
            ylim(obj.Axes, [min(y) max(y)]);
        end

        function plotTruth(obj, truth)
            obj.Truth.XData = truth(2, :);
            obj.Truth.YData = truth(1, :);
        end

        function plotDetections(obj, detections)
            obj.Detections.XData = detections(2, :);
            obj.Detections.YData = detections(1, :);
        end

        function plotClusteredDetections(obj, clusteredDetections)
            obj.ClusteredDetections.XData = clusteredDetections(2, :);
            obj.ClusteredDetections.YData = clusteredDetections(1, :);
        end

        function addFrameToBuffer(obj)
           tl = obj.Axes.Parent;
           fig = tl.Parent;
           obj.FrameBuffer{end+1} = frame2im(getframe(fig));
        end

        function xlabel(obj, str)
            xlabel(obj.Axes, str);
        end

        function ylabel(obj, str)
            ylabel(obj.Axes, str);
        end        

        function title(obj, str)
            title(obj.Axes, str);
        end              
    end
end