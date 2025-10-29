function helperVisualizeResourceGrid(G)
    
    %   Copyright 2025 The MathWorks, Inc.

    x = 0:(size(G, 2)-1);
    y = 0:(size(G, 1)-1);

    fig = figure;
    ax = axes(fig);
    imagesc(ax, x-0.5, y-0.5, abs(G));
    grid(ax, 'on'); 
    ax.GridColor=[1 1 1];
    ax.YDir = 'normal';

    ax.XTick = x-1;
    ax.YTick = y-1;

    ax.XTickLabel = compose('%d', x);
    ax.YTickLabel = compose('%d', y);

    xlabel(ax, 'OFDM Symbols');
    ylabel(ax, 'Subcarriers');
end