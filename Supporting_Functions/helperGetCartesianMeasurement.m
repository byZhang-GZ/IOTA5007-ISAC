function meascart = helperGetCartesianMeasurement(dets, txPosition, rxPosition, rxOrientationAxes)
    n = size(dets, 2);
    meassph = zeros(3, n);
    meassph(1, :) = dets(2, :);
    meassph(3, :) = dets(1, :);

    meassph = local2globalcoord(meassph, 'ss', [0; 0; 0], rxOrientationAxes);    
    meascart = bistaticposest(meassph(3, :), meassph(1:2, :), eps*ones(1, n), repmat([eps; eps], 1, n),...
        txPosition, rxPosition, 'RangeMeasurement', 'BistaticRange');
end