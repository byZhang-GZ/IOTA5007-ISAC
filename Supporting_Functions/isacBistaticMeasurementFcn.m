function z = isacBistaticMeasurementFcn(state, txPos, rxPos, rxOrientationAxes)
%ISACBISTATICMEASUREMENTFCN Nonlinear bistatic radar measurement model.
%   Supports CV (4 states), CT (5 states), and CA (6 states) motion models.

[x, vx, y, vy] = localExtractPositionVelocity(state);

% Bistatic range as the sum of transmitter/receiver distances minus baseline.
rangeTx = hypot(x - txPos(1), y - txPos(2));
rangeRx = hypot(x - rxPos(1), y - rxPos(2));
baseline = vecnorm(txPos - rxPos);
bistaticRange = rangeTx + rangeRx - baseline;

% Arrival angle computed in the receiver's local frame.
posRelRx = [x - rxPos(1); y - rxPos(2); 0];
posLocal = global2localcoord(posRelRx, 'rs', [0; 0; 0], rxOrientationAxes.');
[aoa, ~, ~] = cart2sph(posLocal(1), posLocal(2), posLocal(3));
aoa = rad2deg(aoa);

% Bistatic range rate combines radial components toward TX and RX.
if rangeTx > 1e-6
    vTx = (vx * (x - txPos(1)) + vy * (y - txPos(2))) / rangeTx;
else
    vTx = 0;
end

if rangeRx > 1e-6
    vRx = (vx * (x - rxPos(1)) + vy * (y - rxPos(2))) / rangeRx;
else
    vRx = 0;
end

bistaticRangeRate = vTx + vRx;

z = [bistaticRange; aoa; bistaticRangeRate];
end

function [x, vx, y, vy] = localExtractPositionVelocity(state)
n = numel(state);
switch n
    case {4, 5}
        x = state(1);
        vx = state(2);
        y = state(3);
        vy = state(4);
    case 6
        x = state(1);
        vx = state(2);
        y = state(4);
        vy = state(5);
    otherwise
        error('isacBistaticMeasurementFcn:UnsupportedStateLength', ...
            'Unsupported state length %d.', n);
end
end
