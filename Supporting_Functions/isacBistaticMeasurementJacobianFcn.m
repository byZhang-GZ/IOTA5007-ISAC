function H = isacBistaticMeasurementJacobianFcn(state, txPos, rxPos, rxOrientationAxes)
%ISACBISTATICMEASUREMENTJACOBIANFCN Jacobian of the bistatic measurement model.
%   Supports CV, CT, and CA state parameterisations.

[x, vx, y, vy, indexMap] = localExtractStateDefinition(state);
n = numel(state);
H = zeros(3, n);

dx_tx = x - txPos(1);
dy_tx = y - txPos(2);
dx_rx = x - rxPos(1);
dy_rx = y - rxPos(2);

rangeTx = hypot(dx_tx, dy_tx);
rangeRx = hypot(dx_rx, dy_rx);

rangeTx = max(rangeTx, 1e-6);
rangeRx = max(rangeRx, 1e-6);

% Range derivatives
H(1, indexMap.x) = dx_tx / rangeTx + dx_rx / rangeRx;
H(1, indexMap.y) = dy_tx / rangeTx + dy_rx / rangeRx;

% AoA derivatives (convert through receiver local frame)
posRelRx = [dx_rx; dy_rx; 0];
posLocal = global2localcoord(posRelRx, 'rs', [0; 0; 0], rxOrientationAxes.');
xl = posLocal(1);
yl = posLocal(2);
rhoLocal = max(sqrt(xl^2 + yl^2), 1e-6);

daoa_dxl = -yl / (rhoLocal^2) * (180/pi);
daoa_dyl = xl / (rhoLocal^2) * (180/pi);
Rinv = rxOrientationAxes.';

H(2, indexMap.x) = daoa_dxl * Rinv(1, 1) + daoa_dyl * Rinv(2, 1);
H(2, indexMap.y) = daoa_dxl * Rinv(1, 2) + daoa_dyl * Rinv(2, 2);

% Range-rate derivatives
vxFactorTx = dx_tx / rangeTx;
vyFactorTx = dy_tx / rangeTx;
vxFactorRx = dx_rx / rangeRx;
vyFactorRx = dy_rx / rangeRx;

dotTx = vx * dx_tx + vy * dy_tx;
dotRx = vx * dx_rx + vy * dy_rx;

H(3, indexMap.x) = (vx / rangeTx - dotTx * dx_tx / rangeTx^3) + (vx / rangeRx - dotRx * dx_rx / rangeRx^3);
H(3, indexMap.vx) = vxFactorTx + vxFactorRx;
H(3, indexMap.y) = (vy / rangeTx - dotTx * dy_tx / rangeTx^3) + (vy / rangeRx - dotRx * dy_rx / rangeRx^3);
H(3, indexMap.vy) = vyFactorTx + vyFactorRx;

% Fill unused columns with zeros explicitly when present.
unusedFields = setdiff(fieldnames(indexMap), {'x', 'vx', 'y', 'vy'});
for k = 1:numel(unusedFields)
    H(:, indexMap.(unusedFields{k})) = 0;
end
end

function [x, vx, y, vy, indexMap] = localExtractStateDefinition(state)
n = numel(state);
switch n
    case 4 % CV
        x = state(1);
        vx = state(2);
        y = state(3);
        vy = state(4);
        indexMap = struct('x', 1, 'vx', 2, 'y', 3, 'vy', 4);
    case 5 % CT
        x = state(1);
        vx = state(2);
        y = state(3);
        vy = state(4);
        indexMap = struct('x', 1, 'vx', 2, 'y', 3, 'vy', 4, 'omega', 5);
    case 6 % CA
        x = state(1);
        vx = state(2);
        y = state(4);
        vy = state(5);
        indexMap = struct('x', 1, 'vx', 2, 'ax', 3, 'y', 4, 'vy', 5, 'ay', 6);
    otherwise
        error('isacBistaticMeasurementJacobianFcn:UnsupportedStateLength', ...
            'Unsupported state length %d.', n);
end
end
