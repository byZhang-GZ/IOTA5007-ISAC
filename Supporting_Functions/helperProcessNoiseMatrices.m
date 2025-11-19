function Q = helperProcessNoiseMatrices(dt, noiseIntensity)
%HELPERPROCESSNOISEMATRICES Generate continuous white noise acceleration models
%   Q = helperProcessNoiseMatrices(dt, noiseIntensity) returns a struct of
%   process noise covariance matrices for the supported motion models. The
%   input structure noiseIntensity can specify the following fields:
%       - CV: scalar acceleration spectral density for constant velocity
%       - CA: scalar jerk spectral density for constant acceleration
%       - CTTurnRate: standard deviation of turn-rate noise for CT model
%
%   Missing fields are replaced with reasonable defaults.
%
%   Output struct Q contains the fields:
%       Q.CV  -> 4x4 covariance for [x vx y vy]
%       Q.CT  -> 5x5 covariance for [x vx y vy omega]
%       Q.CA  -> 6x6 covariance for [x vx ax y vy ay]
%
%   All models assume independent motion in x and y dimensions.

arguments
    dt (1,1) double {mustBePositive}
    noiseIntensity struct = struct()
end

if ~isfield(noiseIntensity, 'CV') || isempty(noiseIntensity.CV)
    noiseIntensity.CV = 5;
end
if ~isfield(noiseIntensity, 'CA') || isempty(noiseIntensity.CA)
    noiseIntensity.CA = 1;
end
if ~isfield(noiseIntensity, 'CTTurnRate') || isempty(noiseIntensity.CTTurnRate)
    noiseIntensity.CTTurnRate = deg2rad(5); % rad/s noise on turn rate
end

qCV = noiseIntensity.CV;
qCA = noiseIntensity.CA;
qOmega = noiseIntensity.CTTurnRate;

Qcv_single = [dt^4/4 dt^3/2; dt^3/2 dt^2];
Q.CV = blkdiag(qCV^2 * Qcv_single, qCV^2 * Qcv_single);

Q.CA = blkdiag(qCA^2 * helperCAContinuousBlock(dt), qCA^2 * helperCAContinuousBlock(dt));

Q.CT = blkdiag(Q.CV, qOmega^2);
end

function Qblock = helperCAContinuousBlock(dt)
Qblock = [dt^5/20 dt^4/8  dt^3/6;
          dt^4/8  dt^3/3  dt^2/2;
          dt^3/6  dt^2/2  dt];
end
