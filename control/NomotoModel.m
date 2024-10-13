function [dTurningRate, dpsi] = NomotoModel(states, RA)
%% [dTurningRate, dpsi] = NomotoModel(states, RA)
%% Description:
% this function provides the Nomoto model to be used in the LowlevelMPC
% Author:
%   AmirReza Haqshenas M.
% Date:
%	22/02/2024
% Version:
% 	1.0
% states = [TurningRate HeadingTrue]
% RA = RudderAngle
%% identified model
T1 = 5.107025;
K1 = 0.002253;
K2 = 0;

ROT = states(1);

dTurningRate = -ROT / T1 + K1 * RA / T1 + K2;
dpsi = ROT;
end