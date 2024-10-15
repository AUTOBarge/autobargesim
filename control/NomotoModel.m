function [dTurningRate, dpsi] = NomotoModel(states, RA, L, K_dash, T_dash)
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
U = 2.5; % This should be replaced by U= sqrt(u^2+v^2)
T1 = T_dash*(L/U);
K1 = K_dash*(U/L);
ROT = states(1);

dTurningRate = -ROT / T1 + K1 * RA / T1;
dpsi = ROT;
end