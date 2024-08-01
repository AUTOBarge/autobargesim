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
T1 = 20.193996887082818;
K1 = 0.003081885184454478;
K2 = 4.493237483452767e-06;

ROT = states(1);

dTurningRate = -ROT / T1 + K1 * RA / T1 + K2;
dpsi = ROT;
end