clear
clc
close all
addpath( 'E:\Git\simulator-dev\maps'); % Set the +maps (namespace) path
folder = 'E:\Git\simulator-dev\maps\demo\.shp\Albert canal';  % Set the folder path for reading .shp files
desirename=["depare","bridge","wtwaxs","lndare"]; %give the desirename %"notmrk"

p = maps.processor(desirename, folder);
p.plot();
%% 

pl = maps.planner(p.pgon_memory);
% Defines the given starting and ending points
% given_point1 = [~, ~];
% given_point2 = [~, ~];

[given_point1, given_point2]=pl.generate_random_points(); % generate start(given point 1) and end points(given point 2) for testing

% planned path
pl = pl.plan_path(given_point1, given_point2);

% Draw Path
pl.plot_path(1);