clear
clc
close all

folder = cd;
folder = fullfile(folder, '..');
addpath(folder); % Set the +maps (namespace) path

folder = append(cd,'\.shp\Gent area'); % Set the folder path for reading .shp files
desirename=["depare","bridge","wtwaxs","lndare"]; %give the desirename %"notmrk"

p = maps.processor(desirename, folder);
p.plot();
%% 

pl = maps.planner(p.pgon_memory);
% Defines the given starting and ending points
% given_point1 = [3,755556, 51.0064];
% given_point2 = [3.74138, 51.0107];

[given_point1, given_point2]=pl.generate_random_points(); % generate start(given point 1) and end points(given point 2) for testing

% planned path
pl = pl.plan_path(given_point1, given_point2);

% Draw Path
%   num - Controls the content to be plotted
%         1: All information
%         2: Only waypoints, start point, end point, without node, best
%         path and depth
pl.plot_path(1);