%% Description: demo for mission planner class
clear all
clc

%% User Inputs
%      x   y
pos = [50, 50;
       75, 80;
       100, 150];
speed = [5, 10, 15];

%% Program
M = mission(pos, speed);

position_matrix = M.pos_vec
speed_vector = M.speed_vec
mission_plan = M.mission_plan