clear;
clc;

wp_pos = [50 50;
          70 70;
          90 90;
          100 100];
wp_speed = [2;
            3;
            3;
            2];
x = [50, 45, deg2rad(45), 0];
wp_idx = 1;

los = LOSguidance();

% inside the loop
wp_idx = los.find_active_wp_segment(wp_pos, x, wp_idx);
[chi, U] = los.compute_LOSRef(wp_pos, wp_speed, x, wp_idx);
% inside the loop