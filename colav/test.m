clear;
clc;

% init guidance
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

% init sbmpc
T = 10; dt = 1;
sbmpc = sbmpc(T, dt);
x_ts = [100, 100,deg2rad(-60), 1;
        0, 0, deg2rad(60), 0.5]; % target ships
chi_m_last = 0;
U_m_last = 1;

% inside the loop
wp_idx = los.find_active_wp_segment(wp_pos, x, wp_idx);
[chi_d, U_d] = los.compute_LOSRef(wp_pos, wp_speed, x, wp_idx);

[chi_c, U_c, chi_m, U_m] = run_sbmpc(x, chi_d, U_d, chi_m_last, U_m_last, x_ts);
% inside the loop