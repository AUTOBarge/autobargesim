%% Description
% This is a demo for the Scenario-Based Model Predictive Control algorithm. 
% The ship is expected to follow the predefined waypoints (wp_pos) while
% avoiding other target vessels.

%% Program
close all;
clear;
clc;

% Starting waypoint index own-ship
wp_idx_os = 1;
% Starting waypoint index target-ship
wp_idx_ts = 1;

% sampling time
Tp = 0.5;

% Initialize LOS Guidance with same parameters for both vessels
los = LOSguidance();

% Initialize SBMPC only for the own-ship. Target vessel does not run any
% collision avoidance algorithm.
colav = sbmpc(10, Tp);

SCENARIO = 'HO';
switch SCENARIO
    case 'HO'
        % Predefined waypoints for own-ship: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
        wp_pos_os = [10 10;
                     140 140];
        % Predefined waypoints for target-ship: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
        wp_pos_ts = [120 140;
                     30 10];
        
        % Predefined surge speed at each waypoint segment for own-ship: wp_speed = [U_1;...;U_n]
        wp_speed_os = [1; 0];
        % Predefined surge speed at each waypoint segment for target-ship: wp_speed = [U_1;...;U_n]
        wp_speed_ts = [1; 0];

        % State of own ship w.r.t inertial coordinate frame state_x=[u, v, r, x, y, psi]
        state_os = [0, 0, 0, 10, 10, deg2rad(45)];
        % State of target ship w.r.t inertial coordinate frame state_x=[u, v, r, x, y, psi]
        state_ts = [0, 0, 0, 120, 140, deg2rad(135)];
    case 'CR'
        % Predefined waypoints for own-ship: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
        wp_pos_os = [10 10;
                     140 140];
        % Predefined waypoints for target-ship: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
        wp_pos_ts = [140 10;
                     10 140];
        
        % Predefined surge speed at each waypoint segment for own-ship: wp_speed = [U_1;...;U_n]
        wp_speed_os = [1; 0];
        % Predefined surge speed at each waypoint segment for target-ship: wp_speed = [U_1;...;U_n]
        wp_speed_ts = [1; 0];

        % State of own ship w.r.t inertial coordinate frame state_x=[u, v, r, x, y, psi]
        state_os = [0, 0, 0, 10, 10, deg2rad(45)];
        % State of target ship w.r.t inertial coordinate frame state_x=[u, v, r, x, y, psi]
        state_ts = [0, 0, 0, 140, 10, deg2rad(135)];
    otherwise
        error('Invalid COLREGS scenario')
end

% Temporary variable
state_new = zeros(1, 6);

% Variables to hold previous SBMPC modifications for the guidance commands
chi_m_last_os = 1;
U_m_last_os = 0;

% Start the loop for simulation
for i=1:600
    % Find the active waypoint
    wp_idx_os = los.find_active_wp_segment(wp_pos_os, state_os, wp_idx_os);
    wp_idx_ts = los.find_active_wp_segment(wp_pos_ts, state_ts, wp_idx_ts);

    % Call LOS algorithm
    [chi_os, U_os] = los.compute_LOSRef(wp_pos_os, wp_speed_os, state_os, wp_idx_os, 1);
    [chi_ts, U_ts] = los.compute_LOSRef(wp_pos_ts, wp_speed_ts, state_ts, wp_idx_ts, 1);

    % Modify course and speed of the own-ship using SBMPC collision
    % avoidance
    [chi_os, U_os, chi_m_last_os, U_m_last_os] = colav.run_sbmpc(state_os, chi_os, U_os, ...
                                                                 chi_m_last_os, U_m_last_os, state_ts);

    % Update state
    state_os = vessel_model(state_os, chi_os, U_os, Tp);
    state_ts = vessel_model(state_ts, chi_ts, U_ts, Tp);
    
    % Plot the trajectory for visualization
    cla
    plot(wp_pos_os(:,1), wp_pos_os(:,2), '-*r')
    plot(wp_pos_ts(:,1), wp_pos_ts(:,2), '-xr')
    
    hold on
    % own-ship in blue
    plot(state_os(4), state_os(5), 'ob', LineWidth = 2);
    plot([state_os(4), state_os(4) + 5 * cos(state_os(6))], ...
         [state_os(5), state_os(5) + 5 * sin(state_os(6))], ...
         '-b', LineWidth = 2);
    % target-ship in black
    plot(state_ts(4), state_ts(5), 'oblack', LineWidth = 2);
    plot([state_ts(4), state_ts(4) + 5 * cos(state_ts(6))], ...
         [state_ts(5), state_ts(5) + 5 * sin(state_ts(6))], ...
         '-black', LineWidth = 2);

    xlabel('X (m)'),ylabel('Y (m)'),grid on
    axis([0 150 0 150]);
    legend({'own-ship waypoints', 'target-ship waypoints'}, ...
            'Location','north');
    pause(0.01);
end

% This is an ad-hoc kinematic vessel model used to derive the state of both
% own-ship and target-ship at the next time step
function state_new = vessel_model(state, chi, U, dt)
    state_new(4) = state(4) + dt * U * cos(state(6));
    state_new(5) = state(5) + dt * U * sin(state(6));
    state_new(6) = chi;
    state_new(1) = U;
end