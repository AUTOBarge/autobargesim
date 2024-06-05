%% Description
% This is a demo for the line-of-sight (LOS) guidance algorithm. Ship is
% expected to follow the predefined waypoints (wp_pos)

%%
close all;
clear;
clc;
% Predefined waypoints: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
wp_pos = [50 50;
    70 70;
    90 100;
    100 150];
% Predefined surge speed at each waypoint segment: wp_speed = [U_1;...;U_n]
wp_speed = [2;
    3;
    3;
    2];
% Initial position of own ship with respect to inertial coordinate frame: x_0 = [x, y, chi, U]
x_0 = [0,0,0,50, 45, deg2rad(45)];
%
wp_idx = 1;

los = LOSguidance();
% State of own ship w.r.t inertial coordinate frame state_x=[x, y, chi, U]
state = x_0;
% sampling time
Tp=0.05;
% temporary variable
state_new =zeros(1,6);
% Start the loop for simulation
% inside the loop
for i=1:800
    % Find the active waypoint
    wp_idx = los.find_active_wp_segment(wp_pos, state, wp_idx);
    % Call LOS algorithm
    [chi, U] = los.compute_LOSRef(wp_pos, wp_speed, state, wp_idx,1);

    % Update state
    state_new(4) =state(4) + Tp*U*cos(state(6));
    state_new(5) =state(5) + Tp*U*sin(state(6));
    state_new(6) =chi;
    state_new(1) =U;
    state=state_new;
    % Plot the trajectory for visualization
    cla
    plot(wp_pos(:,1),wp_pos(:,2),'-*r')
    
    hold on
    plot(state(4),state(5),'ob',LineWidth=2);
    plot([state(4),state(4)+5*cos(state(6))],[state(5),state(5)+5*sin(state(6))],'-b',LineWidth=2);
    xlabel('X (m)'),ylabel('Y (m)'),grid on
    axis([0 150 0 150]);
    legend({'waypoint','position of ship','heading direction'},'Location','southeast');
    pause(0.01);
end
% inside the loop