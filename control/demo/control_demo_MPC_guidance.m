%%Control module demo 
% 
% Demo for the implementation of waypoint tracking
% using an MPC heading controller.
%
% Authors:
%   AmirReza Haqshenas M.
%   Abhishek Dhyani.
% Date:
%	11/10/2024
% Version:
% 	1.0
clc; clear all; close all;
%% Setting
t_f = 500; % final simulation time (sec)
h = 0.2; % sample time (sec)

los = LOSguidance();
% Predefined waypoints: wp_pos =[x_1 y_1; x_2 y_2;...; x_n y_n]
wp_pos = [0 0; 50 50; 170 170;340 350;480 580; 580 950];
% Predefined surge speed at each waypoint segment: wp_speed = [U_1;...;U_n]
wp_speed = ones(length(wp_pos),1);
wp_idx = 1;
chi_d_prev = atan2(wp_pos(2, 2)-wp_pos(1, 2),wp_pos(2, 1)-wp_pos(1, 1));

%% Model, environment and controller parameters
ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", pi);
prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
states = [0, 0, 0, 0, 0, deg2rad(45)]'; % Initial state [u v r x y psi] in column
Flag_cont = 2;
initial_ctrl = [200; 0]; % Initial control
xtetot = 0;
psi_er_tot = 0;

%% Initialization
Vessel = modelClass(ship_dim);
SRSP = actuatorClass(ship_dim, prop_params, rud_params);
Vessel = Vessel.ship_params_calculator(env_set, rud_params);
Vessel.sensor_state = states;
ctrl_last = initial_ctrl;
L = Vessel.ship_dim.L;
K_dash = Vessel.KTindex.K_dash;
T_dash = Vessel.KTindex.T_dash;
mpc_params = struct('Ts', h, 'N', 80, 'headingGain', 30, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34, 'K_dash', K_dash, 'T_dash', T_dash, 'L', L);

MPCobj=controlClass(Flag_cont,mpc_params);
mpc_nlp = MPCobj.init_mpc();
args = MPCobj.constraintcreator();
next_guess = MPCobj.initial_guess_creator(vertcat(Vessel.sensor_state(3),Vessel.sensor_state(6)), ctrl_last);

%% --- MAIN LOOP ---
N = round(t_f / h); % number of samples
n_c = 250; % defined command for the RPM of the propellers

for i=1:N+1
    time = (i - 1) * h;
    % State [u v r x y psi delta n], Controls [n_c delta_c]
    states = vertcat(Vessel.sensor_state, ctrl_last);
    vel = states(1:3);
    
    % Find the active waypoint
    wp_idx = los.find_active_wp_segment(wp_pos, Vessel.sensor_state', wp_idx);
    
    % Call LOS algorithm
    [chi, U] = los.compute_LOSRef(wp_pos, wp_speed, Vessel.sensor_state', wp_idx,1, chi_d_prev, i);
    psi_d=chi;
    chi_d_prev = chi;
    r_d = psi_d - states(6);
    [ctrl_command_MPC, next_guess,~] = MPCobj.LowLevelMPCCtrl(states, psi_d, r_d, args, next_guess, mpc_nlp);
    
    ctrl_command = [n_c ; ctrl_command_MPC];
    SRSP = SRSP.act_response(ctrl_last, ctrl_command, h);
    [J_P, K_T, SRSP] = SRSP.get_prop_force(env_set, vel);
    SRSP = SRSP.get_rud_force(env_set, vel, J_P, K_T);
    SRSP = SRSP.get_act_force();
    Vessel = Vessel.sensor_dynamic_model(SRSP,env_set);

    % Euler integration
    Vessel.sensor_state = Vessel.sensor_state + Vessel.sensor_state_dot * h;

    % Calculate the performance indices
    [xte,psi_er,xtetot,psi_er_tot,MPCobj] = MPCobj.XTECalc(Vessel.sensor_state, psi_d, wp_pos, wp_idx, xtetot, psi_er_tot);
    
    % Update control action
    ctrl_last = SRSP.ctrl_actual';

    % store data for presentation
    xout(i, :) = [time, Vessel.sensor_state', SRSP.ctrl_actual, Vessel.sensor_state_dot(1:3)'];
    %disp(i)

    % store the performance indices
    pout(i,:) = [xte,psi_er,xtetot,psi_er_tot];

    %End condition
    distance = norm([xout(i, 5)-wp_pos(end,1),xout(i, 6)-wp_pos(end,2)],2);
    if distance <10
        break
    end
    
end

% time-series
t = xout(:, 1);
u = xout(:, 2);
v = xout(:, 3);
r = xout(:, 4) * 180 / pi;
x = xout(:, 5);
y = xout(:, 6);
psi = xout(:, 7) * 180 / pi;
n = xout(:, 8);
delta = xout(:, 9);
u_dot = xout(:, 10);
v_dot = xout(:, 11);
r_dot = xout(:, 12) * 180 / pi;

xte = pout(:,1);
psi_er = pout(:,2) * 180 / pi;
xtetot = pout(:,3);
psi_er_tot = pout(:,4) * 180 / pi;

%% Plots
figure(1)
plot(wp_pos(:,2),wp_pos(:,1),'-*r')
hold on
plot(y, x, 'b')
grid, axis('equal'), xlabel('East (y)'), ylabel('North (x)'), title('Ship position')

figure(2)
subplot(321),plot(t,u,'r'),xlabel('time (s)'),title('u (m/s)'),grid
hold on;
subplot(322),plot(t,v,'r'),xlabel('time (s)'),title('v (m/s)'),grid
subplot(323),plot(t,r,'r'),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(324),plot(t,psi,'r'),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(325),plot(t,delta,'r'),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid 
subplot(326),plot(t,n,'r'),xlabel('time (s)'),title('rpm'),grid

figure(3)
subplot(211),plot(t,xte),xlabel('time (s)'),title('Cross-track error (m)'),grid
subplot(212),plot(t,psi_er),xlabel('time (s)'),title('Heading error (deg)'),grid
fprintf('Total accumulated cross-track error:%d \n',xtetot(end));
fprintf('Total accumulated heading error:%d \n',psi_er_tot(end));


