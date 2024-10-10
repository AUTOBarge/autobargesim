%%Control module demo 
% 
% Demo for the implementation of a heading setpoint 
% tracking PID controller.
%
% Author:
%   Abhishek Dhyani
% Date:
%	10/10/2024
% Version:
% 	1.0
clc; clear all; close all;
%% Setting
t_f = 500; % final simulation time (sec)
h = 0.1; % sample time (sec)

ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", pi);
prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
rud_params = struct("C_R", 1.6, "B_R", 1.4, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
initial_state = [0 0 0 0 0 0]'; % Initial state [u v r x y psi] in column
initial_ctrl = [0 0]; % Initial control
%K_p: Controller P-gain, T_i: Controller integration time, T_d: Controller derivative time
pid_params = struct("K_p",50,"T_i",10,"T_d",40,"psi_d_old",0,"error_old",0);
Flag_cont = 1;
psi_d=[pi/4];
%% Initialization
Vessel = modelClass(ship_dim);
SRSP = actuatorClass(ship_dim, prop_params, rud_params);
PIDobj=controlClass(Flag_cont,pid_params);
Vessel = Vessel.ship_params_calculator(env_set, rud_params);
Vessel.sensor_state = initial_state;
ctrl_last = initial_ctrl;

%% --- MAIN LOOP ---
N = round(t_f / h); % number of samples
xout = zeros(N + 1, 12); % Storing data

for i=1:N+1 
    vel = Vessel.sensor_state(1:3);
    r = Vessel.sensor_state(3); %rad/s
    psi = Vessel.sensor_state(6);
    time = (i - 1) * h;
    [ctrl_command,PIDobj] = PIDobj.LowLevelPIDCtrl(psi_d,r,psi,h);
    SRSP = SRSP.act_response(ctrl_last, ctrl_command, h);
    [J_P, K_T, SRSP] = SRSP.get_prop_force(env_set, vel);
    SRSP = SRSP.get_rud_force(env_set, vel, J_P, K_T);
    SRSP = SRSP.get_act_force();
    Vessel = Vessel.sensor_dynamic_model(SRSP,env_set);
    
    % Euler integration
    Vessel.sensor_state = Vessel.sensor_state + Vessel.sensor_state_dot * h;

    % Update control action
    ctrl_last = SRSP.ctrl_actual;

    % store data for presentation
    xout(i, :) = [time, Vessel.sensor_state', SRSP.ctrl_actual, Vessel.sensor_state_dot(1:3)'];
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

%% Plots
figure(1)
plot(y / 38.5, x / 38.5, 'r')
grid, axis('equal'), xlabel('East (y/L)'), ylabel('North (x/L)'), title('Ship position')

figure(2)
subplot(321),plot(t,u,'r'),xlabel('time (s)'),title('u (m/s)'),grid
hold on;
subplot(322),plot(t,v,'r'),xlabel('time (s)'),title('v (m/s)'),grid
subplot(323),plot(t,r,'r'),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(324),plot(t,psi,'r'),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(325),plot(t,delta,'r'),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid 
subplot(326),plot(t,n,'r'),xlabel('time (s)'),title('rpm'),grid
        
