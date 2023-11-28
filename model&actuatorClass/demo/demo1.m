% Description: Demo for model class and actuator class

%% Setting
t_f = 500; % final simulation time (sec)
h = 0.1; % sample time (sec)
ctrl_command = [340 0]; % [n_c delta_c]

ship_dim = dictionary("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
env_set = dictionary("rho_water", 1000, "H_d", 2);
prop_params = dictionary("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
rud_params = dictionary("C_R", 1.6, "B_R", 1.4, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
initial_state = [4 0 0 0 0 0]'; % Initial state [u v r x y psi] in column
initial_ctrl = [340 0]; % Initial control

%% Initialization
Vessel = modelClass(ship_dim, env_set);
SRSP = actuatorClass(ship_dim, env_set, prop_params, rud_params);
Vessel = Vessel.ship_params_calculator();
Vessel.sensor_state = initial_state;
ctrl_last = initial_ctrl;

%% --- MAIN LOOP ---
N = round(t_f / h); % number of samples
xout = zeros(N + 1, 12); % Storing data

for i = 1:N + 1
    vel = Vessel.sensor_state(1:3);
    time = (i - 1) * h; % simulation time in seconds
    SRSP = SRSP.act_response(ctrl_last, ctrl_command, h);
    [J_P, K_T, SRSP] = SRSP.get_prop_force(vel);
    SRSP = SRSP.get_rud_force(vel, J_P, K_T);
    SRSP = SRSP.get_act_force();
    Vessel = Vessel.sensor_dynamic_model(SRSP.tau_act);

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

% plots
figure(1)
plot(y / 38.5, x / 38.5, 'r')
grid, axis('equal'), xlabel('East (y/L)'), ylabel('North (x/L)'), title('Ship position')

figure(2)
subplot(331), plot(t, u, 'r'), xlabel('time (s)'), title('u (m/s)'), grid
subplot(332), plot(t, v, 'r'), xlabel('time (s)'), title('v (m/s)'), grid
subplot(334), plot(t, psi, 'r'), xlabel('time (s)'), title('yaw angle \psi (deg)'), grid
subplot(335), plot(t, delta, 'r'), xlabel('time (s)'), title('rudder angle \delta (deg)'), grid
subplot(336), plot(t, n, 'r'), xlabel('time (s)'), title('rpm'), grid
subplot(337), plot(t, u_dot, 'r'), xlabel('time (s)'), title('u dot (m/s)'), grid
subplot(338), plot(t, v_dot, 'r'), xlabel('time (s)'), title('v dot (m/s)'), grid
subplot(339), plot(t, r_dot, 'r'), xlabel('time (s)'), title('yaw rate r dot (deg/s)'), grid
