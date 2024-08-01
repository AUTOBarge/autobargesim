function main()
clc
clear
close all
fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
fprintf('AUTOBargeSim: MATLAB toolbox for the design and analysis of \nthe GNC System for autonomous inland vessels.\n');
fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
	
% Prompt user to provide the directory for shape files
usePackageShapeFiles = input('Do you want to use the maps included in the package? (y/n): ', 's');
    
if strcmpi(usePackageShapeFiles, 'y')
    % Prompt user to choose the area
    areaChoice = input('Choose the area (1 for Albert canal, 2 for Leuven area): ');
    switch areaChoice
        case 1
        shapeFileDirectory = fullfile(pwd, 'maps','demo','.shp', 'Albert canal');
        defaultStart = [4.4266966, 51.2383286];  % Default values for Albert canal
        defaultEnd = [4.5088209, 51.2361481];   
        case 2
        shapeFileDirectory = fullfile(pwd, 'maps','demo','.shp', 'Leuven area');
        defaultStart = [0, 0];  % Default values for Leven area
        defaultEnd = [0, 0];
        otherwise
        error('Invalid choice. Please run the program again and select a valid area.');
    end
else
    shapeFileDirectory = input('Please provide the full directory path for the shape files: ', 's');
    defaultStart = [];  % No defaults for custom files
    defaultEnd = [];    
    end
    
fprintf('Shape files directory set to: %s\n', shapeFileDirectory);
    
% Create a 'process' class object and plot the area
desirename=["depare","bridge","wtwaxs","lndare"];
warning('off','MATLAB:polyshape:repairedBySimplify');
process = maps.processor(desirename, shapeFileDirectory);
process.plot();

% Prompt user to provide start and end points
fprintf('Please select the start and end points for the route from the map \n');
st=drawpoint;
en=drawpoint;
start_long = st.Position(1);
start_lat= st.Position(2);
end_long = en.Position(1);
end_lat = en.Position(2);
% Use defaults if user input is empty
if isempty(start_lat) || isempty(start_long) || isempty(end_lat) || isempty(end_long)
    startPoint = defaultStart;
    endPoint = defaultEnd;
else
    startPoint = [start_long, start_lat];
    endPoint = [end_long, end_lat];
end
 
% Create a 'plan' class object and provide it the start and end points, plot the path
plan = maps.planner(process.pgon_memory);
plan = plan.plan_path([startPoint(1), startPoint(2)], [endPoint(1), endPoint(2)]);
plan.plot_path(1);
    
%%
wp_wgs84 = plan.path_points;
wgs84 = wgs84Ellipsoid;
lon0 = wp_wgs84(1,1);
lat0 = wp_wgs84(1,2);
wp_pos = zeros(length(wp_wgs84),2);
height = 0;
for i =1:length(wp_wgs84)
    [xEast,yNorth,zUp] = geodetic2enu(wp_wgs84(i,2),wp_wgs84(i,1),height,lat0,lon0,height,wgs84);
    wp_pos(i,:) = [xEast,yNorth];
end


%Initialisation
t_f = 8000; % final simulation time (sec)
h = 0.2; % sample time (sec)

%Create and initialise guidance class object
los = LOSguidance();
wp_speed = 100*ones(length(wp_pos),1);
wp_idx = 1;
initial_state = [0 0 0 0 0 0]'; % Initial state [u v r x y psi] in column
[chi, U] = los.compute_LOSRef(wp_pos, wp_speed, initial_state', wp_idx,1);
initial_state = [0 0 0 0 0 chi]'; % Initial state [u v r x y psi] in column
%states_plan = initial_state;
%state_new =zeros(1,6); %Variable to update states_plan
chi_d = zeros(1,t_f); %Desired track is stored in chi_d

%Create and initialise model and actuator class objects
ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", 0);
prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
Vessel = modelClass(ship_dim);
SRSP = actuatorClass(ship_dim, prop_params, rud_params);
Vessel = Vessel.ship_params_calculator(env_set);
Vessel.sensor_state = initial_state;

%Create and initialise control class object
initial_ctrl = [200;0]; % Initial control
ctrl_last = initial_ctrl;
pid_params = struct("K_p",35,"T_i",33,"T_d",22,"psi_d_old",0,"error_old",0);
mpc_params = struct('Ts', 0.2, 'N', 80, 'headingGain', 100, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34);
%Flag_cont = 0; %0 for PID, 1 for MPC
Flag_cont = input('Select the controller (Type 0 for PID or 1 for MPC): ');    
control=controlClass(pid_params,mpc_params,Flag_cont);
if Flag_cont == 1
    mpc_nlp = control.init_mpc();
    args = control.constraintcreator();
    next_guess = control.initial_guess_creator(vertcat(Vessel.sensor_state(3),Vessel.sensor_state(6)), ctrl_last);
end

% Start the loop for simulation
for i=1:t_f
    states=Vessel.sensor_state;
    vel = states(1:3);
    psi = states(6);
    r = states(3);
    time = (i - 1) * h; % simulation time in seconds
    % Find the active waypoint
    wp_idx = los.find_active_wp_segment(wp_pos, states', wp_idx);
    % Call LOS algorithm
    [chi, U] = los.compute_LOSRef(wp_pos, wp_speed, states', wp_idx,1);

    if Flag_cont == 1 % Implement the MPC controller
        r_d = chi - psi;
        [ctrl_command_MPC, next_guess,control] = control.LowLevelMPCCtrl(vertcat(states,ctrl_last), chi, r_d, args, next_guess, mpc_nlp);
        ctrl_command = [340 ; ctrl_command_MPC];%n_c
    else % Implement the PID controller
        [ctrl_command,control] = control.LowLevelPIDCtrl(chi,r,psi,h);
    end
    
    % Provide the vessel with the computed control command
    SRSP = SRSP.act_response(ctrl_last, ctrl_command,h);
    %[J_P, K_T, SRSP] = SRSP.get_prop_force(vel);
    %SRSP = SRSP.get_rud_force(vel, J_P, K_T);
    %SRSP = SRSP.get_act_force();
    Vessel = Vessel.sensor_dynamic_model(SRSP, env_set);

    % Vessle's state update (Euler integration)
    Vessel.sensor_state = Vessel.sensor_state + Vessel.sensor_state_dot * h;
    
    % Update control action
    ctrl_last = SRSP.ctrl_actual';

    % store data for presentation
    xout(i, :) = [time, Vessel.sensor_state', SRSP.ctrl_actual, Vessel.sensor_state_dot(1:3)'];
    chi_d(i)=chi;

    %End condition
    
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
figure(2)
plot(wp_pos(:,1)/ 38.5,wp_pos(:,2)/ 38.5,'-*r',LineWidth=1.5)
hold on
plot(x / 38.5, y / 38.5, '-b',LineWidth=1.5)
grid, axis('equal'), xlabel('East (y/L)'), ylabel('North (x/L)'), title('Ship position')

figure(3)
subplot(321),plot(t,u,'r'),xlabel('time (s)'),title('u (m/s)'),grid
hold on;
subplot(322),plot(t,v,'r'),xlabel('time (s)'),title('v (m/s)'),grid
subplot(323),plot(t,r,'r'),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(324),plot(t,psi,'r'),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(325),plot(t,delta,'r'),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid 
subplot(326),plot(t,n,'r'),xlabel('time (s)'),title('rpm'),grid

end