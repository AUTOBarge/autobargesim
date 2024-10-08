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
    areaChoice = input('Choose the area\n 1 for Albert canal or,\n 2 for Leuven area: ');
    switch areaChoice
        case 1
        shapeFileDirectory = fullfile(pwd, 'maps','demo','.shp', 'Albert canal');
        defaultStart = [4.4266966, 51.2383286];  % Default values for Albert canal
        defaultEnd = [4.5088209, 51.2361481];   
        case 2
        shapeFileDirectory = fullfile(pwd, 'maps','demo','.shp', 'Leuven area');
        defaultStart = [0, 0];  % Default values for Leuven area
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

%% Initialisation
t_f = 8000; % final simulation time (sec)
h = 0.2; % sample time (sec)

%% Vessel 1
vessel1 = [];

% Prompt user to provide start and end points
fprintf('Please select the start and end points for the route of the ownship from the map \n');
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

%
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

% Create and initialise guidance class object
vessel1.guidance = LOSguidance();
vessel1.wp.pos = wp_pos;
vessel1.wp.speed = 100*ones(length(wp_pos),1);
vessel1.wp.idx = 1;
initial_state = [0 0 0 0 0 0]'; % Initial state [u v r x y psi] in column
[chi, ~] = vessel1.guidance.compute_LOSRef(vessel1.wp.pos, vessel1.wp.speed, initial_state', vessel1.wp.idx, 1);
initial_state = [0 0 0 wp_pos(1, 1) wp_pos(1, 2) chi]'; % Initial state [u v r x y psi] in column
chi_d = zeros(1,t_f); %Desired track is stored in chi_d

% Create and initialise model class objects
ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", 0);
vessel1.model = modelClass(ship_dim);
vessel1.model = vessel1.model.ship_params_calculator(env_set);
vessel1.model.sensor_state = initial_state;

% Create and initialise actuator class objects
prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
vessel1.actuators = actuatorClass(ship_dim, prop_params, rud_params);

% Create and initialise control class object
pid_params = struct("K_p",35,"T_i",33,"T_d",22,"psi_d_old",0,"error_old",0);
mpc_params = struct('Ts', 0.2, 'N', 80, 'headingGain', 100, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34);
Flag_cont = input('Select the controller (Type 1 for PID or 2 for MPC): '); 

vessel1.control.output = [200; 0]; % Initial control
vessel1.control.param = [];
vessel1.err.xtetot = 0;
vessel1.err.psi_er_tot = 0;
if Flag_cont == 2
    vessel1.control.model=controlClass(Flag_cont,mpc_params);
    vessel1.control.param.mpc_nlp = vessel1.control.model.init_mpc();
    vessel1.control.param.args = vessel1.control.model.constraintcreator();
    vessel1.control.param.next_guess = vessel1.control.model.initial_guess_creator(vertcat(vessel1.model.sensor_state(3), vessel1.model.sensor_state(6)), vessel1.control.output);
elseif Flag_cont == 1
    vessel1.control.model = controlClass(Flag_cont,pid_params);
else
    error('Invalid Input. Please run main.m again');
end

add_ts_vessel = input('Do you want to add a target vessel? (y/n): ', 's');
if strcmpi(add_ts_vessel, 'y')
    % Initialize colision avoidance
    vessel1.colav.alg = sbmpc(10, h);
    vessel1.colav.param = [1; 0]; % chi_m, U_m
end

%% Vessel 2
vessel2 = [];

% Prompt user to provide start and end points
if strcmpi(add_ts_vessel, 'y')
    fprintf('Please select the start and end points for the route of the targetship from the map \n');
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
    
    %
    wp_wgs84 = plan.path_points;
    wgs84 = wgs84Ellipsoid;
    wp_pos = zeros(length(wp_wgs84),2);
    height = 0;
    for i =1:length(wp_wgs84)
        [xEast,yNorth,zUp] = geodetic2enu(wp_wgs84(i,2),wp_wgs84(i,1),height,lat0,lon0,height,wgs84);
        wp_pos(i,:) = [xEast,yNorth];
    end
    
    % Create and initialise guidance class object
    vessel2.guidance = LOSguidance();
    vessel2.wp.pos = wp_pos;
    vessel2.wp.speed = 100*ones(length(wp_pos),1);
    vessel2.wp.idx = 1;
    [chi, ~] = vessel2.guidance.compute_LOSRef(vessel2.wp.pos, vessel2.wp.speed, [0 0 0 0 0 0], vessel2.wp.idx, 1);
    initial_state = [0 0 0 wp_pos(1, 1) wp_pos(1, 2) chi]'; % Initial state [u v r x y psi] in column
    
    % Create and initialise model class objects
    ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
    env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", 0);
    vessel2.model = modelClass(ship_dim);
    vessel2.model = vessel2.model.ship_params_calculator(env_set);
    vessel2.model.sensor_state = initial_state;
    
    % Create and initialise actuator class objects
    prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
    rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
    vessel2.actuators = actuatorClass(ship_dim, prop_params, rud_params);
    
    % Create and initialise control class object
    pid_params = struct("K_p",35,"T_i",33,"T_d",22,"psi_d_old",0,"error_old",0);
    mpc_params = struct('Ts', 0.2, 'N', 80, 'headingGain', 100, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34);
    Flag_cont = 1; % The second vessel will get PID controller by default
    vessel2.control.output = [200; 0]; % Initial control
    vessel2.control.param = [];
    vessel2.err.xtetot = 0;
    vessel2.err.psi_er_tot = 0;
    if Flag_cont == 2
        vessel2.control.model=controlClass(Flag_cont,mpc_params);
        vessel2.control.param.mpc_nlp = vessel2.control.model.init_mpc();
        vessel2.control.param.args = vessel2.control.model.constraintcreator();
        vessel2.control.param.next_guess = vessel2.control.model.initial_guess_creator(vertcat(vessel2.model.sensor_state(3), vessel2.model.sensor_state(6)), vessel2.control.output);
    else
        vessel2.control.model = controlClass(Flag_cont,pid_params);
    end

    % Initialize colision avoidance
    vessel2.colav.alg = sbmpc(10, h);
    vessel2.colav.param = [1; 0]; % chi_m, U_m
end

vessels = [vessel1; vessel2];
STOP = zeros(numel(vessels),1);
stop_time =zeros(numel(vessels),1);
%% Start the loop for simulation
for i = 1:t_f
    vessels_hold = vessels;
    for j = 1:numel(vessels_hold)
        if STOP(j)==0
        os = vessels_hold(j);
        ts = vessels_hold(setxor(1:numel(vessels_hold), j));

        ts_sensor_states = [];
        for v = 1:numel(ts)
            ts_sensor_states(v, :) = ts(v).model.sensor_state;
        end
        ts_sensor_states = reshape(ts_sensor_states, numel(ts), []);

        vel = os.model.sensor_state(1:3);
        psi = os.model.sensor_state(6);
        r = os.model.sensor_state(3);
        time = (i - 1) * h; % simulation time in seconds
    
        % Find the active waypoint
        os.wp.idx = os.guidance.find_active_wp_segment(os.wp.pos, os.model.sensor_state', os.wp.idx);
    
        % Call LOS algorithm
        [chi, U] = os.guidance.compute_LOSRef(os.wp.pos, os.wp.speed, os.model.sensor_state', os.wp.idx, 1);
        
        if strcmpi(add_ts_vessel, 'y')
            [chi, U, os.colav.parameters(1), os.colav.parameters(2)] = os.colav.alg.run_sbmpc(os.model.sensor_state', ...
                                                                                               chi, U, ...
                                                                                               os.colav.param(1), ...
                                                                                               os.colav.param(2), ...
                                                                                               ts_sensor_states);
        end
    
        if Flag_cont == 2 % Implement the MPC controller
            r_d = chi - psi;
            [ctrl_command_MPC, os.control.param.next_guess, os.control.model] = os.control.model.LowLevelMPCCtrl(vertcat(os.model.sensor_state, os.control.output), chi, r_d, os.control.param.args, os.control.param.next_guess, os.control.param.mpc_nlp);
            ctrl_command = [340; ctrl_command_MPC];
        else  % Implement the PID controller
            [ctrl_command, os.control.model] = os.control.model.LowLevelPIDCtrl(chi, r, psi, h);
        end
        
        % Provide the vessel with the computed control command
        os.actuators = os.actuators.act_response(os.control.output, ctrl_command, h);
        os.model = os.model.sensor_dynamic_model(os.actuators, env_set);
    
        % Vessel's state update (Euler integration)
        os.model.sensor_state = os.model.sensor_state + os.model.sensor_state_dot * h;
        
        % Calculate the performance indices
        [xte,psi_er,os.err.xtetot,os.err.psi_er_tot,os.control.model] = os.control.model.XTECalc(os.model.sensor_state, chi, os.wp.pos, os.wp.idx, os.err.xtetot, os.err.psi_er_tot);
        
        % Update control action
        os.control.output = os.actuators.ctrl_actual';
    
        % store data for presentation
        xout(j, i, :) = [time, os.model.sensor_state', os.actuators.ctrl_actual, os.model.sensor_state_dot(1:3)'];
        vessels_hold(j) = os;

        % store the performance indices
        pout(j, i, :) = [xte, psi_er, os.err.xtetot, os.err.psi_er_tot];
        % Checking if OS reaching the last wp:
        x_cur=os.model.sensor_state(4);
        y_cur=os.model.sensor_state(5);
        distance = norm([x_cur-os.wp.pos(end,1),y_cur-os.wp.pos(end,2)],2);
        if distance < 3
            STOP(j)= 1; % Rise the stop flag for this vessel
            stop_time(j)=i; % Record the stop time
        end
        else
            % Vessel keep the same position with zero velocity
            os = vessels_hold(j);
            os.model.sensor_state = [0;0;0;os.model.sensor_state(4);os.model.sensor_state(5);os.model.sensor_state(6)];
            os.model.sensor_state_dot = [0;0;0;0;0;0];
            os.control.output     = [0;0];
            xout(j, i, :) = [time, os.model.sensor_state', os.actuators.ctrl_actual, os.model.sensor_state_dot(1:3)'];            
            vessels_hold(j) = os;
            pout(j, i, :)= pout(j, i-1, :);
        end
    end
    if prod(STOP)==1
        break;
    end
    vessels = vessels_hold;
end

% time-series
t = xout(1, :, 1);
u = xout(1, :, 2);
v = xout(1, :, 3);
r = xout(1, :, 4) * 180 / pi;
x = xout(1, :, 5);
y = xout(1, :, 6);
psi = xout(1, :, 7) * 180 / pi;
n = xout(1, :, 8);
delta = xout(1, :, 9);
u_dot = xout(1, :, 10);
v_dot = xout(1, :, 11);
r_dot = xout(1, :, 12) * 180 / pi;

xte = pout(1, :, 1);
psi_er = pout(1, :, 2) * 180 / pi;
xtetot = pout(1, :, 3);
psi_er_tot = pout(1, :, 4) * 180 / pi;

if strcmpi(add_ts_vessel, 'y')
    x_ts = xout(2, :, 5);
    y_ts = xout(2, :, 6);
    psi_ts = xout(2, :, 7) * 180 / pi;
end
[nominal_time_os, nominal_dist_os, actual_time_os, actual_dist_os] = os.guidance.perf(vessels(1).wp.pos,x,y,3,h,stop_time(1),3);
[nominal_time_ts, nominal_dist_ts, actual_time_ts, actual_dist_ts] = ts.guidance.perf(vessels(2).wp.pos,x_ts,y_ts,3,h,stop_time(2),3);
%% Plots
f2=figure(2);
movegui(f2,'northwest');
% Stop button: stops the loop and closes the window

uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
              'Position', [20 20 60 20], ...
              'Callback', @(src, event) stopAndClose(f2));
set(f2, 'UserData', true);

plot(wp_pos(:,1),wp_pos(:,2),'-*r',LineWidth=1.5)
hold on
plot(x, y, '-b',LineWidth=1.5)
grid, axis('equal'), xlabel('East (x)'), ylabel('North (y)'), title('Ship position')
L = 38.5;%ship_length
B = 5.05;%ship_width
tr = 2;

ship_body = [-L/2, -B/2; L/2, -B/2; L/2, B/2; -L/2, B/2];
ship_nose = [L/2, -B/2;L/2 + tr, 0; L/2, B/2];

%Function to transform the ship vertices
transform_vertices = @(vertices, angle, x, y) (vertices * [cosd(angle), sind(angle); -sind(angle), cosd(angle)]) + [x, y];

% Initial transformation and plotting
transformed_body_os = transform_vertices(ship_body, psi(1), x(1), y(1));
transformed_nose_os = transform_vertices(ship_nose, psi(1), x(1), y(1));
if strcmpi(add_ts_vessel, 'y')
    transformed_body_ts = transform_vertices(ship_body, psi_ts(1), x_ts(1), y_ts(1));
    transformed_nose_ts = transform_vertices(ship_nose, psi_ts(1), x_ts(1), y_ts(1));
end

ship_body_plot_os = fill(transformed_body_os(:,1), transformed_body_os(:,2), 'g');
ship_nose_plot_os = fill(transformed_nose_os(:,1), transformed_nose_os(:,2), 'y');
if strcmpi(add_ts_vessel, 'y')
    ship_body_plot_ts = fill(transformed_body_ts(:,1), transformed_body_ts(:,2), 'g');
    ship_nose_plot_ts = fill(transformed_nose_ts(:,1), transformed_nose_ts(:,2), 'y');
end

for k=2:length(x)-1
    % If the figure has been closed manually
    if ~ishandle(f2)
        break;
    end

    transformed_body_os = transform_vertices(ship_body, psi(k), x(k), y(k));
    transformed_nose_os = transform_vertices(ship_nose, psi(k), x(k), y(k));
    if strcmpi(add_ts_vessel, 'y')
        transformed_body_ts = transform_vertices(ship_body, psi_ts(k), x_ts(k), y_ts(k));
        transformed_nose_ts = transform_vertices(ship_nose, psi_ts(k), x_ts(k), y_ts(k));
    end

    % Update the ship's position
    set(ship_body_plot_os, 'Vertices', transformed_body_os);
    set(ship_nose_plot_os, 'Vertices', transformed_nose_os);
    if strcmpi(add_ts_vessel, 'y')
        set(ship_body_plot_ts, 'Vertices', transformed_body_ts);
        set(ship_nose_plot_ts, 'Vertices', transformed_nose_ts);
    end
    pause(0.01);
    
    % If the Stop button is pressed
    if ~get(f2, 'UserData')
        break;  
    end
end

f3=figure(3);
movegui(f3,'northeast');
subplot(321),plot(t,u,'r'),xlabel('time (s)'),title('u (m/s)'),grid
hold on;
subplot(322),plot(t,v,'r'),xlabel('time (s)'),title('v (m/s)'),grid
subplot(323),plot(t,r,'r'),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(324),plot(t,psi,'r'),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(325),plot(t,delta,'r'),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid 
subplot(326),plot(t,n,'r'),xlabel('time (s)'),title('rpm'),grid

f4=figure(4);
movegui(f4,'southeast');
subplot(211),plot(t,xte),xlabel('time (s)'),title('Cross-track error (m)'),grid
subplot(212),plot(t,psi_er),xlabel('time (s)'),title('Heading error (deg)'),grid
fprintf('Total accumulated cross-track error:%d \n',xtetot(end));
fprintf('Total accumulated heading error:%d \n',psi_er_tot(end));
end

function stopAndClose(figHandle)
        set(figHandle, 'UserData', false);
        %close(figHandle);
end
