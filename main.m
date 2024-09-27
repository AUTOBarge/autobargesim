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
t_f = 2e4; % final simulation time (sec)
h = 0.2; % sample time (sec)

%Create and initialise guidance class object
los = LOSguidance();
wp_speed = 3*ones(length(wp_pos),1);
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
xtetot = 0;
psi_er_tot = 0;
pid_params = struct("K_p",35,"T_i",33,"T_d",22,"psi_d_old",0,"error_old",0);
mpc_params = struct('Ts', 0.2, 'N', 80, 'headingGain', 100, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34);
Flag_cont = input('Select the controller (Type 1 for PID or 2 for MPC): ');    

if Flag_cont == 2
    control=controlClass(Flag_cont,mpc_params);
    mpc_nlp = control.init_mpc();
    args = control.constraintcreator();
    next_guess = control.initial_guess_creator(vertcat(Vessel.sensor_state(3),Vessel.sensor_state(6)), ctrl_last);
elseif Flag_cont == 1
    control=controlClass(Flag_cont,pid_params);
else
    error('Invalid Input. Please run main.m again');
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

    if Flag_cont == 2 % Implement the MPC controller
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
    
    % Calculate the performance indices
    [xte,psi_er,xtetot,psi_er_tot,control] = control.XTECalc(Vessel.sensor_state, chi, wp_pos, wp_idx, xtetot, psi_er_tot);
    
    % Update control action
    ctrl_last = SRSP.ctrl_actual';

    % store data for presentation
    xout(i, :) = [time, Vessel.sensor_state', SRSP.ctrl_actual, Vessel.sensor_state_dot(1:3)'];
    chi_d(i)=chi;
    
    % store the performance indices
    pout(i,:) = [xte,psi_er,xtetot,psi_er_tot];

    %End condition
    x_cur=xout(i, 5);
    y_cur=xout(i, 6);
    distance = norm([x_cur-wp_pos(end,1),y_cur-wp_pos(end,2)],2);
    if distance <3
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
psi_rad = psi* (pi / 180); % Heading angle in radians
n = xout(:, 8);
delta = xout(:, 9);
u_dot = xout(:, 10);
v_dot = xout(:, 11);
r_dot = xout(:, 12) * 180 / pi;

xte = pout(:,1);
psi_er = pout(:,2) * 180 / pi;
xtetot = pout(:,3);
psi_er_tot = pout(:,4) * 180 / pi;
[nominal_time, nominal_dist, actual_time, actual_dist] = los.perf(wp_pos,x,y,3,h,i,3);

% Convert ENU to WGS84
ship_wgs84 =zeros(length(x),2);
for i =1:length(x)
    [lat,lon,h] = enu2geodetic(x(i),y(i),0,lat0,lon0,height,wgs84Ellipsoid);
    ship_wgs84(i,:) = [lat,lon];
end

%% Plots
figure(1)
hold on
lat=ship_wgs84(:,1);
lon=ship_wgs84(:,2);
plot(lon,lat,'-b',LineWidth=1.5)

% Stop button: stops the loop and closes the window
uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
              'Position', [20 20 60 20], ...
              'Callback', @(src, event) stopAndClose(figure(1)));
set(figure(1), 'UserData', true);

%Draw ship
L=38.5;%ship_length
B=5.05;%ship_width
tr=2;
ship_body = [-L/2, -B/2; L/2, -B/2; L/2, B/2; -L/2, B/2];
ship_nose = [L/2, -B/2;L/2 + tr, 0; L/2, B/2];

%Animate ship motion
%lat_ref = lat(1); % Reference latitude
meters_per_deg_lat = 111320;
%meters_per_deg_lon = 111320 * cos(deg2rad(lat_ref));

%Function to transform the ship vertices
%transform_vertices = @(vertices, angle, x, y) (vertices * [cosd(angle), sind(angle); -sind(angle), cosd(angle)]) + [x, y];
transform_vertices_geo = @(vertices, angle, lat, lon) ...
    (vertices * [cos(angle), sin(angle); -sin(angle), cos(angle)] * (1 / meters_per_deg_lat)) + [lon, lat];

% Initial transformation and plotting
transformed_body = transform_vertices_geo(ship_body, psi_rad(1), lat(1), lon(1));
transformed_nose = transform_vertices_geo(ship_nose, psi_rad(1), lat(1), lon(1));

ship_body_plot = fill(transformed_body(:,1), transformed_body(:,2), 'g');
ship_nose_plot = fill(transformed_nose(:,1), transformed_nose(:,2), 'y');

for k=2:length(x)
    % If the figure has been closed manually
    if ~ishandle(figure(1))
        break;
    end
    transformed_body = transform_vertices_geo(ship_body, psi_rad(k), lat(k), lon(k));
    transformed_nose = transform_vertices_geo(ship_nose, psi_rad(k), lat(k), lon(k));
    % Update the ship's position
    set(ship_body_plot, 'XData', transformed_body(:,1), 'YData', transformed_body(:,2));
    set(ship_nose_plot, 'XData', transformed_nose(:,1), 'YData', transformed_nose(:,2));
    pause(0.01);
    
    % If the Stop button is pressed
    if ~get(figure(1), 'UserData')
        break;  
    end
end

f2=figure(2);
movegui(f2,'northwest');
plot(wp_pos(:,1),wp_pos(:,2),'-*r',LineWidth=1.5)
hold on
plot(x, y, '-b',LineWidth=1.5)
grid, axis('equal'), xlabel('East (x)'), ylabel('North (y)'), title('Ship position')
legend('Desired Path with waypoints', 'Actual Path');

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

fprintf('Nominal time:%d \n',nominal_time);
fprintf('Nominal distance:%d \n',nominal_dist);
fprintf('Actual time:%d \n',actual_time);
fprintf('Actual distance:%d \n',actual_dist);
fprintf('Total accumulated cross-track error:%d \n',xtetot(end));
fprintf('Total accumulated heading error:%d \n',psi_er_tot(end));
end
%%
function stopAndClose(figHandle)
        set(figHandle, 'UserData', false);
        %close(figHandle);
end