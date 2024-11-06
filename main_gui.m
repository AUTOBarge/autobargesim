function main_gui()
    clc; close all;
    shapeFileDirectory = '';
    defaultStart = '';
    defaultEnd = '';
    defaultStart2 = '';
    defaultEnd2 = '';
    %% GUI Figure and Logo
    f = figure('Name', 'AUTOBargeSim', 'Position', [200, 35, 500, 750]);
    set(f, 'Resize', 'off');
    imgFolder = append(pwd,'\img');
    logo = imread(fullfile(imgFolder,'Logo.png'));
    %uicontrol('Style', 'text', 'Position', [50, 640, 400, 40], 'String', 'AUTOBargeSim: MATLAB toolbox for the design and analysis of the GNC System for autonomous inland vessels.', 'HorizontalAlignment', 'center');
    axesHandle = axes('Parent', f, 'Position', [0.25,0.7,0.5,0.3]);
    imshow(logo, 'Parent', axesHandle);
    %% Shape Files Section
    uicontrol('Style', 'text', 'Position', [50, 520, 400, 20], 'String', 'Select the map area:', 'HorizontalAlignment', 'left');
    areaPopup = uicontrol('Style', 'popupmenu', 'String', {'Albert canal', 'Leuven area', 'Specify directory with shape files ...'}, 'Position', [50, 500, 400, 25]);
    %% Start and End Points Section
    %bgPoints = uipanel('Title', 'Own Vessel Start and End Points', 'FontSize', 8,'Position', [0.1 0.59 0.8 0.07]);
    uicontrol('Style', 'text', 'Position', [50, 475, 400, 20], 'String', 'Own Vessel Start and End Points', 'HorizontalAlignment', 'left');
    bgPoints = uibuttongroup('Position', [0.1 0.59 0.8 0.05]);
    uicontrol(bgPoints, 'Style', 'radiobutton', 'String', 'Select on the map', 'Position', [10 10 250 15]);
    uicontrol(bgPoints, 'Style', 'radiobutton', 'String', 'Use default points', 'Position', [270 10 150 15]);
    %% Controller Parameters Selection
    % Controller selection
    uicontrol('Style', 'text', 'Position', [50, 410, 400, 20], 'String', 'Select the controller type:', 'HorizontalAlignment', 'left');
    controllerPopup = uicontrol('Style', 'popupmenu', 'String', {'Select ...','PID', 'MPC'}, 'Position', [50, 390, 400, 25], 'Callback', @controllerCallback);

    % Default or Custom Parameters for Controllers
    uicontrol('Style', 'text', 'Position', [50, 370, 400, 20], 'String', 'Choose default values or manually enter controller parameters:', 'HorizontalAlignment', 'left');
    bgControllerParams = uibuttongroup('Position', [0.1, 0.45, 0.8, 0.05], 'SelectionChangedFcn', @controllerParamsSelectionCallback);
    uicontrol(bgControllerParams, 'Style', 'radiobutton', 'String', 'Default', 'Position', [10 5 100 15]);
    uicontrol(bgControllerParams, 'Style', 'radiobutton', 'String', 'Manual', 'Position', [150 5 100 15]);

    % Controller Parameters Input
    paramPanel = uipanel('Title', 'Controller Parameters', 'FontSize', 8, 'Position', [0.1, 0.3, 0.8, 0.15]);
    param1Label = uicontrol(paramPanel, 'Style', 'text', 'Position', [2, 50, 150, 20], 'String', '');
    param1Input = uicontrol(paramPanel, 'Style', 'slider', 'Min', 80, 'Max', 300, 'Value', 80, 'Position', [160, 50, 200, 20], 'Visible', 'off');
    addlistener(param1Input, 'Value', 'PostSet', @(src, event) updateSliderLabel2());
    param1Labelval = uicontrol(paramPanel, 'Style', 'text', 'Position', [160, 30, 200, 20], 'String', '');
    param2Label = uicontrol(paramPanel, 'Style', 'text', 'Position', [-10, 10, 150, 20], 'String', '');
    param2Input = uicontrol(paramPanel, 'Style', 'edit', 'Position', [100, 10, 70, 25], 'Visible', 'off');
    param3Label = uicontrol(paramPanel, 'Style', 'text', 'Position', [200, 10, 100, 20], 'String', '');
    param3Input = uicontrol(paramPanel, 'Style', 'edit', 'Position', [290, 10, 70, 25], 'Visible', 'off');
    
    function controllerCallback(~, ~)
        updateControllerParams();
    end

    function controllerParamsSelectionCallback(~, ~)
        updateControllerParams();
    end

    function updateControllerParams(~, ~)
    controllerType = controllerPopup.Value;
    paramMode = get(get(bgControllerParams, 'SelectedObject'), 'String');

      if controllerType == 2 && strcmp(paramMode, 'Manual') % PID
        set(param1Label, 'String', 'K_p:', 'Visible', 'on');
        set(param1Input, 'Style', 'edit', 'String', '120', 'Visible', 'on', 'Enable', 'on');
        set(param2Label, 'String', 'T_i:', 'Visible', 'on');
        set(param2Input, 'String', '20', 'Visible', 'on', 'Enable', 'on');
        set(param3Label, 'String', 'T_d:', 'Visible', 'on');
        set(param3Input, 'String', '10', 'Visible', 'on', 'Enable', 'on');
      elseif controllerType == 3 && strcmp(paramMode, 'Manual')  % MPC
        set(param1Label, 'String', 'Prediction Horizon:', 'Visible', 'on');
        set(param1Input, 'Style', 'slider', 'Min', 80, 'Max', 300, 'Value', 80, 'Visible', 'on', 'Enable', 'on');
        set(param1Labelval, 'String', 'Value: 80', 'Visible', 'on');
        set(param2Label, 'String', 'Heading Gain:', 'Visible', 'on');
        set(param2Input, 'String', '100', 'Visible', 'on', 'Enable', 'on');
        set(param3Label, 'String', 'Rudder Gain:', 'Visible', 'on');
        set(param3Input, 'String', '0.0009', 'Visible', 'on', 'Enable', 'on');
      elseif strcmp(paramMode, 'Default') % Default mode
            if controllerType == 2 % PID default
                set(param1Label, 'String', 'K_p:', 'Visible', 'on');
                set(param1Input, 'Style', 'edit', 'String', '120', 'Visible', 'on', 'Enable', 'off');
                set(param2Label, 'String', 'T_i:', 'Visible', 'on');
                set(param2Input, 'String', '20', 'Visible', 'on', 'Enable', 'off');
                set(param3Label, 'String', 'T_d:', 'Visible', 'on');
                set(param3Input, 'String', '10', 'Visible', 'on', 'Enable', 'off');
            elseif controllerType == 3 % MPC default
                set(param1Label, 'String', 'Prediction Horizon:', 'Visible', 'on');
                set(param1Input, 'Style', 'slider', 'Min', 80, 'Max', 300, 'Value', 80, 'Visible', 'on', 'Enable', 'off');
                set(param1Labelval, 'String', 'Value: 80', 'Visible', 'on');
                set(param2Label, 'String', 'Heading Gain:', 'Visible', 'on');
                set(param2Input, 'String', '100', 'Visible', 'on', 'Enable', 'off');
                set(param3Label, 'String', 'Rudder Gain:', 'Visible', 'on');
                set(param3Input, 'String', '0.0009', 'Visible', 'on', 'Enable', 'off');
            end
        else
            set(param1Label, 'Visible', 'off');
            set(param1Input, 'Visible', 'off');
            set(param2Label, 'Visible', 'off');
            set(param2Input, 'Visible', 'off');
            set(param3Label, 'Visible', 'off');
            set(param3Input, 'Visible', 'off');
      end
    end
    %% Guidance Parameters Selection
    % Pass Angle Threshold slider
    uicontrol('Style', 'text', 'Position', [50, 200, 400, 20], 'String', 'Set Pass Angle Threshold:', 'HorizontalAlignment', 'left');
    passAngleSlider = uicontrol('Style', 'slider', 'Min', 50, 'Max', 150, 'Value', 100, 'Position', [50, 180, 400, 20]);
    addlistener(passAngleSlider, 'Value', 'PreSet', @(src, event) updateSliderLabel());

    % Slider value label
    sliderValueLabel = uicontrol('Style', 'text', 'Position', [200, 160, 100, 20], 'String', 'Value: 100', 'HorizontalAlignment', 'center');
    
    function updateSliderLabel()
    sliderValue = get(passAngleSlider, 'Value');
    set(sliderValueLabel, 'String', sprintf('Value: %.0f', sliderValue));
    end
    
    function updateSliderLabel2()
    sliderValue = get(param1Input, 'Value');
    set(param1Labelval, 'String', sprintf('Value: %.0f', sliderValue));
    end
    %% Target Vessel Plotting Section
    uicontrol('Style', 'text', 'Position', [50, 145, 400, 20], 'String', 'Plot the target vessel?', 'HorizontalAlignment', 'left');
    bgTargetVessel = uibuttongroup('Position', [0.1 0.15 0.8 0.05], 'SelectionChangedFcn', @targetVesselCallback, 'SelectedObject', []);
    uicontrol(bgTargetVessel, 'Style', 'radiobutton', 'String', 'Yes', 'Position', [10 10 50 15]);
    rbNo=uicontrol(bgTargetVessel, 'Style', 'radiobutton', 'String', 'No', 'Position', [100 10 50 15]);
    bgTargetVessel.SelectedObject = rbNo;

    %% Target Vessel Start and End Points
    bgPointstext=uicontrol('Style', 'text', 'Position', [50, 90, 400, 20], 'String', 'Target Vessel Start and End Points', 'HorizontalAlignment', 'left', 'Visible', 'off');
    bgPoints2 = uibuttongroup('Position', [0.1 0.07 0.8 0.05], 'Visible', 'off');
    uicontrol(bgPoints2, 'Style', 'radiobutton', 'String', 'Select on the map', 'Position', [10 10 150 15], 'Tag', 'TargetMapSelect');
    uicontrol(bgPoints2, 'Style', 'radiobutton', 'String', 'Use default points', 'Position', [270 10 150 15], 'Tag', 'TargetDefaultPoints');
    
    %% Target Vessel Visibility Control
    function targetVesselCallback(~, event)
        selectedOption = event.NewValue.String;
        if strcmp(selectedOption, 'Yes')
            set(bgPoints2, 'Visible', 'on');
            set(bgPointstext, 'Visible', 'on');
        else
            set(bgPoints2, 'Visible', 'off');
            set(bgPointstext, 'Visible', 'off');
        end
    end

    %% Execute Button
    executeButton = uicontrol('Style', 'pushbutton', 'String', 'Execute', 'Position', [200, 10, 100, 40], 'Callback', @executeCallback);

    % Function to handle the execution
    function executeCallback(~, ~)
        
        areaChoice = get(areaPopup, 'Value');
        useDefaultPoints = get(get(bgPoints, 'SelectedObject'), 'String');
        useDefaultPoints2 = get(get(bgPoints2, 'SelectedObject'), 'String');
        controllerType = get(controllerPopup, 'Value');
        useDefaultParams = get(get(bgControllerParams, 'SelectedObject'), 'String');
        add_ts_vessel = get(get(bgTargetVessel, 'SelectedObject'), 'String');
        % Get Pass Angle Threshold from slider
        passAngleThreshold = get(passAngleSlider, 'Value');
        % Determine shape file directory and default points
            switch areaChoice
                case 1
                    shapeFileDirectory = fullfile(pwd, 'maps', 'demo', '.shp', 'Albert canal');
                    defaultStart = [4.72662, 51.1819];  % Default values for Albert canal
                    defaultEnd = [4.84824, 51.1625];
                    defaultStart2 = [4.74202, 51.18]; 
                    defaultEnd2 = [4.72593, 51.182];
                case 2
                    shapeFileDirectory = fullfile(pwd, 'maps', 'demo', '.shp', 'Leuven area');
                    defaultStart = [4.48571, 51.0806];  % Default values for Leuven area
                    defaultEnd = [4.52222, 51.106];
                    defaultStart2 = [4.50013, 51.089]; 
                    defaultEnd2 = [4.48463, 51.0799];
                otherwise
                    shapeFileDirectory = uigetdir(pwd, 'Select the directory containing shape files');
                    defaultStart = [];  % No defaults for custom files
                    defaultEnd = [];
                    defaultStart2 = []; 
                    defaultEnd2 = [];
            end
         % Collect controller parameters if custom is selected
    if strcmp(useDefaultParams, 'Manual') && controllerType ~= 1
        if controllerType == 2
            param1 = str2double(get(param1Input, 'String'));
        else
            param1 = round(get(param1Input, 'Value'));
        end
        param2 = str2double(get(param2Input, 'String'));
        param3 = str2double(get(param3Input, 'String'));
    else
        % Use default values
        if controllerType == 1 || controllerType == 2 % PID default
            param1 = 120; % K_p
            param2 = 20; % T_i
            param3 = 10; % T_d
        elseif controllerType == 3 % MPC default
            param1 = 80; % Prediction Horizon
            param2 = 100; % Heading Gain
            param3 = 0.0009; % Rudder Gain
        end
    end


    % Display collected data
    fprintf('Using maps from package: %s\n', shapeFileDirectory);
    fprintf('Pass Angle Threshold: %.2f\n', passAngleThreshold);
    
    if controllerType == 2
        fprintf('Controller: PID\n');
        fprintf('K_p: %.2f, T_i: %.2f, T_d: %.2f\n', param1, param2, param3);
    elseif controllerType == 3
        fprintf('Controller: MPC\n');
        fprintf('Prediction Horizon: %.2f, Heading Gain: %.2f, Rudder Gain: %.4f\n', param1, param2, param3);
    else
        fprintf('Controller type not specified.\n The simulation will proceed with PID controller with default values\n');
        fprintf('K_p: %.2f, T_i: %.2f, T_d: %.2f\n', param1, param2, param3);
    end
       
        % Create a 'process' class object and plot the area
        desirename = ["depare", "bridge", "wtwaxs", "lndare"];
        warning('off', 'MATLAB:polyshape:repairedBySimplify');
        mapFig = figure(); %New figure for the map
        process = maps.processor(desirename, shapeFileDirectory);
        process.plot();
        
        % Determine start and end points
        if strcmpi(useDefaultPoints, 'Use default points')
            if isempty(defaultStart) || isempty(defaultEnd)|| isempty(defaultStart2)|| isempty(defaultEnd2)
                    error('No default start and end points exist for custom maps. Please re-run and select the points on the map.')
            end
            startPoint = defaultStart;
            endPoint = defaultEnd;
        else
            % Prompt user to provide start and end points
            msg1=msgbox('Please select the start and end points for the vessel from the map', 'Select Points', 'help');
            uiwait(msg1);
            figure(mapFig);
            st = drawpoint;
            en = drawpoint;
            start_long = st.Position(1);
            start_lat = st.Position(2);
            end_long = en.Position(1);
            end_lat = en.Position(2);
            startPoint = [start_long, start_lat];
            endPoint = [end_long, end_lat];
        end
        
        %% Vessel 1
        vessel1 = [];

        % Create a 'plan' class object and provide it the start and end points, plot the path
        plan = maps.planner(process.pgon_memory);
        plan = plan.plan_path([startPoint(1), startPoint(2)], [endPoint(1), endPoint(2)]);
        plan.plot_path(2);
        
        %%
        wp_wgs84 = plan.path_points;
        wgs84 = wgs84Ellipsoid;
        lon0 = wp_wgs84(1,1);
        lat0 = wp_wgs84(1,2);
        wp_pos = zeros(length(wp_wgs84),2);
        height = 0;
        for i =1:length(wp_wgs84)
          [xEast,yNorth,~] = geodetic2enu(wp_wgs84(i,2),wp_wgs84(i,1),height,lat0,lon0,height,wgs84);
          wp_pos(i,:) = [xEast,yNorth];
        end
        
        % Initialisation
        t_f = 2e4; % final simulation time (sec)
        h = 0.2; % sample time (sec)
        
        % Create and initialise guidance class object
        vessel1.guidance = LOSguidance();
        vessel1.wp.pos = wp_pos;
        vessel1.wp.speed = 100*ones(length(wp_pos),1);
        vessel1.wp.idx = 1;
        initial_state = [0 0 0 0 0 0]'; % Initial state [u v r x y psi] in column
        vessel1.chi_d_prev = atan2(wp_pos(2, 2)-wp_pos(1, 2),wp_pos(2, 1)-wp_pos(1, 1));
        [chi, ~] = vessel1.guidance.compute_LOSRef(vessel1.wp.pos, vessel1.wp.speed, initial_state', vessel1.wp.idx, 1, vessel1.chi_d_prev);
        initial_state = [0 0 0 wp_pos(1, 1) wp_pos(1, 2) chi]'; % Initial state [u v r x y psi] in column
        vessel1.chi_d_prev = chi;
        % Create and initialise model and actuator class objects
        ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
        env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", 0);
        prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
        rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
        vessel1.model = modelClass(ship_dim);
        vessel1.model = vessel1.model.ship_params_calculator(env_set, rud_params);
        vessel1.model.sensor_state = initial_state;
        
        % Create and initialise actuator class objects
        vessel1.actuators = actuatorClass(ship_dim, prop_params, rud_params);
        L = vessel1.model.ship_dim.L;
        K_dash = vessel1.model.KTindex.K_dash;
        T_dash = vessel1.model.KTindex.T_dash;
        % Create and initialise control class object
        vessel1.control.output = [200; 0]; % Initial control
        vessel1.control.param = [];
        vessel1.err.xtetot = 0;
        vessel1.err.psi_er_tot = 0;
        
        % Determine the controller type
        if controllerType == 1 || controllerType == 2 % PID
            Flag_cont = 1; 
            pid_params = struct("K_p",param1,"T_i",param2,"T_d",param3,"psi_d_old",0,"error_old",0);
            vessel1.control.model = controlClass(Flag_cont,pid_params);
        elseif controllerType == 3 % MPC
            Flag_cont = 2; 
            mpc_params = struct('Ts', 0.2, 'N', param1, 'headingGain', param2, 'rudderGain', param3, 'max_iter', 200, 'deltaMAX', 34, 'K_dash', K_dash, 'T_dash', T_dash, 'L', L);
            vessel1.control.model=controlClass(Flag_cont,mpc_params);
            vessel1.control.param.mpc_nlp = vessel1.control.model.init_mpc();
            vessel1.control.param.args = vessel1.control.model.constraintcreator();
            vessel1.control.param.next_guess = vessel1.control.model.initial_guess_creator(vertcat(vessel1.model.sensor_state(3), vessel1.model.sensor_state(6)), vessel1.control.output);
        end
        
        if strcmpi(add_ts_vessel, 'Yes')
            % Initialize colision avoidance
            vessel1.colav.alg = sbmpc(10, h);
            vessel1.colav.param = [1; 0]; % chi_m, U_m
        end
        
%% Vessel 2
vessel2 = [];

% Prompt user to provide start and end points
if strcmpi(add_ts_vessel, 'Yes')
    
    if strcmpi(useDefaultPoints2, 'Use default points')
            startPoint = defaultStart2;
            endPoint = defaultEnd2;
    else
         % Prompt user to provide start and end points
         msg2=msgbox('Please select the start and end points for the target vessel from the map', 'Select Points', 'help');
         uiwait(msg2);
         figure(mapFig);
         st=drawpoint;
         en=drawpoint;
         start_long = st.Position(1);
         start_lat= st.Position(2);
         end_long = en.Position(1);
         end_lat = en.Position(2);  
         startPoint = [start_long, start_lat];
         endPoint = [end_long, end_lat];
    end
   
    % Create a 'plan' class object and provide it the start and end points, plot the path
    plan = maps.planner(process.pgon_memory);
    plan = plan.plan_path([startPoint(1), startPoint(2)], [endPoint(1), endPoint(2)]);
    plan.plot_path(2);
    
    %
    wp_wgs84 = plan.path_points;
    wgs84 = wgs84Ellipsoid;
    wp_pos = zeros(length(wp_wgs84),2);
    height = 0;
    for i =1:length(wp_wgs84)
        [xEast,yNorth,~] = geodetic2enu(wp_wgs84(i,2),wp_wgs84(i,1),height,lat0,lon0,height,wgs84);
        wp_pos(i,:) = [xEast,yNorth];
    end
    
    % Create and initialise guidance class object
    vessel2.guidance = LOSguidance();
    vessel2.wp.pos = wp_pos;
    vessel2.wp.speed = 100*ones(length(wp_pos),1);
    vessel2.wp.idx = 1;
    vessel2.chi_d_prev = atan2(wp_pos(2, 2)-wp_pos(1, 2),wp_pos(2, 1)-wp_pos(1, 1));
    [chi, ~] = vessel2.guidance.compute_LOSRef(vessel2.wp.pos, vessel2.wp.speed, [0 0 0 0 0 0], vessel2.wp.idx, 1, vessel2.chi_d_prev);
    initial_state = [0 0 0 wp_pos(1, 1) wp_pos(1, 2) chi]'; % Initial state [u v r x y psi] in column
    vessel2.chi_d_prev = chi;
    % Create and initialise model class objects
    ship_dim = struct("scale", 1, "disp", 505, "L", 38.5, "L_R", 3.85, "B", 5.05, "d", 2.8, "C_b", 0.94, "C_p", 0.94, "S", 386.2, "u_0", 4.1, "x_G", 0);
    env_set = struct("rho_water", 1000, "H", 5, "V_c", 0.1, "beta_c", 0);
    prop_params = struct("D_P", 1.2, "x_P_dash", -0.5, "t_P", 0.249, "w_P0", 0.493, "k_0", 0.6, "k_1", -0.3, "k_2", -0.5, "n_dot", 50);
    rud_params = struct("C_R", 3.2, "B_R", 2.8, "l_R_dash", -0.71, "t_R", 0.387, "alpha_H", 0.312, "gamma_R", 0.395, "epsilon", 1.09, "kappa", 0.5, "x_R_dash", -0.5, "x_H_dash", -0.464, "delta_dot", 5);
    vessel2.model = modelClass(ship_dim);
    vessel2.model = vessel2.model.ship_params_calculator(env_set, rud_params);
    vessel2.model.sensor_state = initial_state;

    % Create and initialise actuator class objects
    vessel2.actuators = actuatorClass(ship_dim, prop_params, rud_params);
    L = vessel2.model.ship_dim.L;
    K_dash = vessel2.model.KTindex.K_dash;
    T_dash = vessel2.model.KTindex.T_dash;

    % Create and initialise control class object
    vessel2.control.output = [200; 0]; % Initial control
    vessel2.control.param = [];
    vessel2.err.xtetot = 0;
    vessel2.err.psi_er_tot = 0;
    if Flag_cont == 2
        mpc_params = struct('Ts', 0.2, 'N', param1, 'headingGain', param2, 'rudderGain', param3, 'max_iter', 200, 'deltaMAX', 34, 'K_dash', K_dash, 'T_dash', T_dash, 'L', L);
        vessel2.control.model=controlClass(Flag_cont,mpc_params);
        vessel2.control.param.mpc_nlp = vessel2.control.model.init_mpc();
        vessel2.control.param.args = vessel2.control.model.constraintcreator();
        vessel2.control.param.next_guess = vessel2.control.model.initial_guess_creator(vertcat(vessel2.model.sensor_state(3), vessel2.model.sensor_state(6)), vessel2.control.output);
    else
        pid_params = struct("K_p",param1,"T_i",param2,"T_d",param3,"psi_d_old",0,"error_old",0);
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
            if i==1
                fprintf('\nPlease wait. This may take a while...\n')
            end
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
                [chi, U] = os.guidance.compute_LOSRef(os.wp.pos, os.wp.speed, os.model.sensor_state', os.wp.idx, 1, os.chi_d_prev);
                os.chi_d_prev = chi;
                if strcmpi(add_ts_vessel, 'Yes')
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
            d_threshold = norm([x_cur-os.wp.pos(end,1),y_cur-os.wp.pos(end,2)],2);
            if d_threshold < 20
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
        psi_rad = xout(1, :, 7);
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
        
        if strcmpi(add_ts_vessel, 'Yes')
            x_ts = xout(2, :, 5);
            y_ts = xout(2, :, 6);
            psi_ts_rad = xout(2, :, 7);
            psi_ts = xout(2, :, 7) * 180 / pi;
        end
        [nominal_time_os, nominal_dist_os, actual_time_os, actual_dist_os] = os.guidance.perf(vessels(1).wp.pos,x,y,3,h,stop_time(1),3);
        if strcmpi(add_ts_vessel, 'Yes')
        [nominal_time_ts, nominal_dist_ts, actual_time_ts, actual_dist_ts] = ts.guidance.perf(vessels(2).wp.pos,x_ts,y_ts,3,h,stop_time(2),3);
        end
        % Convert ENU to WGS84
        ship_wgs84 =zeros(length(x),2);
        for i =1:length(x)
            [lat,lon,~] = enu2geodetic(x(i),y(i),0,lat0,lon0,height,wgs84Ellipsoid);
            ship_wgs84(i,:) = [lat,lon];
        end
        
        if strcmpi(add_ts_vessel, 'Yes')
            tship_wgs84 =zeros(length(x_ts),2);
            for i =1:length(x_ts)
                [lat2,lon2,~] = enu2geodetic(x_ts(i),y_ts(i),0,lat0,lon0,height,wgs84Ellipsoid);
                tship_wgs84(i,:) = [lat2,lon2];
            end
        end

%% Plots
mapFig;
hold on
lat=ship_wgs84(:,1);
lon=ship_wgs84(:,2);
plot(lon,lat,'-b',LineWidth=1.5)

if strcmpi(add_ts_vessel, 'Yes')
    lat2=tship_wgs84(:,1);
    lon2=tship_wgs84(:,2);
    plot(lon2,lat2,'-g',LineWidth=1.5)
end

% Stop button: stops the loop and closes the window
uicontrol('Style', 'pushbutton', 'String', '<HTML> <B>Stop</B>', ...
              'Position', [20 20 60 20], ...
              'Callback', @(src, event) stopAndClose(mapFig));
set(mapFig, 'UserData', true);

%Draw ship
L=38.5;%ship_length
B=5.05;%ship_width
tr=2;
ship_body = [-L/2, -B/2; L/2, -B/2; L/2, B/2; -L/2, B/2];
ship_nose = [L/2, -B/2;L/2 + tr, 0; L/2, B/2];

%Animate ship motion
lat_ref = lat(1); % Reference latitude
meters_per_deg_lat = 111320;
meters_per_deg_lon = 111320 * cos(deg2rad(lat_ref));

%Function to transform the ship vertices
%transform_vertices = @(vertices, angle, x, y) (vertices * [cosd(angle), sind(angle); -sind(angle), cosd(angle)]) + [x, y];
transform_vertices_geo = @(vertices, angle, lat, lon) ...
    (vertices * [cos(angle), sin(angle); -sin(angle), cos(angle)] * diag([1/meters_per_deg_lon, 1/meters_per_deg_lat])) + [lon, lat];

% Initial transformation and plotting
transformed_body_os = transform_vertices_geo(ship_body, psi_rad(1), lat(1), lon(1));
transformed_nose_os = transform_vertices_geo(ship_nose, psi_rad(1), lat(1), lon(1));

if strcmpi(add_ts_vessel, 'Yes')
    transformed_body_ts = transform_vertices_geo(ship_body, psi_ts_rad(1), lat2(1), lon2(1));
    transformed_nose_ts = transform_vertices_geo(ship_nose, psi_ts_rad(1), lat2(1), lon2(1));
end

ship_body_plot_os = fill(transformed_body_os(:,1), transformed_body_os(:,2), 'g');
ship_nose_plot_os = fill(transformed_nose_os(:,1), transformed_nose_os(:,2), 'y');

if strcmpi(add_ts_vessel, 'Yes')
    ship_body_plot_ts = fill(transformed_body_ts(:,1), transformed_body_ts(:,2), 'g');
    ship_nose_plot_ts = fill(transformed_nose_ts(:,1), transformed_nose_ts(:,2), 'y');
end

for k=2:length(x)
    % If the figure has been closed manually
    if ~ishandle(mapFig)
        break;
    end
    transformed_body_os = transform_vertices_geo(ship_body, psi_rad(k), lat(k), lon(k));
    transformed_nose_os = transform_vertices_geo(ship_nose, psi_rad(k), lat(k), lon(k));
    
      if strcmpi(add_ts_vessel, 'Yes')
        transformed_body_ts = transform_vertices_geo(ship_body, psi_ts_rad(k), lat2(k), lon2(k));
        transformed_nose_ts = transform_vertices_geo(ship_nose, psi_ts_rad(k), lat2(k), lon2(k));
      end

    % Update the ship's position
    set(ship_body_plot_os, 'XData', transformed_body_os(:,1), 'YData', transformed_body_os(:,2));
    set(ship_nose_plot_os, 'XData', transformed_nose_os(:,1), 'YData', transformed_nose_os(:,2));
    if strcmpi(add_ts_vessel, 'Yes')
        set(ship_body_plot_ts, 'XData', transformed_body_ts(:,1), 'YData', transformed_body_ts(:,2));
        set(ship_nose_plot_ts, 'XData', transformed_nose_ts(:,1), 'YData', transformed_nose_ts(:,2));
    end
    pause(0.01);
    
    % If the Stop button is pressed
    if ~get(mapFig, 'UserData')
        break;  
    end
end

f3=figure(3);
movegui(f3,'northwest');
plot(vessel1.wp.pos(:,1),vessel1.wp.pos(:,2),'-*r',LineWidth=1.5)
hold on
plot(x, y, '-b',LineWidth=1.5)
grid, axis('equal'), xlabel('East (x)'), ylabel('North (y)'), title('Ship position')

if strcmpi(add_ts_vessel, 'Yes')
    plot(vessel2.wp.pos(:,1),vessel2.wp.pos(:,2),'-*m',LineWidth=1.5)
    plot(x_ts, y_ts, '-g',LineWidth=1.5)
    legend('Own ships Desired Path with Waypoints', 'Own ships Actual Path',...
        'Target ships Desired Path with Waypoints', 'Target ships Actual Path');
else
legend('Desired Path with waypoints', 'Actual Path');
end

f4=figure(4);
movegui(f4,'northeast');
subplot(321),plot(t,u,'r'),xlabel('time (s)'),title('u (m/s)'),grid
hold on;
subplot(322),plot(t,v,'r'),xlabel('time (s)'),title('v (m/s)'),grid
subplot(323),plot(t,r,'r'),xlabel('time (s)'),title('yaw rate r (deg/s)'),grid
subplot(324),plot(t,psi,'r'),xlabel('time (s)'),title('yaw angle \psi (deg)'),grid
subplot(325),plot(t,delta,'r'),xlabel('time (s)'),title('rudder angle \delta (deg)'),grid 
subplot(326),plot(t,n,'r'),xlabel('time (s)'),title('rpm'),grid

f5=figure(5);
movegui(f5,'southeast');
subplot(211),plot(t,xte),xlabel('time (s)'),title('Cross-track error (m)'),grid
subplot(212),plot(t,psi_er),xlabel('time (s)'),title('Heading error (deg)'),grid

fprintf('Own Ship Nominal time:%d \n',nominal_time_os);
fprintf('Own Ship Nominal distance:%d \n',nominal_dist_os);
fprintf('Own Ship Actual time:%d \n',actual_time_os);
fprintf('Own Ship Actual distance:%d \n',actual_dist_os);

if strcmpi(add_ts_vessel, 'Yes')
fprintf('Target Ship Nominal time:%d \n',nominal_time_ts);
fprintf('Target Ship Nominal distance:%d \n',nominal_dist_ts);
fprintf('Target Ship Actual time:%d \n',actual_time_ts);
fprintf('Target Ship Actual distance:%d \n',actual_dist_ts);
end

fprintf('Total accumulated cross-track error:%d \n',xtetot(end));
fprintf('Total accumulated heading error:%d \n',psi_er_tot(end));
end
end

function stopAndClose(figHandle)
        set(figHandle, 'UserData', false);
        %close(figHandle);
end