<<<<<<< Updated upstream:model&actuatorClass/modelClass.m
classdef modelClass
    % ClassName: modelClass
    %
    % Description:
    %   This class provides dynamic models as virtual sensor / control reference model based on provided ship dimensions, environment setting and actuating forces from actuatorClass.
    %
    % Properties:
    %   - ship_dim: Ship dimensions. datatype: structure array.
    %   - env_set: External environment. datatype: structure array.
    %   - dyn_model_params: Parameters in the sensor dynamic model. datatype: structure array.
    %   - ref_model_params: Parameters in the control reference model. datatype: structure array.
    %   - sensor_state: States used in sensor dynamic model. datatype: array (6, 1).
    %   - sensor_state_dot: Output (state_dot) of sensor dynamic model. datatype: array (6, 1).
    %   - ref_state: States used in control reference model. datatype: array (6, 1).
    %   - ref_state_dot: Output (state_dot) of control reference model. datatype: array (6, 1).
    %
    % Methods:
    %   - pramsCalculator:
    %       -- ship_params_calculator: This function calculates model parameters using imperical formulas based on ship dimensions.
    %   - Models:
    %       -- sensor_dynamic_model: This function provides a dynamic model for 3DOF maneuvring motion. It is highly accurate and serves as a virtual sensor.
    %       -- ctrl_reference_model: This function provides reference model for controllers.
    %
    % Author:
    %   Yan-Yun Zhang
    %
    % Date:
    %   2023-11-23
    %
    % Version:
    %   1.0

    properties
        ship_dim
        env_set
        dyn_model_params
        ref_model_params
        sensor_state
        sensor_state_dot
        ref_state
        ref_state_dot
    end

    % Constructor
    methods

        function obj = modelClass(ship_dim, env_set)
            % Initialize the object
            if nargin > 0
                obj.ship_dim = ship_dim;
                obj.env_set = env_set;
            end

        end

    end

    % pramsCalculator
    methods

        function obj = ship_params_calculator(obj)
            %This function calculates model parameters using imperical formulas based on ship dimensions.
            %
            %Updated Arguments:
            %- obj.dyn_model_params (dic): Parameters in the sensor dynamic model.
            %- obj.ref_model_params (dic): Parameters in the control reference model.

            %% Load environment setting
            rho_water = obj.env_set.rho_water; % Water density in kg/m^3
            H_d = obj.env_set.H_d; % Ratio of water depth (h) to ship draught

            %% Load ship dimensions
            scale = obj.ship_dim.scale; % Scale factor
            disp = obj.ship_dim.disp / scale ^ 3; % ship displacement in m^3
            L = obj.ship_dim.L / scale; % Ship length in m
            L_R = obj.ship_dim.L_R / scale; % Ship run length in m
            B = obj.ship_dim.B / scale; % Ship breadth in m
            d = obj.ship_dim.d / scale; % Ship draught in m
            C_b = obj.ship_dim.C_b; % Block coefficient
            C_p = obj.ship_dim.C_p; % Prismatic coefficient
            S = obj.ship_dim.S / scale ^ 2; % Wetted surface area in m^2
            u_0 = obj.ship_dim.u_0 / sqrt(scale); % Ship service speed in m/s
            x_G = obj.ship_dim.x_G / scale; % Gravity point location in m. positive - bow; negative - stern.
            m = disp * rho_water;

            %% Calculate non-dimensional coefficients
            % Inertia and added mass
            m_dash = m / (0.5 * rho_water * L ^ 2 * d);
            I_z = m * (0.2536 * L) ^ 2;
            I_zG = I_z + x_G ^ 2 * m;
            J_z = m * (0.01 * L * (33 - 76.85 * C_b * (1 - 0.784 * C_b) + 3.43 * L / B * (1 - 0.63 * C_b))) ^ 2;
            m_x = 0.05 * m;
            m_x_dash = m_x / (0.5 * rho_water * L ^ 2 * d);
            m_y = m * (0.882 - 0.54 * C_b * (1 - 1.6 * d / B) - 0.156 * (1 - 0.673 * C_b) * L / B + 0.826 * d / B * L / B * (1 - 0.678 * d / B) - 0.638 * C_b * L / B * d / B * (1 - 0.669 * d / B));
            m_y_dash = m_y / (0.5 * rho_water * L ^ 2 * d);

            %% Calculate the resistance
            Re = u_0 * L / 1.306e-6;
            C_F = 0.075 / (log10(Re) - 2) ^ 2;
            k = -0.07 + 0.487118 * (B / L) ^ 1.06806 * (d / L) ^ 0.46106 * (L / L_R) ^ 0.121563 * (L ^ 3 / disp) ^ 0.36486 * (1 - C_p) ^ (-0.604247);
            delta_k = 0.644 * H_d ^ (-1.72); %Form factor correction from Millward (1989)
            C_W = 0.0033;
            C_T = (1 + k + delta_k) * C_F + C_W; % Ct = R/(0.5*rho_water*S*U*2)
            R_dash = C_T * S / L / d;

            %% Calculate surge hydrodynamics
            X_vv_dash = 1.15 * C_b / (L / B) - 0.18;
            X_rr_dash = -0.085 * C_b / (L / B) + 0.008;
            X_vr_dash = -m_y_dash + 1.91 * C_b / (L / B) - 0.08;

            %% Calculate sway hydrodynamics
            Y_v_dash = -0.5 * pi * 2 * d / L - 1.4 * C_b * B / L;
            Y_vvv_dash = -0.185 * L / B - 0.48;
            Y_r_dash = m_dash + m_x_dash - 1.5 * C_b * B / L;
            Y_rrr_dash = -0.051;
            Y_vvr_dash = -0.75;
            Y_vrr_dash = -0.26 * L * (1 - C_b) / B - 0.11;

            %% Calculate yaw hydrodynamics
            N_v_dash = -2 * d / L;
            N_vvv_dash = 0.69 * C_b - 0.66;
            N_r_dash = -0.54 * 2 * d / L + (2 * d / L) ^ 2;
            N_rrr_dash = 0.25 * C_b * B / L - 0.056;
            N_vvr_dash = 1.55 * C_b * B / L - 0.76;
            N_vrr_dash = -0.075 * (1 - C_b) * L / B + 0.098;

            %% Construct outputs
            %dyn_model_names = ['L' 'd' 'rho_water' 'm' 'x_G' 'I_zG' 'm_x' 'm_y' 'J_z' 'R_dash' 'X_vv_dash' 'X_rr_dash' 'X_vr_dash' 'Y_v_dash' 'Y_vvv_dash' 'Y_r_dash' 'Y_rrr_dash' 'Y_vvr_dash' 'Y_vrr_dash' 'N_v_dash' 'N_vvv_dash' 'N_r_dash' 'N_rrr_dash' 'N_vvr_dash' 'N_vrr_dash'];
            %dyn_model_wheels = [L d rho_water m x_G I_zG m_x m_y J_z R_dash X_vv_dash X_rr_dash X_vr_dash Y_v_dash Y_vvv_dash Y_r_dash Y_rrr_dash Y_vvr_dash Y_vrr_dash N_v_dash N_vvv_dash N_r_dash N_rrr_dash N_vvr_dash N_vrr_dash];
            obj.dyn_model_params = struct('L',L,'d',d,'rho_water',rho_water,'m',m,'x_G',x_G,'I_zG',I_zG,'m_x',m_x,'m_y',m_y,'J_z',J_z,'R_dash',R_dash,'X_vv_dash',X_vv_dash,'X_rr_dash',X_rr_dash,'X_vr_dash',X_rr_dash,'Y_v_dash',Y_v_dash,'Y_vvv_dash',Y_vvv_dash,'Y_r_dash',Y_r_dash,'Y_rrr_dash',Y_rrr_dash,'Y_vvr_dash',Y_vvr_dash,'Y_vrr_dash',Y_vrr_dash,'N_v_dash',N_v_dash,'N_vvv_dash',N_vvv_dash,'N_r_dash',N_r_dash,'N_rrr_dash',N_rrr_dash,'N_vvr_dash',N_vvr_dash,'N_vrr_dash',N_vrr_dash);
            %ref_model_names = ['L' 'd' 'rho_water' 'm' 'x_G' 'I_zG' 'm_x' 'm_y' 'J_z' 'R_dash' 'Y_v_dash' 'Y_r_dash' 'N_v_dash' 'N_r_dash'];
            %ref_model_wheels = [L d rho_water m x_G I_zG m_x m_y J_z R_dash Y_v_dash Y_r_dash N_v_dash N_r_dash];
            obj.ref_model_params = struct('L',L,'d',d,'rho_water',rho_water,'m',m,'x_G',x_G,'I_zG',I_zG,'m_x',m_x,'m_y',m_y,'J_z',J_z,'R_dash',R_dash,'Y_v_dash',Y_v_dash,'Y_r_dash',Y_r_dash,'N_v_dash',N_v_dash,'N_r_dash',N_r_dash);

        end

    end

    % Models
    methods

        function obj = sensor_dynamic_model(obj, tau_act)
            %This function provides a dynamic model for 3DOF maneuvring motion. It is highly accurate and serves as a virtual sensor.
            %
            %Output Arguments:
            %- sensor_state_dot (list)

            %% Load states
            u = obj.sensor_state(1);
            v = obj.sensor_state(2);
            r = obj.sensor_state(3); %rad/s
            x = obj.sensor_state(4);
            y = obj.sensor_state(5);
            psi = obj.sensor_state(6);

            %% Load model parameters
            L = obj.dyn_model_params.L;
            d = obj.dyn_model_params.d;
            rho_water = obj.dyn_model_params.rho_water;
            m = obj.dyn_model_params.m;
            x_G = obj.dyn_model_params.x_G;
            I_zG = obj.dyn_model_params.I_zG;
            m_x = obj.dyn_model_params.m_x;
            m_y = obj.dyn_model_params.m_y;
            J_z = obj.dyn_model_params.J_z;
            R_dash = obj.dyn_model_params.R_dash;
            X_vv_dash = obj.dyn_model_params.X_vv_dash;
            X_rr_dash = obj.dyn_model_params.X_rr_dash;
            X_vr_dash = obj.dyn_model_params.X_vr_dash;
            Y_v_dash = obj.dyn_model_params.Y_v_dash;
            Y_vvv_dash = obj.dyn_model_params.Y_vvv_dash;
            Y_r_dash = obj.dyn_model_params.Y_r_dash;
            Y_rrr_dash = obj.dyn_model_params.Y_rrr_dash;
            Y_vvr_dash = obj.dyn_model_params.Y_vvr_dash;
            Y_vrr_dash = obj.dyn_model_params.Y_vrr_dash;
            N_v_dash = obj.dyn_model_params.N_v_dash;
            N_vvv_dash = obj.dyn_model_params.N_vvv_dash;
            N_r_dash = obj.dyn_model_params.N_r_dash;
            N_rrr_dash = obj.dyn_model_params.N_rrr_dash;
            N_vvr_dash = obj.dyn_model_params.N_vvr_dash;
            N_vrr_dash = obj.dyn_model_params.N_vrr_dash;

            %% Non-dimensionalize
            U = sqrt(u ^ 2 + v ^ 2);
            v_dash = v / U;
            r_dash = r * L / U;
            if U == 0
                v_dash = 0;
                r_dash = 0;
            end
            F_cal = 0.5 * rho_water * L * d * U ^ 2;
            N_cal = 0.5 * rho_water * L ^ 2 * d * U ^ 2;

            %% Mass matrix
            M = [m + m_x 0 0
                 0 m + m_y x_G * m
                 0 x_G * m I_zG + J_z];

            %% Coriolis matrix
            C = [(m + m_y) * v * r + m * x_G * r ^ 2
                 - (m + m_x) * u * r
                 - m * x_G * u * r + (m_x - m_y) * u * v];

            %% Damping matrix
            D = [
                 F_cal * (-R_dash + X_vv_dash * (v_dash ^ 2) + X_vr_dash * v_dash * r_dash + X_rr_dash * r_dash ^ 2)
                 F_cal * (Y_v_dash * v_dash + Y_r_dash * r_dash + Y_vvv_dash * v_dash ^ 3 + Y_vvr_dash * v_dash ^ 2 * r_dash + Y_vrr_dash * v_dash * r_dash ^ 2 + Y_rrr_dash * r_dash ^ 3)
                 N_cal * (N_v_dash * v_dash + N_r_dash * r_dash + N_vvv_dash * v_dash ^ 3 + N_vvr_dash * v_dash ^ 2 * r_dash + N_vrr_dash * v_dash * r_dash ^ 2 + N_rrr_dash * r_dash ^ 3)
                 ];

            %%
            vel_dot = M \ (tau_act + D + C);
            obj.sensor_state_dot = [vel_dot
                                    cos(psi) * u - sin(psi) * v
                                    sin(psi) * u + cos(psi) * v
                                    r
                                    ];

        end

        function obj = control_ref_model(obj, tau_act)
            %This function provides reference model for controllers.
            %
            %Output Arguments:
            %- ref_state_dot (list)

            %% Load states
            u = obj.ref_state(1);
            v = obj.ref_state(2);
            r = obj.ref_state(3); %rad/s
            x = obj.ref_state(4);
            y = obj.ref_state(5);
            psi = obj.ref_state(6);

            %% Load model parameters
            L = obj.ref_model_params.L;
            d = obj.ref_model_params.d;
            rho_water = obj.ref_model_params.rho_water;
            m = obj.ref_model_params.m;
            x_G = obj.ref_model_params.x_G;
            I_zG = obj.ref_model_params.I_zG;
            m_x = obj.ref_model_params.m_x;
            m_y = obj.ref_model_params.m_y;
            J_z = obj.ref_model_params.J_z;
            R_dash = obj.ref_model_params.R_dash;
            Y_v_dash = obj.ref_model_params.Y_v_dash;
            Y_r_dash = obj.ref_model_params.Y_r_dash;
            N_v_dash = obj.ref_model_params.N_v_dash;
            N_r_dash = obj.ref_model_params.N_r_dash;

            %% Non-dimensionalize
            U = sqrt(u ^ 2 + v ^ 2);
            v_dash = v / U;
            r_dash = r * L / U;
            if U == 0
                v_dash = 0;
                r_dash = 0;
            end
            F_cal = 0.5 * rho_water * L * d * U ^ 2;
            N_cal = 0.5 * rho_water * L ^ 2 * d * U ^ 2;

            %% Mass matrix
            M = [m + m_x 0 0
                 0 m + m_y x_G * m
                 0 x_G * m I_zG + J_z];

            %% Coriolis matrix
            C = [(m + m_y) * v * r + m * x_G * r ^ 2
                 - (m + m_x) * u * r
                 - m * x_G * u * r + (m_x - m_y) * u * v];

            %% Damping matrix
            D = [
                 - F_cal * R_dash
                 F_cal * (Y_v_dash * v_dash + Y_r_dash * r_dash)
                 N_cal * (N_v_dash * v_dash + N_r_dash * r_dash)
                 ];

            %%
            vel_dot = M \ (tau_act + D + C);
            obj.ref_state_dot = [vel_dot
                                 cos(psi) * u - sin(psi) * v
                                 sin(psi) * u + cos(psi) * v
                                 r
                                 ];

        end

    end

end
=======
classdef modelClass
    % ClassName: modelClass
    %
    % Description:
    %   This class provides dynamic models as virtual sensor / control reference model based on provided ship dimensions, environment setting and actuating forces from actuatorClass.
    %
    % Properties:
    %   - ship_dim: Ship dimensions. datatype: structure array.
    %   - dyn_model_params: Parameters in the sensor dynamic model. datatype: structure array.
    %   - ref_model_params: Parameters in the control reference model. datatype: structure array.
    %   - sensor_state: States used in sensor dynamic model. datatype: array (6, 1).
    %   - sensor_state_dot: Output (state_dot) of sensor dynamic model. datatype: array (6, 1).
    %   - sensor_vel_relative: Relative velocity over ground used in sensor dynamic model. datatype: array (3, 1).
    %   - ref_state: States used in control reference model. datatype: array (6, 1).
    %   - ref_state_dot: Output (state_dot) of control reference model. datatype: array (6, 1).
    %
    % Methods:
    %   - pramsCalculator:
    %       -- ship_params_calculator: This function calculates model parameters using imperical formulas based on ship dimensions.
    %   - Models:
    %       -- sensor_dynamic_model: This function provides a dynamic model for 3DOF maneuvring motion. It is highly accurate and serves as a virtual sensor.
    %       -- ctrl_reference_model: This function provides a reference model for controllers (Fossen form with linear damping).
    %
    % Author:
    %   Yan-Yun Zhang
    %
    % Date:
    %   2023-11-23
    %
    % Version:
    %   1.0

    properties
        ship_dim
        dyn_model_params
        ref_model_params
        sensor_state
        sensor_state_dot
        sensor_vel_relative
        ref_state
        ref_state_dot
    end

    % Constructor
    methods

        function obj = modelClass(ship_dim)
            % Initialize the object
            if nargin > 0
                obj.ship_dim = ship_dim;
            end

        end

    end

    % pramsCalculator
    methods

        function obj = ship_params_calculator(obj, env_set)
            %This function calculates model parameters using imperical formulas based on ship dimensions.
            %
            %Output Arguments:
            %- obj.dyn_model_params (structure array): Parameters in the sensor dynamic model.
            %- obj.ref_model_params (structure array): Parameters in the control reference model.

            %% Load environment setting
            rho_water = env_set.rho_water; % Water density in kg/m^3
            H = env_set.H; % Water depth in m

            %% Load ship dimensions
            scale = obj.ship_dim.scale; % Scale factor
            disp = obj.ship_dim.disp / scale ^ 3; % ship displacement in m^3
            L = obj.ship_dim.L / scale; % Ship length in m
            L_R = obj.ship_dim.L_R / scale; % Ship run length in m
            B = obj.ship_dim.B / scale; % Ship breadth in m
            d = obj.ship_dim.d / scale; % Ship draught in m
            C_b = obj.ship_dim.C_b; % Block coefficient
            C_p = obj.ship_dim.C_p; % Prismatic coefficient
            S = obj.ship_dim.S / scale ^ 2; % Wetted surface area in m^2
            u_0 = obj.ship_dim.u_0 / sqrt(scale); % Ship service speed in m/s
            x_G = obj.ship_dim.x_G / scale; % Gravity point location in m. positive - bow; negative - stern.
            m = disp * rho_water;

            %% Calculate non-dimensional coefficients
            % Inertia and added mass
            m_dash = m / (0.5 * rho_water * L ^ 2 * d);
            I_z = m * (0.2536 * L) ^ 2;
            I_zG = I_z + x_G ^ 2 * m;
            J_z = m * (0.01 * L * (33 - 76.85 * C_b * (1 - 0.784 * C_b) + 3.43 * L / B * (1 - 0.63 * C_b))) ^ 2;
            m_x = 0.05 * m;
            m_x_dash = m_x / (0.5 * rho_water * L ^ 2 * d);
            m_y = m * (0.882 - 0.54 * C_b * (1 - 1.6 * d / B) - 0.156 * (1 - 0.673 * C_b) * L / B + 0.826 * d / B * L / B * (1 - 0.678 * d / B) - 0.638 * C_b * L / B * d / B * (1 - 0.669 * d / B));
            m_y_dash = m_y / (0.5 * rho_water * L ^ 2 * d);

            %% Calculate the resistance
            Re = u_0 * L / 1.306e-6;
            C_F = 0.075 / (log10(Re) - 2) ^ 2;
            k = -0.07 + 0.487118 * (B / L) ^ 1.06806 * (d / L) ^ 0.46106 * (L / L_R) ^ 0.121563 * (L ^ 3 / disp) ^ 0.36486 * (1 - C_p) ^ (-0.604247);
            delta_k = 0.644 * H / d ^ (-1.72); %Form factor correction from Millward (1989)
            C_W = 0.0033;
            C_T = (1 + k + delta_k) * C_F + C_W; % Ct = R/(0.5*rho_water*S*U*2)
            R_dash = C_T * S / L / d;

            %% Calculate surge hydrodynamics
            X_vv_dash = 1.15 * C_b / (L / B) - 0.18;
            X_rr_dash = -0.085 * C_b / (L / B) + 0.008;
            X_vr_dash = -m_y_dash + 1.91 * C_b / (L / B) - 0.08;

            %% Calculate sway hydrodynamics
            Y_v_dash = -0.5 * pi * 2 * d / L - 1.4 * C_b * B / L;
            Y_vvv_dash = -0.185 * L / B - 0.48;
            Y_r_dash = m_dash + m_x_dash - 1.5 * C_b * B / L;
            Y_rrr_dash = -0.051;
            Y_vvr_dash = -0.75;
            Y_vrr_dash = -0.26 * L * (1 - C_b) / B - 0.11;

            %% Calculate yaw hydrodynamics
            N_v_dash = -2 * d / L;
            N_vvv_dash = 0.69 * C_b - 0.66;
            N_r_dash = -0.54 * 2 * d / L + (2 * d / L) ^ 2;
            N_rrr_dash = 0.25 * C_b * B / L - 0.056;
            N_vvr_dash = 1.55 * C_b * B / L - 0.76;
            N_vrr_dash = -0.075 * (1 - C_b) * L / B + 0.098;

            %% Construct outputs
            obj.dyn_model_params = struct('L', L, 'd', d, 'rho_water', rho_water, 'm', m, 'x_G', x_G, 'I_zG', I_zG, 'm_x', m_x, 'm_y', m_y, 'J_z', J_z, 'R_dash', R_dash, 'X_vv_dash', X_vv_dash, 'X_rr_dash', X_rr_dash, 'X_vr_dash', X_vr_dash, 'Y_v_dash', Y_v_dash, 'Y_vvv_dash', Y_vvv_dash, 'Y_r_dash', Y_r_dash, 'Y_rrr_dash', Y_rrr_dash, 'Y_vvr_dash', Y_vvr_dash, 'Y_vrr_dash', Y_vrr_dash, 'N_v_dash', N_v_dash, 'N_vvv_dash', N_vvv_dash, 'N_r_dash', N_r_dash, 'N_rrr_dash', N_rrr_dash, 'N_vvr_dash', N_vvr_dash, 'N_vrr_dash', N_vrr_dash);
            obj.ref_model_params = struct('L', L, 'd', d, 'rho_water', rho_water, 'm', m, 'x_G', x_G, 'I_zG', I_zG, 'm_x', m_x, 'm_y', m_y, 'J_z', J_z, 'R_dash', R_dash, 'Y_v_dash', Y_v_dash, 'Y_r_dash', Y_r_dash, 'N_v_dash', N_v_dash, 'N_r_dash', N_r_dash);

        end

    end

    % Models
    methods

        function obj = sensor_dynamic_model(obj, Act, env_set)
            %This function provides a dynamic model for 3DOF maneuvring motion. It is highly accurate and serves as a virtual sensor.
            %
            %Output Arguments:
            %- sensor_state_dot (list)

            %% Load environment setting
            V_c = env_set.V_c; % Water density in kg/m^3
            beta_c = env_set.beta_c; % Water depth in m

            %% Load states
            u = obj.sensor_state(1); % Surge speed over ground in m/s
            v = obj.sensor_state(2); % Sway speed over ground in m/s
            r = obj.sensor_state(3); % Yaw rate in rad/s
            x = obj.sensor_state(4); % X position in m
            y = obj.sensor_state(5); % Y position in m
            psi = obj.sensor_state(6); % Heading in rad

            %% Current effect
            u_r = u - V_c * cos(beta_c - psi); % Surge speed over water in m/s
            v_r = v - V_c * sin(beta_c - psi); % Sway speed over water in m/s
            obj.sensor_vel_relative(1) = u_r;
            obj.sensor_vel_relative(2) = v_r;
            obj.sensor_vel_relative(3) = r;

            %% Load model parameters
            L = obj.dyn_model_params.L;
            d = obj.dyn_model_params.d;
            rho_water = obj.dyn_model_params.rho_water;
            m = obj.dyn_model_params.m;
            x_G = obj.dyn_model_params.x_G;
            I_zG = obj.dyn_model_params.I_zG;
            m_x = obj.dyn_model_params.m_x;
            m_y = obj.dyn_model_params.m_y;
            J_z = obj.dyn_model_params.J_z;
            R_dash = obj.dyn_model_params.R_dash;
            X_vv_dash = obj.dyn_model_params.X_vv_dash;
            X_rr_dash = obj.dyn_model_params.X_rr_dash;
            X_vr_dash = obj.dyn_model_params.X_vr_dash;
            Y_v_dash = obj.dyn_model_params.Y_v_dash;
            Y_vvv_dash = obj.dyn_model_params.Y_vvv_dash;
            Y_r_dash = obj.dyn_model_params.Y_r_dash;
            Y_rrr_dash = obj.dyn_model_params.Y_rrr_dash;
            Y_vvr_dash = obj.dyn_model_params.Y_vvr_dash;
            Y_vrr_dash = obj.dyn_model_params.Y_vrr_dash;
            N_v_dash = obj.dyn_model_params.N_v_dash;
            N_vvv_dash = obj.dyn_model_params.N_vvv_dash;
            N_r_dash = obj.dyn_model_params.N_r_dash;
            N_rrr_dash = obj.dyn_model_params.N_rrr_dash;
            N_vvr_dash = obj.dyn_model_params.N_vvr_dash;
            N_vrr_dash = obj.dyn_model_params.N_vrr_dash;

            %% Non-dimensionalize
            U = sqrt(u_r ^ 2 + v_r ^ 2);

            if U == 0
                v_dash = 0;
                r_dash = 0;
            else
                v_dash = v_r / U;
                r_dash = r * L / U;
            end

            F_cal = 0.5 * rho_water * L * d * U ^ 2;
            N_cal = 0.5 * rho_water * L ^ 2 * d * U ^ 2;

            %% Mass matrix
            M = [m + m_x 0 0
                 0 m + m_y x_G * m
                 0 x_G * m I_zG + J_z];

            %% Coriolis matrix
            C = [(m + m_y) * v_r * r + m * x_G * r ^ 2
                 - (m + m_x) * u_r * r
                 - m * x_G * u_r * r + (m_x - m_y) * u_r * v_r];

            %% Damping matrix
            D = [
                 F_cal * (-R_dash + X_vv_dash * (v_dash ^ 2) + X_vr_dash * v_dash * r_dash + X_rr_dash * r_dash ^ 2)
                 F_cal * (Y_v_dash * v_dash + Y_r_dash * r_dash + Y_vvv_dash * v_dash ^ 3 + Y_vvr_dash * v_dash ^ 2 * r_dash + Y_vrr_dash * v_dash * r_dash ^ 2 + Y_rrr_dash * r_dash ^ 3)
                 N_cal * (N_v_dash * v_dash + N_r_dash * r_dash + N_vvv_dash * v_dash ^ 3 + N_vvr_dash * v_dash ^ 2 * r_dash + N_vrr_dash * v_dash * r_dash ^ 2 + N_rrr_dash * r_dash ^ 3)
                 ];

            %% Actuation force
            [J_P, K_T, Act] = Act.get_prop_force(env_set, obj.sensor_vel_relative);
            Act = Act.get_rud_force(env_set, obj.sensor_vel_relative, J_P, K_T);
            Act = Act.get_act_force();
            tau_act = Act.tau_act;

            %%
            vel_dot = M \ (tau_act + D + C);
            obj.sensor_state_dot = [vel_dot
                                    cos(psi) * u - sin(psi) * v
                                    sin(psi) * u + cos(psi) * v
                                    r
                                    ];

        end

        function obj = control_ref_model(obj, tau_act)
            %This function provides reference model for controllers.
            %
            %Output Arguments:
            %- ref_state_dot (list)

            %% Load states
            u = obj.ref_state(1);
            v = obj.ref_state(2);
            r = obj.ref_state(3); %rad/s
            x = obj.ref_state(4);
            y = obj.ref_state(5);
            psi = obj.ref_state(6);

            %% Load model parameters
            L = obj.ref_model_params.L;
            d = obj.ref_model_params.d;
            rho_water = obj.ref_model_params.rho_water;
            m = obj.ref_model_params.m;
            x_G = obj.ref_model_params.x_G;
            I_zG = obj.ref_model_params.I_zG;
            m_x = obj.ref_model_params.m_x;
            m_y = obj.ref_model_params.m_y;
            J_z = obj.ref_model_params.J_z;
            R_dash = obj.ref_model_params.R_dash;
            Y_v_dash = obj.ref_model_params.Y_v_dash;
            Y_r_dash = obj.ref_model_params.Y_r_dash;
            N_v_dash = obj.ref_model_params.N_v_dash;
            N_r_dash = obj.ref_model_params.N_r_dash;

            %% Non-dimensionalize
            U = sqrt(u ^ 2 + v ^ 2);
            v_dash = v / U;
            r_dash = r * L / U;

            if U == 0
                v_dash = 0;
                r_dash = 0;
            end

            F_cal = 0.5 * rho_water * L * d * U ^ 2;
            N_cal = 0.5 * rho_water * L ^ 2 * d * U ^ 2;

            %% Mass matrix
            M = [m + m_x 0 0
                 0 m + m_y x_G * m
                 0 x_G * m I_zG + J_z];

            %% Coriolis matrix
            C = [(m + m_y) * v * r + m * x_G * r ^ 2
                 - (m + m_x) * u * r
                 - m * x_G * u * r + (m_x - m_y) * u * v];

            %% Damping matrix
            D = [
                 - F_cal * R_dash
                 F_cal * (Y_v_dash * v_dash + Y_r_dash * r_dash)
                 N_cal * (N_v_dash * v_dash + N_r_dash * r_dash)
                 ];

            %%
            vel_dot = M \ (tau_act + D + C);
            obj.ref_state_dot = [vel_dot
                                 cos(psi) * u - sin(psi) * v
                                 sin(psi) * u + cos(psi) * v
                                 r
                                 ];

        end

    end

end
>>>>>>> Stashed changes:model&actuator/modelClass.m
