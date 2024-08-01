classdef actuatorClass
    % ClassName: actuatorClass
    %
    % Description:
    %   This class provides a force model for actuators.
    %
    % Properties:
    %   - ship_dim: Ship dimensions. datatype: structure array.
    %   - env_set: External environment. datatype: structure array.
    %   - prop_params: Propeller force model parameters. datatype: structure array.
    %   - rud_params: Rudder force model parameters. datatype: structure array.
    %   - ctrl_actual: Actual control actions (n, delta). datatype: array (1, 2).
    %   - F_P: Propeller force. datatype: array (3, 1).
    %   - F_R: Rudder force. datatype: array (3, 1).
    %   - tau_act: Total actuation force. datatype: array (3, 1).
    %
    % Methods:
    %   - lowLevelControl:
    %       -- act_response: This function describes the response of the actuators to the control command.
    %   - forceModels:
    %       -- get_prop_force: This function provides a propeller force model.
    %       -- get_rud_force: This function provides a rudder force model.
    %       -- get_act_force: This function combines forces from all actuation devices and produces a total actuation force.
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
        prop_params
        rud_params
        ctrl_actual
        F_R
        F_P
        tau_act
    end

    % Constructor
    methods


        function obj = actuatorClass(ship_dim, prop_params, rud_params)
            % Initialize the object
            if nargin > 0
                obj.ship_dim = ship_dim;

        function obj = actuatorClass(ship_dim, env_set, prop_params, rud_params)
            % Initialize the object
            if nargin > 0
                obj.ship_dim = ship_dim;
                obj.env_set = env_set;

                obj.prop_params = prop_params;
                obj.rud_params = rud_params;
            end

        end

    end

    % lowLevelControl
    methods

        function obj = act_response(obj, ctrl_last, ctrl_command, h)
            %This function describes the response of the actuators to the control command.
            %Output Arguments:
            %- ctrl_actual: Actual control actions.

            n_l = ctrl_last(1);
            delta_l = ctrl_last(2);
            n_c = ctrl_command(1);
            delta_c = ctrl_command(2);
            n_dot = obj.prop_params.n_dot; % rpm/s
            delta_dot = obj.rud_params.delta_dot; % deg/s

            %% Update rpm
            if n_c ~= n_l
                sign_n = sign(n_c - n_l);
                n = n_l + sign_n * n_dot * h;

                if sign_n * (n_c - n) <= 0
                    n = n_c;
                end

            else
                n = n_c;
            end

            %% Update delta
            if delta_c ~= delta_l
                sign_delta = sign(delta_c - delta_l);
                delta = delta_l + sign_delta * delta_dot * h;

                if sign_delta * (delta_c - delta) <= 0
                    delta = delta_c;
                end

            else
                delta = delta_c;
            end

            obj.ctrl_actual = [n delta];

        end

    end

    % forceModels
    methods

        function [J_P, K_T, obj] = get_prop_force(obj, env_set, vel)
        function [J_P, K_T, obj] = get_prop_force(obj, vel)
            %This function provides a propeller force model.
            %Output Arguments:
            %- J_P: Propeller advanced ratio.
            %- K_T: Propeller thrust open water characteristic.
            %- F_P (list): Propeller force matrix.

            %% Load paramters
            scale = obj.ship_dim.scale;
            L = obj.ship_dim.L; % Ship length
            D_P = obj.prop_params.D_P / scale; % Propeller diameter in m
            x_P_dash = obj.prop_params.x_P_dash; % Longitudinal coordinate of propeller position in m
            t_P = obj.prop_params.t_P; % Thrust deduction factor
            w_P0 = obj.prop_params.w_P0; % Wake coefficient at propeller position in straight moving
            k_0 = obj.prop_params.k_0;
            k_1 = obj.prop_params.k_1;
            k_2 = obj.prop_params.k_2;
            rho_water = env_set.rho_water; % Water density in kg/m^3
            rho_water = obj.env_set.rho_water; % Water density in kg/m^3

            %% Propeller force model
            u = vel(1);
            v = vel(2);
            r = vel(3);
            n_P = obj.ctrl_actual(1) / 60; % rpm--rps
            U = sqrt(u ^ 2 + v ^ 2);
            r_dash = r * L / U;

            if U == 0
                r_dash = 0;
            end

            % Calculate hull drift angle at midship
            if u ~= 0
                beta_m = atan2(-v, u);
            else
                beta_m = 0;
            end

            beta_P = beta_m - x_P_dash * r_dash; % Geometrical inflow angle to propeller in maneuvering motions
            w_P = w_P0 * exp(-4 * beta_P ^ 2); % Wake coefficient at propeller position in maneuvering motions
            % Calculate propeller advanced ratio
            if n_P == 0
                J_P = 0;
            else
                J_P = u * (1 - w_P) / (n_P * D_P);
            end

            K_T = k_2 * J_P ^ 2 + k_1 * J_P + k_0; % Propeller thrust open water characteristic
            T = rho_water * n_P ^ 2 * D_P ^ 4 * K_T; % Propeller thrust
            obj.F_P = [(1 - t_P) * T; 0; 0];

        end

        function obj = get_rud_force(obj, env_set, vel, J_P, K_T)
        function obj = get_rud_force(obj, vel, J_P, K_T)
            %This function provides a rudder force model.
            %Output Arguments:
            %- F_R (list): Rudder force matrix.

            %% Load paramters
            scale = obj.ship_dim.scale;
            L = obj.ship_dim.L; % Ship length
            D_P = obj.prop_params.D_P / scale; % Propeller diameter in m
            w_P0 = obj.prop_params.w_P0; % Wake coefficient at propeller position in straight moving
            C_R = obj.rud_params.B_R / scale; % Averaged rudder chord length
            B_R = obj.rud_params.B_R / scale; % Rudder span
            l_R_dash = obj.rud_params.l_R_dash; % Effective longitudinal coordinate of rudder position
            t_R = obj.rud_params.t_R; % Steering resistance deduction factor
            alpha_H = obj.rud_params.alpha_H; % Rudder force increase factor
            gamma_R = obj.rud_params.gamma_R; % Flow straightening coefficient
            epsilon = obj.rud_params.epsilon; % Ratio of wake fraction at rudder position to that at propeller position
            kappa = obj.rud_params.kappa; % An experimental constant for expressing u_R
            x_R_dash = obj.rud_params.x_R_dash; % Longitudinal coordinate of rudder position
            x_H_dash = obj.rud_params.x_H_dash; % Longitudinal coordinate of acting point of the additional lateral force
            rho_water = env_set.rho_water; % Water density in kg/m^3
            rho_water = obj.env_set.rho_water; % Water density in kg/m^3

            %% Ruddr force model
            u = vel(1);
            v = vel(2);
            r = vel(3);
            delta = obj.ctrl_actual(2) * pi / 180;
            U = sqrt(u ^ 2 + v ^ 2);
            r_dash = r * L / U;

            if U == 0
                r_dash = 0;
            end

            % Calculate hull drift angle at midship
            if u ~= 0
                beta_m = atan2(-v, u);
            else
                beta_m = 0;
            end

            beta_R = beta_m - l_R_dash * r_dash; % Drift angle at the rudder position
            v_R = U * gamma_R * beta_R; % Lateral flow speed after passing the propeller
            eta = D_P / B_R; %Ratio of propeller diameter to rudder span

            if J_P ~= 0
                u_R = epsilon * u * (1 - w_P0) * sqrt(eta * (1 + kappa * (sqrt(1 + 8 * K_T / (pi * J_P ^ 2)) - 1)) ^ 2 + (1 - eta)); % Water flow speed towards the rudder
                alpha_R = delta - atan2(v_R, u_R); % Effective rudder in-flow angle
            else
                u_R = 0;
                alpha_R = 0;
            end

            U_R = sqrt(u_R ^ 2 + v_R ^ 2); % Total flow velocity to the rudder

            A_R = B_R * C_R; % Rudder area
            aspect = B_R / C_R; % Rudder aspect ratio
            f_alpha = 6.13 * aspect / (aspect + 2.25); % Rudder lift gradient coefficient
            F_N = 0.5 * rho_water * A_R * U_R ^ 2 * f_alpha * sin(alpha_R); % Rudder force
            obj.F_R = [- (1 - t_R) * F_N * sin(delta);
                       - (1 + alpha_H) * F_N * cos(delta);
                       - (x_R_dash * L + alpha_H * x_H_dash * L) * F_N * cos(delta)];

        end

        function obj = get_act_force(obj)
            %This function combines forces from all actuation devices and produces a total actuation force.
            %Output Arguments:
            %- tau_act (list): Actuator force matrix.

            obj.tau_act = obj.F_P + obj.F_R;

        end

    end

end
