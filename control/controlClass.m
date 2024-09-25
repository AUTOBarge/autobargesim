classdef controlClass
% ClassName: controlClass
    %
    % Description:
    %   This class provides the control commands for high-level or low-level vessel control.
    %
    % Properties:
    %   - num_st and num_ct are reserved for the designer and represents number of states and number of controls in the MPC model, respectively.
    %   - pid_params: Contains the PID controller gains(K_p, K_i, K_d), 
    %   - mpc_params: mpc_params = struct('Ts', sampling time, 'N', Prediction horizon, 'headingGain', Q in the cost function, 'rudderGain', R in the cost function, 'max_iter', maximum iteration of the MPC solver, 'deltaMAX', maximum allowed rudder angle)
    %   - Flag_cont: To select between the PID and MPC controller
    %   type . (Set as 1 for PID and 2 for MPC controller).
    %   - (To be developed) Flag_act: To select whether the actuator model is considered or not.
    %
    % Methods:
    % -init_mpc: This function implements the high-level controller.
    % -initial_guess_creator: Providing it with initial states and control values this method will create the initial guess for the MPC solver
    % -constraintcreator: This method forms the constraint vectors
    % -LowLevelPIDCtrl: This method implements the low-level PID controller.
	% -LowLevelMPCCtrl: This method implements the low-level MPC controller.
    % Author:
	% 	Abhishek Dhyani
    %   AmirReza Haqshenas M.
	% Date:
	%	29/02/2024
	% Version:
	% 	1.0
    
    properties
    num_st
    num_ct
    pid_params
    mpc_params
    Flag_cont %1=PID,2=MPC
    %Flag_act (1,1) int {mustBePositive} = 0; %0=High-level control only, 1= High+Low-level control
    end

    % Constructor
    methods

        function obj = controlClass(Flag_cont,varargin)
            % Initialize the object
            if Flag_cont==1 && nargin==1
                obj.pid_params = struct("K_p",400,"T_i",10,"T_d",50,"psi_d_old",0,"error_old",0);
                obj.Flag_cont = Flag_cont;
            elseif Flag_cont==1 && nargin==2
                pid_params=varargin{1,1};
                obj.pid_params = pid_params;
                obj.Flag_cont = Flag_cont;
            elseif Flag_cont==2 && nargin==1
                obj.mpc_params = struct('Ts', 0.2, 'N', 80, 'headingGain', 100, 'rudderGain', 0.0009, 'max_iter', 200, 'deltaMAX', 34);
                obj.Flag_cont = Flag_cont;
                obj.num_ct = 1;%mpc_model.num_controls;
                obj.num_st = 2;%mpc_model.num_states;
            elseif Flag_cont==2 && nargin==2
                obj.mpc_params = varargin{1,1};
                obj.Flag_cont = Flag_cont;
                obj.num_ct = 1;%mpc_model.num_controls;
                obj.num_st = 2;%mpc_model.num_states;
            else
                error('Error: Input provided is incorrect. Please refer to the documentation for the control module');
            end

        end

    end

    methods
        
        function nlp = init_mpc(obj)
           
            import casadi.*
            num_states = obj.num_st;
            num_controls = obj.num_ct;

            states = SX.sym('states', num_states, 1);
            controls = SX.sym('controls', num_controls, 1);

         
            [ode_l1, ode_l2] = NomotoModel(states, controls);
            ode_l = vertcat(ode_l1,ode_l2);
            f_dy_l = Function('f_dy_l',{states, controls},{ode_l},{'x0','u0'},{'ODE'});
            
            T_mpc = obj.mpc_params.Ts;
            N_mpc = obj.mpc_params.N;

            % X_MPC = [r psi]' 

            X_mpc = SX.sym('X_mpc', num_states, N_mpc+1);
            U_mpc = SX.sym('U_mpc', num_controls, N_mpc);

            P_mpc = SX.sym('P_mpc', 4,1); % initial heading / ROT and desired heading / ROT

            g_mpc = [];

            obj_mpc = 0;

            Q = zeros(2,2);
            Q(1,1) = obj.mpc_params.headingGain;
            Q(2,2) = obj.mpc_params.headingGain/1000;

            R = zeros(1,1);
            R(1,1) = obj.mpc_params.rudderGain;

            g_mpc = vertcat(g_mpc, X_mpc(:,1)-P_mpc(1:2));

            for k_mpc = 1:N_mpc
                obj_mpc = obj_mpc + ...
                    (X_mpc(:,k_mpc) - P_mpc(3:4))'*Q*(X_mpc(:,k_mpc) - P_mpc(3:4)) + ...
                    U_mpc(:,k_mpc)' * R * U_mpc(:,k_mpc);

                st_next = X_mpc(: , k_mpc+1);
                f_val = f_dy_l(X_mpc(:,k_mpc), U_mpc(:,k_mpc));
                st_next_euler = X_mpc(:,k_mpc) + T_mpc*f_val;
                g_mpc = vertcat(g_mpc,st_next - st_next_euler);
            end

            OPT_variables = [reshape(X_mpc, num_states*(N_mpc+1),1);
                             reshape(U_mpc, num_controls*N_mpc,1)];

            nlp_prob = struct('f',obj_mpc, 'x', OPT_variables, 'g', g_mpc, 'p', P_mpc);

            opts = struct;
            opts.ipopt.max_iter = obj.mpc_params.max_iter;
            opts.ipopt.print_level = 0; %0,3
            opts.print_time = 0;
            opts.verbose = 0;
            opts.ipopt.acceptable_tol = 1e-8; % 1e-8;
            opts.ipopt.acceptable_obj_change_tol = 1e-6;

            nlp =nlpsol('solver', 'ipopt', nlp_prob, opts);
            

        end
        
        function initial_guess = initial_guess_creator(obj, states, ctrl_last)
            N_mpc = obj.mpc_params.N;
            num_states = obj.num_st;
            num_controls = obj.num_ct;

            
            x0 = states;
            X0 = repmat(x0, 1, N_mpc + 1);
            
            u0 = zeros(num_controls, N_mpc);
            u0(:) = ctrl_last(2);
            initial_guess = struct('X0', X0, 'u0',u0);

        end

        function args = constraintcreator(obj)

            N_mpc = obj.mpc_params.N;
            num_states = obj.num_st;
            num_controls = obj.num_ct;

            dMax = obj.mpc_params.deltaMAX;
            % rpmMAX = obj.mpc_params.RPM; % this is reserved for the next-gen control

            args =struct;
            args.lbg(1:(num_states)*(N_mpc+1)) = 0; %-1e-20;
            args.ubg(1:(num_states)*(N_mpc+1)) = 0; %1e-20;

            args.lbx(1:num_states*(N_mpc+1),1) = -inf;
            args.ubx(1:num_states*(N_mpc+1),1) = inf;

            % args.lbx(num_states*(N_mpc+1)+1:2:num_states*(N_mpc+1) + num_controls*N_mpc) = rpmMAX; % 650; % min rpm % this is reserved for the next-gen control
            % args.ubx(num_states*(N_mpc+1)+1:2:num_states*(N_mpc+1) + num_controls*N_mpc) = rpmMAX; % 650; % max rpm % this is reserved for the next-gen control

            args.lbx(num_states*(N_mpc+1)+1:1:num_states*(N_mpc+1) + num_controls*N_mpc) = -dMax; % min delta
            args.ubx(num_states*(N_mpc+1)+1:1:num_states*(N_mpc+1) + num_controls*N_mpc) = dMax;  % max delta


        end

        function [ctrl_command,obj] = LowLevelPIDCtrl(obj,psi_d,r,psi,h)
                
                psi_d_old = obj.pid_params.psi_d_old;
                error_old = obj.pid_params.error_old;
                error_psi = (psi-psi_d);			
				r_d = (psi_d-psi_d_old)/h;
				sum_error = error_psi+error_old;
                K_p = obj.pid_params.K_p;
                T_d = obj.pid_params.T_d;
                T_i = obj.pid_params.T_i;
				delta_c = -K_p*(error_psi+ T_d*(r-r_d)+(1/T_i)*sum_error); %Command rudder angle
				n_c = 350; 
                obj.pid_params.psi_d_old=psi_d;
				obj.pid_params.error_old=error_psi;
                ctrl_command=[n_c,delta_c];

        end	

        function [ctrl_command, next_guess,obj] = LowLevelMPCCtrl(obj, states, psi_d, r_d, args, initial_guess, mpc_nlp)
            
            N_mpc = obj.mpc_params.N;
            num_states = obj.num_st;
            num_controls = obj.num_ct;

            X0 = initial_guess.X0;
            u0 = initial_guess.u0;
      
            args.p = vertcat(states(3),states(6),r_d, psi_d);
            args.x0 = vertcat(reshape(X0, num_states*(N_mpc+1),1), ...
                              reshape(u0, num_controls*N_mpc, 1));

            sol = mpc_nlp('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
            disp(sol.f)
            XX = full(sol.x(1:num_states*(N_mpc+1)));
            UU = full(sol.x(num_states*(N_mpc+1)+1:end));
            UC = reshape(UU, num_controls, N_mpc);

    
            ctrl_command = UC(:,1); % taking out the first iteration to serve as control input
            % using the corrent solve to be the intial guess for the next iteration
            next_guess = struct;
            next_guess.X0 = reshape(XX,num_states, N_mpc+1); 
            next_guess.u0 = horzcat(UC(:,2:end), UC(:,end)); % augmenting the last index of u since the first of is used

        end
        
        function [xte,psi_er,xtetot,psi_er_tot,obj] = XTECalc(obj, states, psi_d, wp_pos, wp_idx,xtetot,psi_er_tot)
            
            xp=states(4);
            yp=states(5);
            psi=states(6);
            
            if wp_idx >= length(wp_pos)
                wp_vec=wp_pos(wp_idx,:)-wp_pos(wp_idx-1,:);
            else
                wp_vec=wp_pos(wp_idx+1,:)-wp_pos(wp_idx,:);
            end
            pos_vec=[xp yp]-wp_pos(wp_idx,:);
            crossprod=cross([wp_vec 0],[pos_vec 0]);
            %Cross track error (XTE)
            xte=norm(crossprod)/norm(wp_vec);
            %Heading angle error
            psi_er=mod(abs(psi_d-psi),pi);
            %Accumulated XTE and heading angle errors
            xtetot=xtetot+xte;
            psi_er_tot=psi_er_tot+psi_er;
            
        end
end

end
