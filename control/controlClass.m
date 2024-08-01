classdef controlClass
% ClassName: controlClass
    %
    % Description:
    %   This class provides the control commands for high-level or low-level vessel control.
    %
    % Properties:
    %   - pid_params: Contains the PID controller gains(K_p, K_i, K_d), 
    % psi_d_old: desired heading angle for next iteration, error_old: heading tracking error for next iteration.
    %   - mpc_params: Contains MPC gains/weight matrices: Q, R.
    %   - Flag_cont: To select the controller type (PID, MPC).
    %   - (To be developed) Flag_act: To select whether the actuator model is considered or not.
    %
    % Methods:
    % -init_mpc: This function implements the high-level controller.
    % -LowLevelPIDCtrl: This method implements the low-level PID controller.
	% -LowLevelMPCCtrl: This method implements the low-level MPC controller.
    % Author:
	% 	Abhishek Dhyani
	% Date:
	%	22/02/2024
	% Version:
	% 	1.0
    
    properties
    pid_params
    mpc_params
    Flag_cont %0=PID,1=MPC
    %Flag_act (1,1) int {mustBePositive} = 0; %0=High-level control only, 1= High+Low-level control
    end

    % Constructor
    methods

        function obj = controlClass(pid_params,mpc_params,Flag_cont)
            % Initialize the object
            if nargin >0 %By default,a PID controller is implemented 
                obj.pid_params = pid_params;
                obj.mpc_params = mpc_params;
                obj.Flag_cont = Flag_cont;
            else
                obj.pid_params = struct("K_p",400,"T_i",10,"T_d",50,"psi_d_old",0,"error_old",0);
                %obj.mpc_params = struct("Q",diag([0,0,0,0,0,0,0,0]),"R",diag([0.0,0.0]));
                obj.mpc_params = [];
                obj.Flag_cont = 0;
            end

        end

    end

    methods
        
		function out = init_mpc(obj,T_mpc,N_mpc)
			import casadi.*
			% states = [ u v r x y psi delta n ]'
            %I simply copy-pasted some lines from your previous code here.
            %You may want to modify it. You can modify the object
            %"mpc_params", and then create casadi variables inside the function. 
			states_linear = SX.sym('states_liner',8,1);
			states_nlinear = SX.sym('states_nliner',8,1);
			inputs = SX.sym('inputs',2,1);
			ode_l = Dynamics_simplified_CA(states_linear,inputs);
			ode_nl = Dynamics_CA(states_nlinear,inputs);
			Dy_l = Function('Dy_l',{states_linear,inputs},{ode_l}); % creating a casadi function for calculation of simplified dynamics
			Dy_nl = Function('Dy_l',{states_nlinear,inputs},{ode_nl}); % creating a casadi function for calculation of dynamics
			%(Define default for T,N)
			T = T_mpc; % Sampling Time 
			N = N_mpc; % Prediction Horizon
			num_states = length(states_linear);
			num_control = length(inputs);% X = [ u v r x y psi delta n ]'
			X = SX.sym('X', num_states, N+1);
			U = SX.sym('U', num_control, N);
			P = SX.sym('P', num_states + num_states, 1);
			g = [];
            st = X(:,1);
            g = [g;st-P(1:num_states)];
			out = 1;
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

        %function [psi_old,error_old,ctrl_command, obj] = HighLevelPIDCtrl(obj,psi_d,psi_old,error_old,h)
                 %ctrl_command=tau_c;
        %end

        function [ctrl_command, obj] = LowLevelMPCCtrl(obj,psi_d,h)
	        %Your MPC SQP/NLP here    
            ctrl_command=[n_c,delta_c];
        end
    end
end


