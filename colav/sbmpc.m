classdef sbmpc < colav
    properties (Constant, Hidden)
        default_D_CLOSE_ = 20 % distance for an nearby obstacle [m]
        default_D_SAFE_ = 10 % distance of safety zone [m]
        default_D_INIT_ = sbmpc.default_D_CLOSE_ * 3

        default_KAPPA_ = 7.0 % cost function parameter
        default_PHI_AH_ = deg2rad(68.5) % colregs angle - ahead [deg]
        default_PHI_OT_ = deg2rad(68.5) % colregs angle - overtaken [deg]
        default_PHI_HO_ = deg2rad(22.5) % colregs angle - head on [deg]
        default_PHI_CR_ = deg2rad(68.5) % colregs angle - crossing [deg]

        default_K_COLL_ = 0.5 % cost scaling factor
        default_P_ = 1.0 % weights the importance of time until the event of collision occurs
        default_Q_ = 4.0 % exponent to satisfy colregs rule 16

        default_K_P_ = 2.5 % cost function parameter
        default_K_CHI_SB_ = 1.3 % cost function parameter
        default_K_CHI_P_ = 30 % cost function parameter
        default_K_DP_ = 2.0 % cost function parameter
        default_K_DCHI_SB_ = 1.2 % cost function parameter
        default_K_DCHI_P_ = 0.9 % cost function parameter

        default_Chi_ca_ = deg2rad([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]) % control behaviors - course offset [deg]
        default_U_ca_ = [0, 0.5, 1.0]  % control behaviors - speed factor
    end

    properties 
        tuning_param % tuning parameters
    end
    
    methods
        function sbmpcObj = sbmpc(T, dt, varargin)
            % Creates an instance of the sbmpc class.
            % sbmpcObj = sbmpc(T, dt, varargin)
            %
            % INPUTS:
            %       T -> Prediction time horizon of the algorithm in
            %            seconds.
            %            scalar | double
            %       dt -> Prediction sample time of the algorithm in
            %             seconds.
            %             scalar | double
            %       optional -> Name-value arguments for the tuning
            %                   parameters of the SBMPC algorithm.
            %

            p = inputParser;
            p.KeepUnmatched = true;
            % tuning parameters
            addParameter(p, 'D_CLOSE_', sbmpc.default_D_CLOSE_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'D_SAFE_', sbmpc.default_D_SAFE_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'D_INIT_', sbmpc.default_D_INIT_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'KAPPA_', sbmpc.default_KAPPA_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_AH_', sbmpc.default_PHI_AH_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_OT_', sbmpc.default_PHI_OT_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_HO_', sbmpc.default_PHI_HO_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_CR_', sbmpc.default_PHI_CR_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_COLL_', sbmpc.default_K_COLL_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'P_', sbmpc.default_P_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'Q_', sbmpc.default_Q_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_P_', sbmpc.default_K_P_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_CHI_SB_', sbmpc.default_K_CHI_SB_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_CHI_P_', sbmpc.default_K_CHI_P_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DP_', sbmpc.default_K_DP_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DCHI_SB_', sbmpc.default_K_DCHI_SB_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DCHI_P_', sbmpc.default_K_DCHI_P_, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'Chi_ca_', sbmpc.default_Chi_ca_, @(x) validateattributes(x, {'double'}, {'vector'}));
            addParameter(p, 'U_ca_', sbmpc.default_U_ca_, @(x) validateattributes(x, {'double'}, {'vector'}));
            
            % internal vessel model parameters
            addParameter(p, 'T_chi', sbmpc.default_T_chi);
            addParameter(p, 'T_U', sbmpc.default_T_U);
            parse(p, varargin{:});

            sbmpcObj = sbmpcObj@colav(T, dt, 'T_chi', p.Results.T_chi, 'T_U', p.Results.T_U);
            sbmpcObj.tuning_param = struct(...
                'D_CLOSE_', p.Results.D_CLOSE_,...
                'D_SAFE_', p.Results.D_SAFE_,...
                'D_INIT_', p.Results.D_INIT_,...
                'KAPPA_', p.Results.KAPPA_,...
                'PHI_AH_', p.Results.PHI_AH_,...
                'PHI_OT_', p.Results.PHI_OT_,...
                'PHI_HO_', p.Results.PHI_HO_,...
                'PHI_CR_', p.Results.PHI_CR_,...
                'K_COLL_', p.Results.K_COLL_,...
                'P_', p.Results.P_,...
                'Q_', p.Results.Q_,...
                'K_P_', p.Results.K_P_,...
                'K_CHI_SB_', p.Results.K_CHI_SB_,...
                'K_CHI_P_', p.Results.K_CHI_P_,...
                'K_DP_', p.Results.K_DP_,...
                'K_DCHI_SB_', p.Results.K_DCHI_SB_,...
                'K_DCHI_P_', p.Results.K_DCHI_P_,...
                'Chi_ca_', p.Results.Chi_ca_,...
                'U_ca_', p.Results.U_ca_);
        end
        function [chi_c, U_c, chi_m, U_m] = run_sbmpc(self, x, chi_d, U_d, chi_m_last, U_m_last, x_ts)
            % Calculate the modifications for the course angle and speed
            % commands from the guidance controller.
            %
            % [chi_c, U_c, chi_m, U_m] = run_sbmpc(x, chi_d, U_d, chi_m_last, U_m_last, x_ts)
            %
            % INPUTS:
            %       x -> own-ship state [u, v, r, x, y, psi]
            %            size (1 x 6) | vector | double
            %       chi_d -> course angle command from the guidance
            %                controller
            %                scalar | double
            %       U_d -> speed command from the guidance controller
            %              scalar | double
            %       chi_m_last -> course modification from the SBMPC at the
            %                     previous time step.
            %                     scalar | double
            %       U_m_last -> speed modification from the SBMPC at the
            %                   previous time step
            %                   scalar | double
            %       x_ts -> target-ship states:
            %               [x_n, y_n, chi_n, U_n] where n is the vessel ID
            %               size (: x 4) | matrix | double
            %

            validateattributes(x, {'double'}, {'vector', 'ncols', 6})
            validateattributes(chi_d, {'double'}, {'scalar'})
            validateattributes(U_d, {'double'}, {'scalar'})
            validateattributes(chi_m_last, {'double'}, {'scalar'})
            validateattributes(U_m_last, {'double'}, {'scalar'})
            validateattributes(x_ts, {'double'}, {'2d', 'ncols', 6})

            state = [x(4), x(5), x(6), sqrt(x(1)^2+x(2)^2)]; % x = [x, y, chi, U]

            n_samples = utils.getNumSamples(self.dt, self.T);
            Chi_ca_ = self.tuning_param.Chi_ca_;
            U_ca_ = self.tuning_param.U_ca_;

            cost_k_min = inf;
            for i = 1:length(Chi_ca_)
                for j = 1:length(U_ca_)
                    cost_i_max = -1;
                    
                    ownship_traj = self.calcVesselTraj(state, [chi_d + Chi_ca_(i), U_d * U_ca_(j)], self.dt, n_samples);
                   
                    num_ts = numel(x_ts(:, 1));
                    for idx = 1:num_ts
                        state_ts = [x_ts(idx, 4), x_ts(idx, 5), x_ts(idx, 6), sqrt(x_ts(idx, 1)^2 + x_ts(idx, 2)^2)]; % x = [x, y, chi, U]
                        targetship_traj = self.calcVesselTraj(state_ts, state_ts(3:4), self.dt, n_samples);
                        
                        [~, ~, ~, ~, ~, ~, colregs_cost] = self.calc_cost_COLREGS(ownship_traj, targetship_traj, n_samples);
                        [~, ~, collision_cost] = self.calc_cost_collision(ownship_traj, targetship_traj, self.dt, n_samples);
        
                        cost_i = max(colregs_cost + collision_cost);
            
                        if cost_i > cost_i_max
                            cost_i_max = cost_i;
                        end
                    end
        
                    if cost_i_max == -1 % when there are no other obstacles, turns to both sides must be equally penalized
                        F = K_P_ * (1 - U_ca_(j))^2 + K_CHI_P_ * Chi_ca_(i)^2;
                    else
                        F = self.calc_cost_maneuvering(chi_d, U_d, Chi_ca_(i), U_ca_(j), chi_m_last, U_m_last);
                    end
        
                    tot_cost = cost_i_max + F;
        
                    if tot_cost < cost_k_min
                        cost_k_min = tot_cost;
                        chi_m = Chi_ca_(i); 
                        U_m = U_ca_(j);
                    end
                end
            end
            chi_c = utils.wrap_angle_to_pmpi(chi_d + chi_m);
            U_c = U_d * U_m;
        end
    end
    methods (Hidden)
        function F = calc_cost_maneuvering(self, chi_d, U_d, chi_m, U_m, chi_m_last, U_m_last)
            % INPUTS:
            %        chi_d -> desired course angle in radians
            %        U_d -> desired speed in m/s
            %        chi_m -> course angle modification
            %        U_m -> speed modification
            %        chi_m_last -> course modification at the previous time
            %                      iteration
            %        U_m_last -> speed modification at the previous time
            %                    iteration
            % OUTPUTS:
            %        F -> maneuvering cost
            %

            K_CHI_SB_ = self.tuning_param.K_CHI_SB_;
            K_CHI_P_ = self.tuning_param.K_CHI_P_;
            K_DCHI_SB_ = self.tuning_param.K_DCHI_SB_;
            K_DCHI_P_ = self.tuning_param.K_DCHI_P_;
            K_P_ = self.tuning_param.K_P_;
            K_DP_ = self.tuning_param.K_DP_;

            if chi_m + chi_d < chi_d
                K_CHI_ = K_CHI_SB_;
            elseif chi_m + chi_d > chi_d
                K_CHI_ = K_CHI_P_;
            else
                K_CHI_ = 0;
            end

            d_chi = chi_m - chi_m_last;
            if d_chi > 0
                K_DCHI_ = K_DCHI_SB_;
            elseif d_chi < 0
                K_DCHI_ = K_DCHI_P_;
            else
                K_DCHI_ = 0;
            end
            F = K_P_ * (1 - U_m) + K_CHI_ * chi_m^2 + K_DP_ * abs(U_m_last - U_m) + K_DCHI_ * d_chi^2;
        end
        function [C_N, R_N, cost_N] = calc_cost_collision(self, os_traj, ts_traj, dt, N)
            D_SAFE_ = self.tuning_param.D_SAFE_;
            P_ = self.tuning_param.P_;
            Q_ = self.tuning_param.Q_;
            K_COLL_ = self.tuning_param.K_COLL_;

            C_N = zeros(1, N);
            R_N = zeros(1, N);
            cost_N = zeros(1, N);
            
            t = 0; t0 = 0;
            for i = 1:N
                t = t + dt;

                [d, v_o, v_s] = self.extractInfoFromTrajsAtSampleN(os_traj, ts_traj, i);

                if norm(d) < D_SAFE_
                    R_N(i) = (1 / (abs(t - t0) ^ P_)) * ((D_SAFE_ / norm(d))^Q_);
                    C_N(i) = K_COLL_ * norm(v_s - v_o) ^ 2;
                else
                    R_N(i) = 0;
                    C_N(i) = 0;
                end
                cost_N(i) = C_N(i) * R_N(i);
            end
        end
        function [CL_N, OT_N, SB_N, HO_N, CR_N, mu_N, cost_N] = calc_cost_COLREGS(self, os_traj, ts_traj, N)
            D_CLOSE_ = self.tuning_param.D_CLOSE_;
            PHI_OT_ = self.tuning_param.PHI_OT_;
            PHI_HO_ = self.tuning_param.PHI_HO_;
            PHI_AH_ = self.tuning_param.PHI_AH_;
            PHI_CR_ = self.tuning_param.PHI_CR_;
            KAPPA_ = self.tuning_param.KAPPA_;

            CL_N = zeros(1, N);
            OT_N = zeros(1, N);
            SB_N = zeros(1, N);
            HO_N = zeros(1, N);
            CR_N = zeros(1, N);
            mu_N = zeros(1, N);
            cost_N = zeros(1, N);

            for i = 1:N
                [d, v_o, v_s] = self.extractInfoFromTrajsAtSampleN(os_traj, ts_traj, i);
                
                % Check whether it is CLOSE
                CL_N(i) = norm(d) <= D_CLOSE_;

                % Overtaken by obstacle
                OT_N(i) = (dot(v_s, v_o) > cos(PHI_OT_) * norm(v_s) * norm(v_o)) & (norm(v_s) < norm(v_o));
    
                % Obstacle on starboard side
                os_psi_ = os_traj(i, 3);
                phi = utils.wrap_angle_diff_to_pmpi(atan2(d(2), d(1)), os_psi_);
                SB_N(i) = phi <= 0;
    
                % Obstacle Head-on
                los = d / norm(d);
                HO_N(i) = ((norm(v_o) > 0.05)...
                      & (dot(v_s, v_o) < -cos(PHI_HO_) * norm(v_s) * norm(v_o))...
                      & (dot(v_s, los) > cos(PHI_AH_) * norm(v_s))...
                     );
    
                % Crossing situation
                CR_N(i) = (dot(v_s, v_o) < cos(PHI_CR_) * norm(v_s) * norm(v_o));

                mu_N(i) = (CL_N(i) & SB_N(i) & HO_N(i)) | (CL_N(i) & SB_N(i) & CR_N(i) & ~OT_N(i));

                cost_N(i) = KAPPA_ * mu_N(i);
            end
        end
        function [d, v_o, v_s] = extractInfoFromTrajsAtSampleN(self, os_traj, ts_traj, N)
            d = zeros(1, 2);
            v_o = zeros(1, 2);
            v_s = zeros(1, 2);

            obs_x_ = ts_traj(N, 1);
            obs_y_ = ts_traj(N, 2);
            obs_psi_ = ts_traj(N, 3);
            obs_U_ = ts_traj(N, 4);
            obs_u_ = obs_U_ * cos(obs_psi_);
            obs_v_ = obs_U_ * sin(obs_psi_);
            
            os_x_ = os_traj(N, 1);
            os_y_ = os_traj(N, 2);
            os_psi_ = os_traj(N, 3);
            os_U_ = os_traj(N, 4);
            os_u_ = os_U_ * cos(os_psi_);
            os_v_ = os_U_ * sin(os_psi_);
    
            d(1) = obs_x_ - os_x_;
            d(2) = obs_y_ - os_y_;
            v_o(1) = obs_u_;
            v_o(2) = obs_v_;
            v_s(1) = os_u_;
            v_s(2) = os_v_;
        end
    end
end

