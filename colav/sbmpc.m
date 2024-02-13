classdef sbmpc
    properties (Constant, Hidden, Access=private)
        default_D_CLOSE_ = 20 % distance for an nearby obstacle [m]
        default_D_SAFE_ = 10 % distance of safety zone [m]
        default_D_INIT_ = SBMPC.default_D_CLOSE_ * 3

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
        T % prediction horizon
        dt % prediction sampletime
    end
    
    methods (Access=public)
        function sbmpcObj = sbmpc(T, dt, varargin)
            validateattributes(T, {'double'}, {'scalar', '>', 0});
            validateattributes(dt, {'double'}, {'scalar', '>', 0, '<', T});

            p = inputParser;
            p.KeepUnmatched = true;
            addParameter(p, 'D_CLOSE_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'D_SAFE_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'D_INIT_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'KAPPA_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_AH_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_OT_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_HO_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'PHI_CR_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_COLL_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'P_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'Q_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_P_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_CHI_SB_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_CHI_P_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DP_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DCHI_SB_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'K_DCHI_P_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'Chi_ca_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'U_ca_', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            parse(p, varargin{:});

            sbmpcObj.T = T;
            sbmpcObj.dt = dt;
            sbmpcObj.tuning_param = struct(...
                'D_CLOSE_', SBMPC.default_D_CLOSE_,...
                'D_SAFE_', SBMPC.default_D_SAFE_,...
                'D_INIT_', SBMPC.default_D_INIT_,...
                'KAPPA_', SBMPC.default_KAPPA_,...
                'PHI_AH_', SBMPC.default_PHI_AH_,...
                'PHI_OT_', SBMPC.default_PHI_OT_,...
                'PHI_HO_', SBMPC.default_PHI_HO_,...
                'PHI_CR_', SBMPC.default_PHI_CR_,...
                'K_COLL_', SBMPC.default_K_COLL_,...
                'P_', SBMPC.default_P_,...
                'Q_', SBMPC.default_Q_,...
                'K_P_', SBMPC.default_K_P_,...
                'K_CHI_SB_', SBMPC.default_K_CHI_SB_,...
                'K_CHI_P_', SBMPC.default_K_CHI_P_,...
                'K_DP_', SBMPC.default_K_DP_,...
                'K_DCHI_SB_', SBMPC.default_K_DCHI_SB_,...
                'K_DCHI_P_', SBMPC.default_K_DCHI_P_,...
                'Chi_ca_', SBMPC.default_Chi_ca_,...
                'U_ca_', SBMPC.default_U_ca_);
        end
    end
    methods (Access=private)
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
                [d, v_o, v_s] = SBMPC.extractInfoFromTrajsAtSampleN(os_traj, ts_traj, i);
                
                % Check whether it is CLOSE
                CL_N(i) = norm(d) <= D_CLOSE_;

                % Overtaken by obstacle
                OT_N(i) = (dot(v_s, v_o) > cos(PHI_OT_) * norm(v_s) * norm(v_o)) & (norm(v_s) < norm(v_o));
    
                % Obstacle on starboard side
                os_psi_ = os_traj(i, 3);
                phi = MISC.wrap_angle_diff_to_pmpi(atan2(d(2), d(1)), os_psi_);
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

