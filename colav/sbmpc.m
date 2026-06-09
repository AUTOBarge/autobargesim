classdef sbmpc < colav
    properties (Constant, Hidden)
        default_D_CLOSE_ = 60 % distance for a nearby obstacle [m] 
        default_D_SAFE_ = 30 % distance of safety zone [m] 

        default_KAPPA_ = 7.0 % cost function parameter
        default_PHI_AH_ = deg2rad(68.5) % colregs angle - ahead [deg]
        default_PHI_OT_ = deg2rad(68.5) % colregs angle - overtaken [deg]
        default_PHI_HO_ = deg2rad(22.5) % colregs angle - head on [deg]
        default_PHI_CR_ = deg2rad(68.5) % colregs angle - crossing [deg]

        default_K_COLL_ = 0.5 % cost scaling factor
        default_P_ = 1.0 % weights the importance of time until the event of collision occurs
        default_Q_ = 4.0 % exponent to satisfy colregs rule 16

        default_K_P_ = 2.5 % cost function parameter
        default_K_CHI_SB_ = 0.5 % cost function parameter
        default_K_CHI_P_ = 8.5 % cost function parameter
        default_K_DP_ = 2.0 % cost function parameter
        default_K_DCHI_SB_ = 0 % cost function parameter
        default_K_DCHI_P_ = 0 % cost function parameter

        default_Chi_ca_ = deg2rad([-90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]) % control behaviors - course offset [deg]
        default_U_ca_ = [0, 0.5, 1.0]  % control behaviors - speed factor
    end

    properties
        tuning_param % tuning parameters
        boundary_xy_enu      % depare boundary, decimated, as Nx2 vertices with NaN row separators (used by inpolygon land check)
        land_penalty = 5     % cost added when a candidate trajectory leaves depare
        land_check_stride = 5  % test every Nth ownship_traj sample for in-water
        update_stride = 5      % main loop calls run_sbmpc once every Nth simulation step
        boundary_decim_stride = 2 % keep every Nth depare boundary vertex for the per-step inpolygon land check
        bank_safety_margin = 8 % depare boundary is inset by this many metres when caching, so COLAV stays this far from the actual bank
        use_parallel = false % set true at construction iff Parallel Computing Toolbox is licensed
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

            sbmpcObj.use_parallel = false;
        end
        function self = set_depare_polyshape(self, polyshape_wgs84, lat0, lon0)
            % Cache the depare polygon in the same ENU frame as the
            % ownship state. 
            if isempty(polyshape_wgs84) || isempty(polyshape_wgs84.Vertices)
                self.boundary_xy_enu = zeros(0, 2);
                return;
            end
            verts_lonlat = polyshape_wgs84.Vertices;
            n = size(verts_lonlat, 1);
            verts_enu = nan(n, 2);
            wgs84 = wgs84Ellipsoid;
            for k = 1:n
                if any(isnan(verts_lonlat(k, :)))
                    continue;
                end
                [xE, yN, ~] = geodetic2enu(verts_lonlat(k, 2), verts_lonlat(k, 1), 0, lat0, lon0, 0, wgs84);
                verts_enu(k, :) = [xE, yN];
            end

            % Decimate the boundary vertices to speed up the inpolygon land check
            stride = max(1, self.boundary_decim_stride);
            nan_mask = any(isnan(verts_enu), 2);
            keep_mask = (mod((1:n)', stride) == 1) | nan_mask;
            decimated = verts_enu(keep_mask, :);
            warn_simp = warning('off', 'MATLAB:polyshape:repairedBySimplify');
            warn_b3 = warning('off', 'MATLAB:polyshape:boundary3Points');
            clean_poly = polyshape(decimated);
            if self.bank_safety_margin > 0
                inset_poly = polybuffer(clean_poly, -self.bank_safety_margin);
                if ~isempty(inset_poly.Vertices)
                    clean_poly = inset_poly;
                end
            end
            warning(warn_b3);
            warning(warn_simp);
            self.boundary_xy_enu = clean_poly.Vertices;
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

            % Pre-compute target-ship trajectories once.
            num_ts = size(x_ts, 1);
            ts_trajs = cell(num_ts, 1);
            for idx = 1:num_ts
                state_ts = [x_ts(idx, 4), x_ts(idx, 5), x_ts(idx, 6), sqrt(x_ts(idx, 1)^2 + x_ts(idx, 2)^2)];
                ts_trajs{idx} = self.calcVesselTraj(state_ts, state_ts(3:4), self.dt, n_samples);
            end

            n_chi = length(Chi_ca_);
            n_U = length(U_ca_);
            n_cand = n_chi * n_U;

            % Build the per-candidate control matrix in row-major order
            % (candidate k = (i-1)*n_U + j).
            controls = zeros(n_cand, 2);
            chi_ca_per_cand = zeros(n_cand, 1);
            U_ca_per_cand = zeros(n_cand, 1);
            for i = 1:n_chi
                base = (i - 1) * n_U;
                for j = 1:n_U
                    k = base + j;
                    controls(k, :) = [chi_d + Chi_ca_(i), U_d * U_ca_(j)];
                    chi_ca_per_cand(k) = Chi_ca_(i);
                    U_ca_per_cand(k) = U_ca_(j);
                end
            end

            % Vectorised RK4: propagate all candidate trajectories in
            % one batched pass
            states_init = repmat(state, n_cand, 1);
            all_trajs = self.calcVesselTrajsBatch(states_init, controls, self.dt, n_samples);

            costs = inf(n_cand, 1);
            if self.use_parallel
                parfor k = 1:n_cand
                    ownship_traj = squeeze(all_trajs(k, :, :));
                    costs(k) = self.computeCostFromTraj(ownship_traj, ...
                        chi_d, U_d, chi_ca_per_cand(k), U_ca_per_cand(k), ...
                        chi_m_last, U_m_last, ts_trajs, num_ts, n_samples);
                end
            else
                for k = 1:n_cand
                    ownship_traj = squeeze(all_trajs(k, :, :));
                    costs(k) = self.computeCostFromTraj(ownship_traj, ...
                        chi_d, U_d, chi_ca_per_cand(k), U_ca_per_cand(k), ...
                        chi_m_last, U_m_last, ts_trajs, num_ts, n_samples);
                end
            end

            [~, k_star] = min(costs);
            i_star = ceil(k_star / n_U);
            j_star = k_star - (i_star - 1) * n_U;
            chi_m = Chi_ca_(i_star);
            U_m = U_ca_(j_star);
            chi_c = utils.wrap_angle_to_pmpi(chi_d + chi_m);
            U_c = U_d * U_m;
        end
    end
    methods (Hidden)
        function cost = computeCostFromTraj(self, ownship_traj, chi_d, U_d, chi_ca, U_ca, chi_m_last, U_m_last, ts_trajs, num_ts, n_samples)
            % This function computes the cost for a given ownship trajectory, which includes the 
            % collision and COLREGS cost with all target ships, as well as the maneuvering cost
            % and land cost for the candidate control that generated the trajectory.
            land_cost = 0;
            if ~isempty(self.boundary_xy_enu) && (chi_ca ~= 0 || U_ca ~= 1)
                idx_check = 1:self.land_check_stride:size(ownship_traj, 1);
                in_water = inpolygon(ownship_traj(idx_check, 1), ownship_traj(idx_check, 2), ...
                                     self.boundary_xy_enu(:, 1), self.boundary_xy_enu(:, 2));
                if ~all(in_water)
                    land_cost = self.land_penalty * mean(~in_water);
                end
            end

            cost_i_max = -1;
            for idx = 1:num_ts
                targetship_traj = ts_trajs{idx};
                [~, ~, ~, ~, ~, ~, colregs_cost] = self.calc_cost_COLREGS(ownship_traj, targetship_traj, n_samples);
                [~, ~, collision_cost] = self.calc_cost_collision(ownship_traj, targetship_traj, self.dt, n_samples);
                cost_i = max(colregs_cost + collision_cost);
                if cost_i > cost_i_max
                    cost_i_max = cost_i;
                end
            end

            F = self.calc_cost_maneuvering(chi_d, U_d, chi_ca, U_ca, chi_m_last, U_m_last);
            cost = cost_i_max + F + land_cost;
        end
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

            dx = ts_traj(1:N, 1) - os_traj(1:N, 1);
            dy = ts_traj(1:N, 2) - os_traj(1:N, 2);
            d_norm = sqrt(dx.^2 + dy.^2);

            obs_U = ts_traj(1:N, 4);
            obs_psi = ts_traj(1:N, 3);
            os_U = os_traj(1:N, 4);
            os_psi = os_traj(1:N, 3);

            vrel_x = os_U .* cos(os_psi) - obs_U .* cos(obs_psi);
            vrel_y = os_U .* sin(os_psi) - obs_U .* sin(obs_psi);

            t = (1:N).' * dt;
            mask = d_norm < D_SAFE_;

            R_N = zeros(N, 1);
            C_N = zeros(N, 1);
            R_N(mask) = (1 ./ (abs(t(mask)) .^ P_)) .* ((D_SAFE_ ./ d_norm(mask)) .^ Q_);
            C_N(mask) = K_COLL_ * (vrel_x(mask).^2 + vrel_y(mask).^2);

            C_N = C_N.';
            R_N = R_N.';
            cost_N = C_N .* R_N;
        end
        function [CL_N, OT_N, SB_N, HO_N, CR_N, mu_N, cost_N] = calc_cost_COLREGS(self, os_traj, ts_traj, N)
            D_CLOSE_ = self.tuning_param.D_CLOSE_;
            PHI_OT_ = self.tuning_param.PHI_OT_;
            PHI_HO_ = self.tuning_param.PHI_HO_;
            PHI_AH_ = self.tuning_param.PHI_AH_;
            PHI_CR_ = self.tuning_param.PHI_CR_;
            KAPPA_ = self.tuning_param.KAPPA_;

            dx = ts_traj(1:N, 1) - os_traj(1:N, 1);
            dy = ts_traj(1:N, 2) - os_traj(1:N, 2);
            d_norm = sqrt(dx.^2 + dy.^2);

            obs_U = ts_traj(1:N, 4);
            obs_psi = ts_traj(1:N, 3);
            os_U = os_traj(1:N, 4);
            os_psi = os_traj(1:N, 3);

            os_u = os_U .* cos(os_psi);
            os_v = os_U .* sin(os_psi);
            obs_u = obs_U .* cos(obs_psi);
            obs_v = obs_U .* sin(obs_psi);

            v_s_norm = abs(os_U);
            v_o_norm = abs(obs_U);
            dot_so = os_u .* obs_u + os_v .* obs_v;

            CL = d_norm <= D_CLOSE_;
            OT = (dot_so > cos(PHI_OT_) .* v_s_norm .* v_o_norm) & (v_s_norm < v_o_norm);

            phi = mod((atan2(dy, dx) - os_psi) + pi, 2*pi) - pi;
            SB = phi <= 0;

            safe_d = max(d_norm, eps);
            los_x = dx ./ safe_d;
            los_y = dy ./ safe_d;
            dot_s_los = os_u .* los_x + os_v .* los_y;
            HO = (v_o_norm > 0.05) ...
               & (dot_so < -cos(PHI_HO_) .* v_s_norm .* v_o_norm) ...
               & (dot_s_los > cos(PHI_AH_) .* v_s_norm);

            CR = dot_so < cos(PHI_CR_) .* v_s_norm .* v_o_norm;

            mu = (CL & SB & HO) | (CL & SB & CR & ~OT);

            CL_N = CL.';
            OT_N = OT.';
            SB_N = SB.';
            HO_N = HO.';
            CR_N = CR.';
            mu_N = mu.';
            cost_N = KAPPA_ * double(mu_N);
        end
    end
end

