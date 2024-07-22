classdef colav
    properties (Constant, Hidden)
        default_T_chi = 1;
        default_T_U = 1;
    end

    properties
        T % prediction horizon
        dt % prediction sampletime
        T_chi % tuning parameters of the internal kinematic model: course time constant
        T_U % tuning parameters of the internal kinematic model: speed time constant
    end
    
    methods (Hidden)
        function colavObj = colav(T, dt, varargin)
            validateattributes(T, {'double'}, {'scalar', '>', 0});
            validateattributes(dt, {'double'}, {'scalar', '>', 0, '<', T});

            p = inputParser;
            p.KeepUnmatched = true;
            addParameter(p, 'T_chi', sbmpc.default_T_chi, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'T_U', sbmpc.default_T_U, @(x) validateattributes(x, {'double'}, {'scalar'}));
            parse(p, varargin{:});
            
            colavObj.T = T;
            colavObj.dt = dt;
            colavObj.T_chi = p.Results.T_chi;
            colavObj.T_U = p.Results.T_U;
        end
        function trajectory = calcVesselTraj(self, x, u, dt, n_samples)
            trajectory = zeros(n_samples, numel(x));
            trajectory(1, :) = x;

            for idx = 2:n_samples
                trajectory(idx, :) = self.step_vessel(trajectory(idx-1, :), u, dt);
            end
        end
        function x_new = step_vessel(self, x, u, dt)            
            k1 = self.step(x, u);
            k2 = self.step(x + k1 * dt/2, u);
            k3 = self.step(x + k2 * dt/2, u);
            k4 = self.step(x + k3 * dt, u);
            x_new = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        function x_next = step(self, x, u)
            x_next = zeros(1, numel(x));

            x_next(1) = x(4) * cos(x(3));
            x_next(2) = x(4) * sin(x(3));
            x_next(3) = utils.wrap_angle_diff_to_pmpi(u(1), x(3)) / self.T_chi;
            x_next(4) = (u(2) - x(4)) / self.T_U;
        end
    end
end

