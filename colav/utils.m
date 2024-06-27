classdef utils
    methods (Static, Hidden)
        function  wrapped_ang_diff = wrap_angle_diff_to_pmpi(a_1, a_2)
            % Wraps angle difference a_1 - a_2 to within [-pi, pi)
            %
            % Args:
            %    a_1: Angle in radians
            %    a_2: Angle in radians
            %
            % Returns:
            %    wrapped_ang_diff: Wrapped angle difference
            
            diff = utils.wrap_angle_to_pmpi(a_1) - utils.wrap_angle_to_pmpi(a_2);
            
            wrapped_ang_diff = utils.wrap_min_max(diff, -pi, pi);
        end

        function wrapped_val = wrap_min_max(x, x_min, x_max)
            % Wraps input x to [x_min, x_max)
            %
            % Args:
            %    x: Unwrapped array
            %    x_min: Minimum value
            %    x_max: Maximum value
            %
            % Returns:
            %    wrapped_val: Wrapped value
            %
            wrapped_val = x_min + mod((x - x_min), (x_max - x_min));
        end

        function wrapped_ang = wrap_angle_to_pmpi(angle)
            % Wraps input angle to [-pi, pi)
            %
            % Args:
            %    angle: Angle in radians
            %
            % Returns:
            %    wrapped_ang = Wrapped angle
            %
            wrapped_ang = utils.wrap_min_max(angle, -pi, pi);
        end

        function norm_vec = normalize_vec(v)
            n = norm(v);
            if n < 0.000001
                norm_vec = v;
            else
                norm_vec = v / n;
            end
        end

        function x_sat = sat(x, x_min, x_max)
            x_sat = min(x_max, max(x_min, x));
        end

        function n_samp_ = getNumSamples(dt, T)
            n_samp_ = round(T / dt);
        end
    end
end

