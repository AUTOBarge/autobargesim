classdef guidance
    properties
        R_a
        pass_angle_threshold
    end
    
    methods
        function g = guidance(varargin) % change to NameValueArgs
        % Creates an instance of the guidance class.
        % g = guidance(Ra, pass_angle_threshold)
        %
        % INPUTS:
        %       Ra -> Radius of acceptance, second threshold for switching 
        %             between wp segments.
        %             scalar | double
        %       pass_angle_threshold -> First threshold for switching between 
        %                               wp segments in degrees.
        %                               scalar | double
            p = inputParser;
            p.KeepUnmatched = true;
            addParameter(p, 'R_a', 10, @(x) validateattributes(x, {'double'}, {'scalar'})); 
            addParameter(p, 'pass_angle_threshold', 5, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0, '<=', 360}));
            parse(p, varargin{:});

            g.R_a = p.Results.R_a;
            g.pass_angle_threshold = p.Results.pass_angle_threshold;
        end
    end
    methods (Static, Hidden, Access=protected)
        function  wrapped_ang_diff = wrap_angle_diff_to_pmpi(a_1, a_2)
            % Wraps angle difference a_1 - a_2 to within [-pi, pi)
            %
            % Args:
            %    a_1: Angle in radians
            %    a_2: Angle in radians
            %
            % Returns:
            %    wrapped_ang_diff: Wrapped angle difference
            
            diff = guidance.wrap_angle_to_pmpi(a_1) - guidance.wrap_angle_to_pmpi(a_2);
            
            wrapped_ang_diff = guidance.wrap_min_max(diff, -pi, pi);
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
            wrapped_ang = guidance.wrap_min_max(angle, -pi, pi);
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

