classdef guidance
    properties (Constant, Hidden)
        default_R_a = 5.0;
        default_pass_angle_threshold = 90.0;
    end

    properties
        R_a
        pass_angle_threshold
    end
    
    methods
        function g = guidance(varargin) 
        % Creates an instance of the guidance class.
        % g = guidance(Ra, pass_angle_threshold)
        %
        % INPUTS:
        %       Ra -> Radius of acceptance, second threshold for switching 
        %             between wp segments (optional).
        %             scalar | double
        %       pass_angle_threshold -> First threshold for switching between 
        %                               wp segments in degrees (optional).
        %                               scalar | double

            p = inputParser;
            p.KeepUnmatched = true;
            addParameter(p, 'R_a', guidance.default_R_a, @(x) validateattributes(x, {'double'}, {'scalar'})); 
            addParameter(p, 'pass_angle_threshold', guidance.default_pass_angle_threshold, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0, '<=', 360}));
            parse(p, varargin{:});

            g.R_a = p.Results.R_a;
            g.pass_angle_threshold = p.Results.pass_angle_threshold;
        end
        function wp_idx = find_active_wp_segment(self, wp_pos, x, wp_idx)
            % Find the active waypoint segment depending on the vessel
            % position.
            % wp_idx = find_active_wp_segment(self, wp_pos, x, wp_idx)
            %           
            % INPUTS:
            %       wp_pos -> list of position coordinates of the waypoints
            %                 wp_pos -> [x_1, y_1;
            %                            x_2, y_2;
            %                               ...
            %                            x_n, y_n]
            %                 size (: x 2)  | matrix | double
            %       x -> current state of the vessel [u, v, r, x, y, psi]
            %            size (1 x 4) | vector | double
            %       wp_idx -> current waypoint index. use the function
            %                 find_active_wp_segment() to find the current 
            %                 waypoint index.
            %                 scalar | double

            validateattributes(wp_pos, {'double'}, {'size', [NaN, 2]})
            validateattributes(x, {'double'}, {'size', [1, 6]})

            wp_pos = wp_pos';

            x_los= [x(4), x(5), x(6), sqrt(x(1)^2+x(2)^2)]; % x_los = [x y psi U]
            x_los = x_los';

            n_wps = length(wp_pos);
            for i = wp_idx:n_wps-1
                d_0wp_vec = wp_pos(:, i + 1) - x_los(1:2);
                L_wp_segment = wp_pos(:, i + 1) - wp_pos(:, i);
        
                segment_passed = self.check_for_wp_segment_switch(L_wp_segment, d_0wp_vec);
                if segment_passed
                    wp_idx = wp_idx + 1;
                    %fprintf("Segment {%d} passed!\n", i);
                else
                    break;
                end
            end
        end
    end

    methods (Hidden)
        function segment_passed = check_for_wp_segment_switch(self, wp_segment, d_0wp)
            % Checks if a switch should be made from the current to the next
            % waypoint segment.
            %
            % Args:
            %    wp_segment: 2D vector describing the distance from waypoint i to i + 1 in the current segment.
            %    d_0wp: 2D distance vector from state to waypoint i + 1.
            %
            % Returns:
            %    bool: If the switch should be made or not.
            %
            wp_segment = utils.normalize_vec(wp_segment);
            d_0wp_norm = norm(d_0wp);
            d_0wp = utils.normalize_vec(d_0wp);
            
            segment_passed = dot(wp_segment, d_0wp) < cos(deg2rad(self.pass_angle_threshold));
        
            segment_passed = segment_passed || d_0wp_norm <= self.R_a;
        end
    end
end

