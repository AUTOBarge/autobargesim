classdef LOSguidance < guidance
    properties (Constant, Hidden, Access=private)
        default_K_p = 1.0 / 20.0;
        default_R_a = 5.0;
        default_pass_angle_threshold = 90.0;
    end

    properties 
        K_p
    end
    
    methods (Access=public)
        function losgObj = LOSguidance(varargin)
            % Creates an instance of the LOS guidance class.
            % losg = LOSguidance(Kp, Ra, pass_angle_threshold)
            %
            % INPUTS:
            %       Kp -> Proportional gain in LOS law, K_p = 1 / lookahead 
            %             distance.
            %             scalar | double
            %       For more information about the parameters 'Ra' and
            %       'pass_angle_threshold' type: help guidance in the command
            %       window.
            
            p = inputParser;
            p.KeepUnmatched = true;
            addParameter(p, 'K_p', LOSguidance.default_K_p, @(x) validateattributes(x, {'double'}, {'scalar'}));
            addParameter(p, 'R_a', LOSguidance.default_R_a);
            addParameter(p, 'pass_angle_threshold', LOSguidance.default_pass_angle_threshold);
            parse(p, varargin{:});
            
            losgObj = losgObj@guidance('R_a', p.Results.R_a, 'pass_angle_threshold', p.Results.pass_angle_threshold);
            losgObj.K_p = p.Results.K_p;
        end
        function [chi_d, U_d] = compute_LOSRef(self, wp_pos, wp_speed, x, wp_idx)
            % Compute reference course angles and speeds using the LOS guidance
            % law.
            % [chi_d, U_d] = compute_LOSRef(waypoints, speed_plan, x_k, wp_counter)
            %           
            % INPUTS:
            %       wp_pos -> list of position coordinates of the waypoints
            %                 wp_pos -> [x_1, y_1;
            %                            x_2, y_2;
            %                               ...
            %                            x_n, y_n]
            %                 size (: x 2)  | matrix | double
            %       wp_speed -> expected speed at each waypoint
            %                   wp_speed -> [u_1;
            %                                u_2;
            %                                ...
            %                                u_n]
            %                   size (: x 1) | vector | double
            %       x -> current state of the vessel [x, y, chi, U]
            %            size (1 x 4) | vector | double
            %       wp_idx -> current waypoint index. use the function
            %                 find_active_wp_segment() to find the current 
            %                 waypoint index.
            %                 scalar | double

            validateattributes(wp_pos, {'double'}, {'size', [NaN,2]})
            validateattributes(wp_speed, {'double'}, {'size', [NaN,1]})
            validateattributes(x, {'double'}, {'size', [1,4]})
            validateattributes(wp_idx, {'double'}, {'scalar'})

            wp_pos = wp_pos';
            wp_speed = wp_speed';
            x = x';

            n_wps = length(wp_speed);
            if wp_idx >= n_wps
                L_wp_segment = wp_pos(:, wp_idx) - wp_pos(:, wp_idx - 1);
            else
                L_wp_segment = wp_pos(:, wp_idx + 1) - wp_pos(:, wp_idx);
            end

            alpha = atan2(L_wp_segment(2), L_wp_segment(1));
            e = -(x(1) - wp_pos(1, wp_idx)) * sin(alpha) + (x(2) - wp_pos(2, wp_idx)) * cos(alpha);

            chi_r = atan2(-(self.K_p * e), 1);
            chi_d = guidance.wrap_angle_to_pmpi(alpha + chi_r);
            U_d = wp_speed(wp_idx);
        end
        function wp_idx = find_active_wp_segment(self, wp_pos, x, wp_idx)
            % Compute reference course angles and speeds using the LOS guidance
            % law.
            % [chi_d, U_d] = compute_LOSRef(waypoints, speed_plan, x_k, wp_counter)
            %           
            % INPUTS:
            %       wp_pos -> list of position coordinates of the waypoints
            %                 wp_pos -> [x_1, y_1;
            %                            x_2, y_2;
            %                               ...
            %                            x_n, y_n]
            %                 size (: x 2)  | matrix | double
            %       x -> current state of the vessel [x, y, chi, U]
            %            size (1 x 4) | vector | double
            %       wp_idx -> current waypoint index. use the function
            %                 find_active_wp_segment() to find the current 
            %                 waypoint index.
            %                 scalar | double
            
            validateattributes(wp_pos, {'double'}, {'size', [NaN,2]})
            validateattributes(x, {'double'}, {'size', [1,4]})

            wp_pos = wp_pos';
            x = x';

            n_wps = length(wp_pos);
            for i = wp_idx:n_wps-1
                d_0wp_vec = wp_pos(:, i + 1) - x(1:2);
                L_wp_segment = wp_pos(:, i + 1) - wp_pos(:, i);
        
                segment_passed = self.check_for_wp_segment_switch(L_wp_segment, d_0wp_vec);
                if segment_passed
                    wp_idx = wp_idx + 1;
                    fprintf("Segment {%d} passed!\n", i);
                else
                    break;
                end
            end
        end
        
% --------------------- SAMPLE --------------------- %
%         function val = testfunc(self, x, y, varargin)
%             validateattributes(x, {'double'}, {'scalar'});
%             validateattributes(y, {'double'}, {'scalar'})
% 
%             p = inputParser;
%             p.KeepUnmatched = true;
%             addParameter(p, 'l', 101, @(x) validateattributes(x, {'double'}, {'scalar'}));
%             addParameter(p, 'm', 100, @(x) validateattributes(x, {'double'}, {'scalar'}));
%             parse(p, varargin{:});
% 
%             val.x = x;
%             val.y = y;
%             val.z = self.K_p;
%             val.l = p.Results.l;
%             val.m = p.Results.m;
%         end
% -------------------------------------------------- %
    end

    methods(Access=private)
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
            wp_segment = guidance.normalize_vec(wp_segment);
            d_0wp_norm = norm(d_0wp);
            d_0wp = guidance.normalize_vec(d_0wp);
            
            segment_passed = dot(wp_segment, d_0wp) < cos(deg2rad(self.pass_angle_threshold));
        
            segment_passed = segment_passed || d_0wp_norm <= self.R_a;
        end
    end
end

