classdef mission
    properties
        pos_vec
        speed_vec
        mission_plan
    end
    
    methods
        function M = mission(pos, speed)
        % MISSION Creates a mission plan.
        % M = MISSION(pos, speed) creates a mission plan matrix of shape (: x 3)
        %       
        % OUTPUTS:
        %       M -> [x_1, y_1, U_1
        %                  ...
        %             x_n, y_n, U_n]
        %            
        %             where n is the number of waypoints. x and y are in
        %             'm' while U is in 'm/s' unit.
        %
        % INPUTS:
        %       pos -> position coordinate array of the waypoints in 'm'.
        %              size (: x 2) | matrix | double
        %       speed -> speed vector containing the expected speed at each
        %                waypoint in 'm/s'
        %                size (: x 1) | vector | double
        %
        
            arguments
                pos (:,2) double
                speed (:,1) double 
            end
            
            % Validation
            if ~isequal(length(pos(:, 1)), length(speed))
                msg = 'Length of first input must equal length of second input.';
                error(msg)
            end

            % Function code
            M.pos_vec = pos;
            M.speed_vec = speed;
            M.mission_plan = [M.pos_vec, M.speed_vec];
        end
    end
end

