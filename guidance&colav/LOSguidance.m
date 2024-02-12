classdef LOSguidance   
    properties (Constant, Hidden)
        default_pass_angle_threshold = 90.0;
        default_R_a = 5.0;
        default_K_p = 1.0 / 20.0;
    end

    properties 
        pass_angle_threshold
        R_a
        K_p
    end
    
    methods
        function losg = LOSguidance(NameValueArgs)
        % Creates an instance of the LOS guidance class.
        % losg = LOSguidance(Kp, Ra, pass_ang_thresh)
        %
        % INPUTS:
        %       Kp -> Proportional gain in LOS law, K_p = 1 / lookahead 
        %             distance.
        %             scalar | double
        %       Ra -> Radius of acceptance, second threshold for switching 
        %             between wp segments.
        %             scalar | double
        %       pass_ang_thresh -> First threshold for switching between 
        %             wp segments in degrees.
        %             scalar | double
            arguments
                NameValueArgs.Kp (1, 1) double = LOSguidance.default_K_p
                NameValueArgs.Ra (1, 1) double = LOSguidance.default_R_a
                NameValueArgs.pass_ang_thresh (1, 1) double {mustBeInRange(NameValueArgs.pass_ang_thresh, 0, 360)} = LOSguidance.default_pass_angle_threshold
            end

            losg.K_p = NameValueArgs.Kp;
            losg.R_a = NameValueArgs.Ra;
            losg.pass_angle_threshold = NameValueArgs.pass_ang_thresh;
        end
    end
end

