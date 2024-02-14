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
            addParameter(p, 'R_a', guidance.default_R_a, @(x) validateattributes(x, {'double'}, {'scalar'})); 
            addParameter(p, 'pass_angle_threshold', guidance.default_pass_angle_threshold, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0, '<=', 360}));
            parse(p, varargin{:});

            g.R_a = p.Results.R_a;
            g.pass_angle_threshold = p.Results.pass_angle_threshold;
        end
    end
end

