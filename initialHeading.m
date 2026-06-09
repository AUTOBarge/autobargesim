function chi0 = initialHeading(wp_pos, look_ahead_m)
% This function computes an initial heading/course angle (chi0) for a path defined by a set of waypoints (wp_pos). 
% It takes into account a look-ahead distance (look_ahead_m) to ensure that the initial heading is stable and not overly influenced by nearby waypoints. 
% Falling back to the last waypoint if the path is shorter than look_ahead_m. This avoids the start-heading being dominated by a single nearby intermediate
% point (which can sit perpendicular to the channel after pull-to-center).
    if nargin < 2 || isempty(look_ahead_m)
        look_ahead_m = 50;
    end
    n = size(wp_pos, 1);
    if n < 2
        chi0 = 0;
        return;
    end
    cum_dist = 0;
    look_idx = n;
    for k = 1:n - 1
        cum_dist = cum_dist + norm(wp_pos(k+1, :) - wp_pos(k, :));
        if cum_dist >= look_ahead_m
            look_idx = k + 1;
            break;
        end
    end
    chi0 = atan2(wp_pos(look_idx, 2) - wp_pos(1, 2), ...
                 wp_pos(look_idx, 1) - wp_pos(1, 1));
end
