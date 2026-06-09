classdef planner
    % ClassName: maps.planner
    %
    % Description:
    % The planner class is used to generate paths within a polygon area.
    % This class includes functionalities to randomly generate start and
    % end points, build a graph, compute the best path and waypoints,
    % and plot. The Dijkstra graph used for shortest-path search is
    % weighted with a bank-clearance penalty so the optimiser prefers
    % wide-channel routes; after planning, each interior corner is
    % replaced with a circular arc of radius R_min tangent to the
    % adjacent segments so the controller is never given a corner the
    % vessel cannot physically execute (arcs that would cut a corner
    % over land are rejected and the original corner is kept);
    % finally, every waypoint on the resulting path is audited against
    % the fairway boundary and the user is warned when the planned
    % route runs the vessel within (B/2 + safety_margin) of the bank.
    %
    % Properties:
    %   - pgon_memory: Structure storing shp.file data
    %   - segments: Cell array of segment data
    %   - G: Weighted graph object representing the connectivity between segments
    %   - unique_coords: Matrix of unique coordinates of the nodes in graph G
    %   - edges: List of edges of the graph
    %   - edge_weights: Per-edge cost used by shortestpath (length plus
    %     bank-clearance penalty for waterway segments, length-only for
    %     synthetic connecting edges)
    %   - best_path: Best path node indices
    %   - path_depths: Depth information of the best path
    %   - path_points: Coordinate points of the best path
    %   - path_clearances: Bank clearance (m) at each row of path_points
    %   - given_point1: Start point coordinates
    %   - given_point2: End point coordinates
    %   - all_points: Collection of all segment coordinate points
    %   - polygon: Vertices of the depare (navigable area) polygon
    %   - polygon_land: Vertices of the lndare polygon
    %   - bank_anchor_lonlat: WGS84 anchor used for the metric conversion
    %   - bank_m_per_deg: Cached [m_per_deg_lon, m_per_deg_lat] at the anchor
    %   - bank_seg_starts_m / bank_seg_ends_m: Boundary edges (in metres,
    %     anchored on bank_anchor_lonlat) used by bank_clearance
    %   - depare_polyshape: Cached polyshape(obj.polygon), used by the
    %     arc smoother to reject any arc that would leave the navigable
    %     area (cut a corner over land)
    %   - vessel_beam: Vessel beam B in metres, used for the audit threshold
    %   - safety_margin_m: Extra clearance margin in metres added to B/2
    %   - k_bank: Bank-clearance weight in the Dijkstra edge cost
    %     (w = length_m + k_bank * length_m / max(eps, min_clearance_m));
    %     larger values bias the planner more strongly toward
    %     wide-channel routes
    %   - R_min: Minimum turning radius (m) for the corner arc smoother;
    %     default 150 m, =~ 4 x ship length); set to 0 to disable corner smoothing
    %
    % Constructor:
    %   planner(pgon_memory) uses default vessel_beam = 5.05 m,
    %   safety_margin_m = 5 m, k_bank = 30, R_min = 150 m. To override,
    %   pass name-value pairs, e.g.:
    %     planner(pgon_memory, 'VesselBeam', 9.5, 'SafetyMargin', 6, ...
    %                          'KBank', 35, 'RMin', 200)
    %
    % Author:
    %   Zhongbi Luo
    %
    % Updated by:
    %   Abhishek Dhyani
    %
    % Date:
    %   2026-05-08
    %
    % Version:
    %   1.1


    properties
        pgon_memory
        segments
        G
        unique_coords
        edges
        edge_weights
        best_path
        path_points
        path_depths
        path_clearances
        given_point1
        given_point2
        all_points
        polygon
        polygon_land
        bank_anchor_lonlat
        bank_m_per_deg
        bank_seg_starts_m
        bank_seg_ends_m
        depare_polyshape
        vessel_beam
        safety_margin_m
        k_bank
        R_min
        max_segment_length_m
    end

     methods
        function obj = planner(pgon_memory, varargin)
            % planner constructor
            p = inputParser;
            addParameter(p, 'VesselBeam', 5.05, @(x) validateattributes(x, {'double'}, {'scalar', '>', 0}));
            addParameter(p, 'SafetyMargin', 5.5, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0}));
            addParameter(p, 'KBank', 30.0, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0}));
            addParameter(p, 'RMin', 150.0, @(x) validateattributes(x, {'double'}, {'scalar', '>=', 0}));
            addParameter(p, 'MaxSegmentLength', 50.0, @(x) validateattributes(x, {'double'}, {'scalar', '>', 0}));
            parse(p, varargin{:});
            obj.pgon_memory = pgon_memory;
            obj.vessel_beam = p.Results.VesselBeam;
            obj.safety_margin_m = p.Results.SafetyMargin;
            obj.k_bank = p.Results.KBank;
            obj.R_min = p.Results.RMin;
            obj.max_segment_length_m = p.Results.MaxSegmentLength;
            obj = obj.extractData();
            obj = obj.prepareBankGeometry();
            obj = obj.buildGraph();
            obj = obj.checkAndConnectComponents();
        end

        function obj = plan_path(obj, given_point1, given_point2)
            % This function plans the path from given_point1 to given_point2.
            % It checks if the points are inside the polygon, 
            % finds the nearest segments, and plans the path accordingly.
            % It also attaches literal endpoints if necessary and audits the path clearance.
            
            obj.given_point1 = given_point1;
            obj.given_point2 = given_point2;
            
            % Check if the given points are inside the polygon
            in_polygon1=obj.is_point_inside_polygon(given_point1);
            in_polygon2=obj.is_point_inside_polygon(given_point2);

                % Check if the points are inside the polygon_land
            in_polygon_land1 = obj.is_point_inside_polygon_land(given_point1);
            in_polygon_land2 = obj.is_point_inside_polygon_land(given_point2);

            if in_polygon1 && in_polygon2
                    disp('Starting point and ending point set successfully');
                    disp(['Starting point: ', num2str(given_point1)]);
                    disp(['Ending point: ', num2str(given_point2)]);
                elseif (in_polygon_land1 || in_polygon_land2)
                    % Points are outside polygon but inside polygon_land
                    if ~in_polygon1
                        disp(['Starting point is outside the boundary: ', num2str(given_point1)]);
                    end
                    if ~in_polygon2
                        disp(['Ending point is outside the boundary: ', num2str(given_point2)]);
                    end
                    fprintf('The selected points are outside the boundary.\nThe simulation will proceed with valid points.');
                else
                    % Points are outside both polygon and polygon_land
                    error('The selected points are outside the map. Please choose points within the map or default points');
             end


            [obj, start_segment, end_segment, start_point, end_point] = obj.findNearestPoints(given_point1, given_point2);
            
            if start_segment == end_segment
            % Determine whether the start and end point are on the same segment
                obj = obj.planDirectPath(start_segment, start_point, end_point);
            else
                obj = obj.planShortestPath(start_segment, end_segment, start_point, end_point);
            end

            obj = obj.attachLiteralEndpoints(in_polygon1, in_polygon2);

            obj = obj.smoothArcsAtCorners();

            obj = obj.densifyPath();
            obj = obj.pullToCenter();

            obj = obj.auditPathClearance();
      end
        

        function [given_point1, given_point2] = generate_random_points(obj)
           % Generate random points method
           rng('shuffle'); % Make sure the random number generator is different each time it is run

            % Pick two points at random
            num_points = size(obj.all_points, 1);
            selected_indices = randperm(num_points, 2);
            selected_points1 = obj.all_points(selected_indices(1), :);
            selected_points2 = obj.all_points(selected_indices(2), :);

            % Add random noise in the range +- 0.004 to X and +- 0.002 to Y
            noise_x = 0.004 * (2 * rand(1, 1) - 1);
            noise_y = 0.002 * (2 * rand(1, 1) - 1);

            given_point1 = selected_points1 + [noise_x, noise_y];
            given_point2 = selected_points2 + [noise_x, noise_y];
        end

        function plot_path(obj,num)
            % plot_path method
            % Draws the graph and path with varying levels of detail based on the input number.
            %   num - Controls the content to be plotted
            %         1: All information
            %         2: Only waypoints, start point, end point, without node and best path
                hold on;

            switch num
    
            case 1
                % Plot the full graph with all information
                h = plot(obj.G, 'XData', obj.unique_coords(:, 1), 'YData', obj.unique_coords(:, 2));
                highlight(h, obj.best_path, 'EdgeColor', 'r', 'LineWidth', 2);
                plot(obj.given_point1(1), obj.given_point1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
                plot(obj.given_point2(1), obj.given_point2(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

            scatter_handle = scatter(obj.path_points(:, 1), obj.path_points(:, 2), 10, 'k', 'filled');
            % Store the depth data in the 'UserData' property
            set(scatter_handle, 'UserData', obj.path_depths);
        
            datacursormode off;
            
            dcm_obj = datacursormode(gcf);
            set(dcm_obj, 'UpdateFcn', @obj.myupdatefcn);

                title('Map Area with Navigation Data');
                xlabel('X Coordinate');
                ylabel('Y Coordinate');
    
           case 2
                % Plot only waypoints, start point, end point, and the best path
                plot(obj.given_point1(1), obj.given_point1(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
                plot(obj.given_point2(1,1), obj.given_point2(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
                plot(obj.path_points(:, 1), obj.path_points(:, 2), 'k.', 'MarkerSize', 10);
                title('Waypoints and Best Path');
                xlabel('X Coordinate');
                ylabel('Y Coordinate');
    
            otherwise
                error('Invalid input number. Please choose 1 or 2.');
            end
%              Scale to the area between the red and green dots
%             x_min = min(obj.given_point1(1), obj.given_point2(1)) - 0.01;
%             x_max = max(obj.given_point1(1), obj.given_point2(1)) + 0.01;
%             y_min = min(obj.given_point1(2), obj.given_point2(2)) - 0.01;
%             y_max = max(obj.given_point1(2), obj.given_point2(2)) + 0.01;
%             xlim([x_min x_max]);
%             ylim([y_min y_max]);
%             Scale from the starting point
            
            x_center = obj.path_points(1,1);
            y_center = obj.path_points(1,2);
            x_min = x_center - 0.015;
            x_max = x_center + 0.015;
            y_min = y_center - 0.015;
            y_max = y_center + 0.015;
            xlim([x_min x_max]);
            ylim([y_min y_max]);


            hold off;
        end

function txt = myupdatefcn(obj, ~, event_obj)
    % Customize the text of the data

    pos = get(event_obj, 'Position');
    % Gets depth data from the drawing object's 'UserData'
    target = get(event_obj, 'Target');
    depth_data = get(target, 'UserData');

    x = obj.path_points(:, 1);
    y = obj.path_points(:, 2);
    idx = find(abs(x - pos(1)) < 1e-6 & abs(y - pos(2)) < 1e-6, 1);
    if isempty(idx)
        depth = NaN;
    else
        depth = depth_data(idx);
    end

    txt = {['X: ', num2str(pos(1))], ...
           ['Y: ', num2str(pos(2))], ...
           ['Depth: ', num2str(depth)]};
end

        function plot_path_points_with_numbers(obj)
                %only for test

                % pl.plot_path_points_with_numbers(); %testing
                
                % Plot path points with numbers
                figure;
                hold on;
                
                % Plot each point and its index
                for i = 1:size(obj.path_points, 1)
                    plot(obj.path_points(i, 1), obj.path_points(i, 2), 'bo', 'MarkerSize', 8);
                    text(obj.path_points(i, 1), obj.path_points(i, 2), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
                end
                
                title('Path Points with Numbers');
                xlabel('X Coordinate');
                ylabel('Y Coordinate');
                hold off;
        end
end


       methods (Access = private)
        function obj = extractData(obj)
            % Extract data method
            % Extracts segment data and polygon data from pgon_memory
            index1 = find(strcmp({obj.pgon_memory.name}, 'wtwaxs'));
            lines_data = obj.pgon_memory(index1).lines;
            info_data = obj.pgon_memory(index1).info;

            index2 = find(strcmp({obj.pgon_memory.name}, 'depare'));
            polygon_data = obj.pgon_memory(index2).polygons;
            obj.polygon = polygon_data.Vertices; 

            index3 = find(strcmp({obj.pgon_memory.name}, 'lndare'));
            polygon_data_land = obj.pgon_memory(index3).polygons;
            obj.polygon_land = polygon_data_land.Vertices; 

            x = lines_data(1, :);
            y = lines_data(2, :);

            start_idx = 1;
            for i = 1:length(x)
                if isnan(x(i))
                    if start_idx < i-1
                        segment.x = x(start_idx:i-1);
                        segment.y = y(start_idx:i-1);
                        % Remove overlapped points
                        [unique_points, unique_indices] = unique([segment.x', segment.y'], 'rows', 'stable');
                        segment.x = unique_points(:, 1)';
                        segment.y = unique_points(:, 2)';
                        % Assign segment to the corresponding region
                        for j = 1:length(info_data)
                            if ismember(segment.x(1), info_data(j).X) && ismember(segment.y(1), info_data(j).Y)
                                segment.region = info_data(j).sourceFile; % Assign the region/label from info_data
                                break;
                            end
                        end
                        obj.segments{end+1} = segment;
                    end
                    start_idx = i + 1;
                elseif i == length(x)
                    if start_idx <= i
                        segment.x = x(start_idx:i);
                        segment.y = y(start_idx:i);
                        % Remove overlapped points
                        [unique_points, unique_indices] = unique([segment.x', segment.y'], 'rows', 'stable');
                        segment.x = unique_points(:, 1)';
                        segment.y = unique_points(:, 2)';
                        % Assign segment to the corresponding region
                        for j = 1:length(info_data)
                            if ismember(segment.x(1), info_data(j).X) && ismember(segment.y(1), info_data(j).Y)
                                segment.region = info_data(j).sourceFile; % Assign the region/label from info_data
                                break;
                            end
                        end
                        obj.segments{end+1} = segment;
                    end
                end
            end

            endpoints = [];
            all_points = [];
            for i = 1:length(obj.segments)
                segment = obj.segments{i};
                endpoints = [endpoints; segment.x(1), segment.y(1); segment.x(end), segment.y(end)];
                all_points = [all_points; [segment.x(:), segment.y(:)]];
            end
            obj.unique_coords = unique(endpoints, 'rows', 'stable');
            obj.all_points=all_points;
        end

        function obj = buildGraph(obj)
            % Build graph with edge weights
            % w = length_m + k_bank * length_m / max(eps, min_clearance_m)
            % so the Dijkstra optimiser prefers wide-channel routes.
            n_seg = length(obj.segments);
            edges = zeros(n_seg, 2);
            weights = zeros(n_seg, 1);
            for i = 1:n_seg
                segment = obj.segments{i};
                start_node = [segment.x(1), segment.y(1)];
                end_node = [segment.x(end), segment.y(end)];
                edges(i, 1) = find(ismember(obj.unique_coords, start_node, 'rows'));
                edges(i, 2) = find(ismember(obj.unique_coords, end_node, 'rows'));
                weights(i) = obj.computeSegmentWeight(segment);
            end
            obj.edges = edges;
            obj.edge_weights = weights;
            obj.G = graph(edges(:, 1), edges(:, 2), weights);
        end

        function w = computeSegmentWeight(obj, segment)
            pts = [segment.x(:), segment.y(:)];
            pts_m = (pts - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
            length_m = sum(vecnorm(diff(pts_m, 1, 1), 2, 2));
            % Sample 3 representative points (start, middle, end) instead
            % of every segment vertex; the per-waypoint audit at the end
            % of plan_path still uses every path point at full precision.
            n_pts = size(pts, 1);
            if n_pts <= 3
                sample_idx = 1:n_pts;
            else
                sample_idx = [1, ceil(n_pts / 2), n_pts];
            end
            min_clearance = inf;
            for k = sample_idx
                c = obj.bank_clearance(pts(k, :));
                if c < min_clearance
                    min_clearance = c;
                end
            end
            threshold = obj.vessel_beam / 2 + obj.safety_margin_m;
            shortfall = max(0, threshold - min_clearance);
            w = length_m + obj.k_bank * length_m / max(eps, min_clearance) + obj.k_bank * length_m * shortfall;
        end

        function obj = checkAndConnectComponents(obj)
            % This function checks if the graph has multiple connected
            % components and connects them with synthetic edges so the
            % planner can route across chart-tile edges and similar gaps.
            local_bridge_threshold_m = 100;

            n_nodes = size(obj.unique_coords, 1);
            coords_m = (obj.unique_coords - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
            bins = conncomp(obj.G);

            if n_nodes >= 2
                K = min(n_nodes, 6);
                [idx_knn, dist_knn] = knnsearch(coords_m, coords_m, 'K', K);
                new_pairs = zeros(0, 2);
                new_dists = zeros(0, 1);
                for i = 1:n_nodes
                    for k = 2:K
                        j = idx_knn(i, k);
                        d_m = dist_knn(i, k);
                        if i < j && bins(i) ~= bins(j) && d_m < local_bridge_threshold_m
                            new_pairs(end+1, :) = [i, j];
                            new_dists(end+1, 1) = d_m;
                        end
                    end
                end
                if ~isempty(new_pairs)
                    obj.edges = [obj.edges; new_pairs];
                    obj.edge_weights = [obj.edge_weights; new_dists];
                    obj.G = graph(obj.edges(:, 1), obj.edges(:, 2), obj.edge_weights);
                    bins = conncomp(obj.G);
                end
            end

            num_components = max(bins);
            while num_components > 1
                min_dist = inf;
                min_pair = [];
                for i = 1:num_components
                    for j = i+1:num_components
                        nodes_i = find(bins == i);
                        nodes_j = find(bins == j);
                        for u = nodes_i
                            for v = nodes_j
                                d = pdist2(obj.unique_coords(u, :), obj.unique_coords(v, :));
                                if d < min_dist
                                    min_dist = d;
                                    min_pair = [u, v];
                                end
                            end
                        end
                    end
                end
                connect_diff_m = (obj.unique_coords(min_pair(2), :) - obj.unique_coords(min_pair(1), :)) .* obj.bank_m_per_deg;
                connect_w = norm(connect_diff_m);
                obj.edges = [obj.edges; min_pair];
                obj.edge_weights = [obj.edge_weights; connect_w];
                obj.G = graph(obj.edges(:, 1), obj.edges(:, 2), obj.edge_weights);
                bins = conncomp(obj.G);
                num_components = max(bins);
            end
        end

        function [obj, start_segment, end_segment,start_point,end_point] = findNearestPoints(obj, given_point1, given_point2)
            % Find nearest points method
            segment_indices = [];
            for i = 1:length(obj.segments)
                segment = obj.segments{i};
                num_points = length(segment.x);
                segment_indices = [segment_indices; repmat(i, num_points, 1)];
            end
            [start_point, start_dist] = knnsearch(obj.all_points, given_point1);
            [end_point, end_dist] = knnsearch(obj.all_points, given_point2);
            start_segment = segment_indices(start_point);
            end_segment = segment_indices(end_point);
            
        end

        function obj = planDirectPath(obj, segment, start_point, end_point)
            % Plan direct path method
            start_point_coords = obj.all_points(start_point, :);
            end_point_coords = obj.all_points(end_point, :); 
            
            all_segment_points = [obj.segments{segment}.x', obj.segments{segment}.y'];
            start_point_idx = find(ismember(all_segment_points, start_point_coords, 'rows'));
            end_point_idx = find(ismember(all_segment_points, end_point_coords, 'rows'));
 
            if start_point_idx < end_point_idx
                obj.path_points = all_segment_points(start_point_idx:end_point_idx, :);
            else
                obj.path_points = all_segment_points(start_point_idx:-1:end_point_idx, :);
            end
            [direct_path_depths, direct_path_points]=obj.segmentDepthRecognition_ForStartAndEnd(segment, obj.path_points);
            obj.path_points = direct_path_points;
            obj.path_depths = direct_path_depths;
        end

        function obj = planShortestPath(obj, start_segment, end_segment, start_point, end_point)
            % This function plans the shortest path between two points that are on different segments.
            % It computes the shortest path using Dijkstra's algorithm. 
            % It also performs depth recognition for each segment of the path.
            
            % Find the actual starting point in the starting point section
            start_point_coords = obj.all_points(start_point, :);
            end_point_coords = obj.all_points(end_point, :); 
            % Extracts the data from the structure array and converts it to an Nx2 matrix
            start_all_segment_points = [obj.segments{start_segment}.x', obj.segments{start_segment}.y'];
            end_all_segment_points = [obj.segments{end_segment}.x', obj.segments{end_segment}.y'];
            start_point_idx = find(ismember(start_all_segment_points, start_point_coords, 'rows'));
            end_point_idx = find(ismember(end_all_segment_points, end_point_coords, 'rows'));
            
            start_segment_points = obj.segments{start_segment};
            end_segment_points = obj.segments{end_segment};

            start_node1 = [start_segment_points.x(1), start_segment_points.y(1)];
            start_node2 = [start_segment_points.x(end), start_segment_points.y(end)];
            end_node1 = [end_segment_points.x(1), end_segment_points.y(1)];
            end_node2 = [end_segment_points.x(end), end_segment_points.y(end)];

            start_index1 = find(ismember(obj.unique_coords, start_node1, 'rows'));
            start_index2 = find(ismember(obj.unique_coords, start_node2, 'rows'));
            end_index1 = find(ismember(obj.unique_coords, end_node1, 'rows'));
            end_index2 = find(ismember(obj.unique_coords, end_node2, 'rows'));

            % Pick the start/end-segment endpoint pair whose Dijkstra
            % cost (length + bank-clearance penalty, computed in buildGraph) is smallest.
            path_indices = [
                start_index1, end_index1;
                start_index1, end_index2;
                start_index2, end_index1;
                start_index2, end_index2
            ];

            min_cost = inf;
            best_path = [];
            best_path_indices = [];
            for i = 1:size(path_indices, 1)
                [path, cost] = shortestpath(obj.G, path_indices(i, 1), path_indices(i, 2));
                if cost < min_cost
                    min_cost = cost;
                    best_path = path;
                    best_path_indices = path_indices(i, :);
                end
            end
            obj.best_path = best_path;

            start_all_segment_points = [obj.segments{start_segment}.x', obj.segments{start_segment}.y'];
            end_all_segment_points = [obj.segments{end_segment}.x', obj.segments{end_segment}.y'];

            start_path_index = find(ismember(start_all_segment_points, obj.unique_coords(best_path_indices(1), :), 'rows'));
            end_path_index = find(ismember(end_all_segment_points, obj.unique_coords(best_path_indices(2), :), 'rows'));

            if start_point_idx < start_path_index
                start_path = start_all_segment_points(start_point_idx:start_path_index, :);
            else
                start_path = start_all_segment_points(start_point_idx:-1:start_path_index, :);
            end

            if end_path_index < end_point_idx
                end_path = end_all_segment_points(end_path_index:end_point_idx, :);
            else
                end_path = end_all_segment_points(end_path_index:-1:end_point_idx, :);
            end

            [start_path_depths, start_path_points] = obj.segmentDepthRecognition_ForStartAndEnd(start_segment,start_path);
            [end_path_depths, end_path_points] = obj.segmentDepthRecognition_ForStartAndEnd(end_segment,end_path);
            obj.path_points = [start_path_points]; % Reset path_points to build it correctly
            obj.path_depths = [start_path_depths];

            for i = 1:length(obj.best_path)-1
                current_node = obj.best_path(i);
                next_node = obj.best_path(i+1);

                % Find the segment that connects these nodes
                found_segment = false;
                for j = 1:length(obj.segments)
                    segment = obj.segments{j};
                    start_node = find(ismember(obj.unique_coords, [segment.x(1), segment.y(1)], 'rows'));
                    end_node = find(ismember(obj.unique_coords, [segment.x(end), segment.y(end)], 'rows'));
                    if (start_node == current_node && end_node == next_node)
                        segment_points = [segment.x(:), segment.y(:)];
                    elseif (start_node == next_node && end_node == current_node)
                        segment_points = flipud([segment.x(:), segment.y(:)]);
                    else
                        continue;
                    end
                    segment_name = segment.region;
                    [segment_polygon_depths, segment_points_path] = obj.segmentDepthRecognition_ForLoop(segment_name, segment_points);
                    obj.path_points = [obj.path_points; segment_points_path];
                    obj.path_depths = [obj.path_depths; segment_polygon_depths];
                    found_segment = true;
                    break;
                end
                if ~found_segment
                    % As a fallback, if no segment is found, just connect the nodes directly
                    next_coord = obj.unique_coords(next_node, :);
                    obj.path_points = [obj.path_points; next_coord];
                    obj.path_depths = [obj.path_depths; NaN];
                end
            end
            obj.path_points = [obj.path_points; end_path_points];
            obj.path_depths = [obj.path_depths; end_path_depths];
            obj = obj.removeDuplicatePoints();
        end

        function obj = pullToCenter(obj)
            % This function iteratively shifts each interior path point (except the first and last) 
            % toward the midpoint of its two nearest points on the bank.
            if isempty(obj.bank_seg_starts_m) || size(obj.path_points, 1) < 3
                return;
            end
            n = size(obj.path_points, 1);
            edge_skip = 3;
            k_lo = max(2, edge_skip + 1);
            k_hi = min(n - 1, n - edge_skip);
            if k_lo > k_hi
                return;
            end
            for iter = 1:20
                max_shift_m = 0;
                for k = k_lo:k_hi
                    p_lonlat = obj.path_points(k, :);
                    p_m = (p_lonlat - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;

                    v = obj.bank_seg_ends_m - obj.bank_seg_starts_m;
                    w = p_m - obj.bank_seg_starts_m;
                    seg_len_sq = sum(v .* v, 2);
                    seg_len_sq(seg_len_sq < eps) = eps;
                    t = sum(w .* v, 2) ./ seg_len_sq;
                    t = max(0, min(1, t));
                    proj = obj.bank_seg_starts_m + t .* v;
                    diffs = p_m - proj;
                    dists = vecnorm(diffs, 2, 2);

                    [d1, idx1] = min(dists);
                    if d1 < eps
                        continue;
                    end
                    dir1 = diffs(idx1, :) / d1;

                    dir_from_bank = diffs ./ max(dists, eps);
                    cos_opp = dir_from_bank * dir1.';
                    opposite_mask = cos_opp < -0.3;
                    opposite_mask(idx1) = false;
                    if ~any(opposite_mask)
                        continue;
                    end
                    opposite_dists = dists;
                    opposite_dists(~opposite_mask) = inf;
                    [~, idx2] = min(opposite_dists);

                    midpoint_m = (proj(idx1, :) + proj(idx2, :)) / 2;
                    shift_m = (midpoint_m - p_m) * 0.5;
                    new_lonlat = obj.path_points(k, :) + shift_m ./ obj.bank_m_per_deg;
                    % Reject any shift that would land the point outside
                    % the navigable area, this can happen at narrow pinch points.
                    if ~isempty(obj.depare_polyshape.Vertices) && ...
                            ~isinterior(obj.depare_polyshape, new_lonlat(1), new_lonlat(2))
                        continue;
                    end
                    if norm(shift_m) > max_shift_m
                        max_shift_m = norm(shift_m);
                    end
                    obj.path_points(k, :) = new_lonlat;
                end
                if max_shift_m < 0.1
                    break;
                end
            end
        end

        function obj = densifyPath(obj)
            % This function densifies the path by adding intermediate points between
            % existing path points if the distance between them exceeds max_segment_length_m.
            if size(obj.path_points, 1) < 2 || obj.max_segment_length_m <= 0
                return;
            end
            max_len = obj.max_segment_length_m;
            seg_m = vecnorm(diff(obj.path_points) .* obj.bank_m_per_deg, 2, 2);
            n_segs = numel(seg_m);
            n_extra = max(0, ceil(seg_m / max_len) - 1);
            if all(n_extra == 0)
                return;
            end
            sync_depths = numel(obj.path_depths) == size(obj.path_points, 1);
            new_pts = cell(n_segs + 1, 1);
            new_dep = cell(n_segs + 1, 1);
            for k = 1:n_segs
                if n_extra(k) > 0
                    t = (1:n_extra(k)).' / (n_extra(k) + 1);
                    p0 = obj.path_points(k, :);
                    p1 = obj.path_points(k+1, :);
                    new_pts{k} = [p0; p0 + t .* (p1 - p0)];
                    if sync_depths
                        d0 = obj.path_depths(k);
                        d1 = obj.path_depths(k+1);
                        new_dep{k} = [d0; d0 + t * (d1 - d0)];
                    end
                else
                    new_pts{k} = obj.path_points(k, :);
                    if sync_depths
                        new_dep{k} = obj.path_depths(k);
                    end
                end
            end
            new_pts{end} = obj.path_points(end, :);
            obj.path_points = vertcat(new_pts{:});
            if sync_depths
                new_dep{end} = obj.path_depths(end);
                obj.path_depths = vertcat(new_dep{:});
            end
        end

        function obj = removeDuplicatePoints(obj)
            % Remove duplicate points from path_points, allowing each point to appear only once
            % and maintaining the original order of appearance
        
            [~, unique_indices] = unique(obj.path_points, 'rows', 'stable');
            
            obj.path_points = obj.path_points(unique_indices, :);
            obj.path_depths = obj.path_depths(unique_indices, :);
        end
        
            function path_points = removeDuplicatePoints_path(obj, path_points)
            % Remove duplicate points from path_points, allowing each point to appear only once
            % and maintaining the original order of appearance
        
            [~, unique_indices] = unique(path_points, 'rows', 'stable');
            
            path_points = path_points(unique_indices, :);
        end
        
            function path_points = smoothPoints_path(obj, path_points)
            % This function smooths the path points by performing linear interpolation.
            % It calculates the distances between consecutive points, computes the 
            % cumulative distance along the path, and then generates new, evenly spaced 
            % points based on a specified lookahead distance. The original path points 
            % are replaced with these new interpolated points.
            if size(path_points, 1) == 1
                return;
            end

            x = path_points(:,1); 
            y = path_points(:,2); 
            distances = sqrt(diff(x).^2 + diff(y).^2);
            total_distance = sum(distances);
            cumulative_distances = [0; cumsum(distances)];

            % Set the lookahead distance, which determines the spacing between new path points
            lookahead_distance = 0.001; 

            % if all_distance < 0.001
            if total_distance < lookahead_distance
                num_samples = 1;  % at least 1 point (start_point)
            else
                num_samples = floor(total_distance / lookahead_distance);
            end

            uniform_distances = linspace(0, total_distance, num_samples);
%             xx = spline(cumulative_distances, x, uniform_distances);
%             yy = spline(cumulative_distances, y, uniform_distances);
            xx = interp1(cumulative_distances, x, uniform_distances);
            yy = interp1(cumulative_distances, y, uniform_distances);
            path_points = zeros(length(xx), 2);
            path_points(:,1) = xx;
            path_points(:,2) = yy;
        end
        

            function [nan_path_depths, nan_path_points] = segmentDepthRecognition_ForStartAndEnd(obj, nan_segment, nan_path)
            % Perform smoothing and depth discrimination on the start points and end points. Output the smoothed points and depth.
            segment_name = obj.segments{nan_segment}.region;
%             disp(['Start segment belongs to: ', segment_name]);

            nan_path_points=obj.removeDuplicatePoints_path(nan_path);
            nan_path_points=obj.smoothPoints_path(nan_path_points);
            nan_path_depths = nan(size(nan_path_points, 1), 1);
            
            depare_index = find(strcmp({obj.pgon_memory.name}, 'depare'));
            
            if ~isempty(depare_index)
                depare_info = obj.pgon_memory(depare_index).info;
        
                matching_indices = find(strcmp({depare_info.sourceFile}, segment_name));
                
                for k = matching_indices
                    polygon_unit = depare_info(k).polygon;
                    bbox = depare_info(k).boundingbox;
        
                    in_bbox = all(nan_path_points >= bbox(1,:), 2) & all(nan_path_points <= bbox(2,:), 2);
                    
                   if any(in_bbox)
                        bbox_indices = find(in_bbox);
                        for idx = bbox_indices'
                            if isinterior(polygon_unit, nan_path_points(idx, 1), nan_path_points(idx, 2))
                                depth_value = depare_info(k).SOUACC;
                                if isnan(depth_value)
                                    depth_value = depare_info(k).VERDAT;
                                end
                                
                                nan_path_depths(idx) = depth_value;
                                
%                                 disp(['Point at index ', num2str(idx), ' of the start path belongs to a polygon with depth: ', num2str(depth_value)]);
                            end
                        end
                   end
                end
            end
            end        

            function [nan_path_depths, nan_path_points] = segmentDepthRecognition_ForLoop(obj, nan_segment_name, nan_path)
            % Perform smoothing and depth discrimination on the mid-segment points. Output the smoothed points and depth.

            depare_index = find(strcmp({obj.pgon_memory.name}, 'depare'));
            if ~isempty(depare_index)
                depare_info = obj.pgon_memory(depare_index).info;
                nan_path_points=obj.removeDuplicatePoints_path(nan_path);
                nan_path_points=obj.smoothPoints_path(nan_path_points);
                nan_path_depths = nan(size(nan_path_points, 1), 1);

                matching_indices = find(strcmp({depare_info.sourceFile}, nan_segment_name));
                
                for k = matching_indices
                    polygon_unit = depare_info(k).polygon;
                    bbox = depare_info(k).boundingbox;
        
                    in_bbox = all(nan_path_points >= bbox(1,:), 2) & all(nan_path_points <= bbox(2,:), 2);
                    
                   if any(in_bbox)
                        bbox_indices = find(in_bbox);
                        for idx = bbox_indices'
                            if isinterior(polygon_unit, nan_path_points(idx, 1), nan_path_points(idx, 2))
                                depth_value = depare_info(k).SOUACC;
                                if isnan(depth_value)
                                    depth_value = depare_info(k).VERDAT;
                                end
                                
                                nan_path_depths(idx) = depth_value;
                                
%                                 disp(['Point at index ', num2str(idx), ' of the start path belongs to a polygon with depth: ', num2str(depth_value)]);
                            end
                        end
                   end
                end
            end
            end

        function obj = attachLiteralEndpoints(obj, use_literal_start, use_literal_end)
            % This function stitches the user-clicked coordinates onto the planned path as
            % the literal first/last waypoint. 
            if use_literal_start
                junction_idx = obj.findJunctionIndices();
                keep_from = obj.chooseStartJunction(junction_idx);
                if ~obj.isLineInDepare(obj.given_point1, obj.path_points(keep_from, :))
                    keep_from = 1;
                end
                start_depth = obj.lookupDepth(obj.given_point1);
                obj.path_points = [obj.given_point1; obj.path_points(keep_from:end, :)];
                obj.path_depths = [start_depth; obj.path_depths(keep_from:end, :)];
            end
            if use_literal_end
                junction_idx = obj.findJunctionIndices();
                keep_to = obj.chooseEndJunction(junction_idx);
                if ~obj.isLineInDepare(obj.path_points(keep_to, :), obj.given_point2)
                    keep_to = size(obj.path_points, 1);
                end
                end_depth = obj.lookupDepth(obj.given_point2);
                obj.path_points = [obj.path_points(1:keep_to, :); obj.given_point2];
                obj.path_depths = [obj.path_depths(1:keep_to, :); end_depth];
            end
        end

        function keep_from = chooseStartJunction(obj, junction_idx)
            % Pick the junction whose direction from given_point1 best
            % aligns with the overall path direction (given_point1 ->
            % given_point2) and whose prepended straight segment stays
            % inside the navigable area.
            n_path = size(obj.path_points, 1);
            if isempty(junction_idx)
                keep_from = n_path;
                return;
            end
            overall = obj.given_point2 - obj.given_point1;
            overall_unit = overall / max(norm(overall), eps);
            n_j = length(junction_idx);
            alignments = -inf(n_j, 1);
            for p = 1:n_j
                candidate = junction_idx(p);
                v_in = obj.path_points(candidate, :) - obj.given_point1;
                v_in_unit = v_in / max(norm(v_in), eps);
                alignments(p) = dot(v_in_unit, overall_unit);
            end
            [~, order] = sort(alignments, 'descend');
            for k = 1:n_j
                p = order(k);
                if alignments(p) < 0
                    break;
                end
                candidate = junction_idx(p);
                if obj.isLineInDepare(obj.given_point1, obj.path_points(candidate, :))
                    keep_from = candidate;
                    return;
                end
            end
            keep_from = junction_idx(1);
        end

        function keep_to = chooseEndJunction(obj, junction_idx)
            % Mirror of chooseStartJunction for the end: pick the
            % junction whose direction toward given_point2 best aligns
            % with the overall path direction and whose appended
            % straight segment stays inside the navigable area.
            if isempty(junction_idx)
                keep_to = 1;
                return;
            end
            overall = obj.given_point2 - obj.given_point1;
            overall_unit = overall / max(norm(overall), eps);
            n_j = length(junction_idx);
            alignments = -inf(n_j, 1);
            for p = 1:n_j
                candidate = junction_idx(p);
                v_out = obj.given_point2 - obj.path_points(candidate, :);
                v_out_unit = v_out / max(norm(v_out), eps);
                alignments(p) = dot(v_out_unit, overall_unit);
            end
            [~, order] = sort(alignments, 'descend');
            for k = 1:n_j
                p = order(k);
                if alignments(p) < 0
                    break;
                end
                candidate = junction_idx(p);
                if obj.isLineInDepare(obj.path_points(candidate, :), obj.given_point2)
                    keep_to = candidate;
                    return;
                end
            end
            keep_to = junction_idx(end);
        end

        function junction_idx = findJunctionIndices(obj)
            % Find indices in path_points whose coordinates match a junction
            % node in unique_coords
            if isempty(obj.path_points) || isempty(obj.unique_coords)
                junction_idx = [];
                return;
            end
            tol = 1e-9;
            distances = pdist2(obj.path_points, obj.unique_coords);
            junction_idx = find(min(distances, [], 2) < tol);
        end

        function depth = lookupDepth(obj, point)
            % Look up the charted depth at an arbitrary (lon, lat) point by
            % checking containment against every depare polygon. Returns
            % NaN when the point is not inside any depare polygon.
            depth = NaN;
            depare_index = find(strcmp({obj.pgon_memory.name}, 'depare'));
            if isempty(depare_index)
                return;
            end
            depare_info = obj.pgon_memory(depare_index).info;
            for k = 1:numel(depare_info)
                bbox = depare_info(k).boundingbox;
                if all(point >= bbox(1, :)) && all(point <= bbox(2, :)) ...
                        && isinterior(depare_info(k).polygon, point(1), point(2))
                    depth_value = depare_info(k).SOUACC;
                    if isnan(depth_value)
                        depth_value = depare_info(k).VERDAT;
                    end
                    depth = depth_value;
                    return;
                end
            end
        end

        function obj = prepareBankGeometry(obj)
            verts = obj.polygon;
            if isempty(verts)
                obj.bank_anchor_lonlat = [0, 0];
                obj.bank_m_per_deg = [111320, 111320];
                obj.bank_seg_starts_m = zeros(0, 2);
                obj.bank_seg_ends_m = zeros(0, 2);
                obj.depare_polyshape = polyshape();
                return;
            end
            obj.depare_polyshape = polyshape(verts);

            valid_rows = ~any(isnan(verts), 2);
            anchor = mean(verts(valid_rows, :), 1);
            m_per_deg_lat = 111320;
            m_per_deg_lon = 111320 * cos(deg2rad(anchor(2)));
            obj.bank_anchor_lonlat = anchor;
            obj.bank_m_per_deg = [m_per_deg_lon, m_per_deg_lat];

            % Build banks from the outer boundaries of depare only.
            % Skip rings that polyshape identifies as holes. These are typically bridges
            %  or other internal cut-outs that should not be treated as banks).
            
            seg_starts = zeros(0, 2);
            seg_ends   = zeros(0, 2);
            for b = 1:numboundaries(obj.depare_polyshape)
                if ishole(obj.depare_polyshape, b)
                    continue;
                end
                ring = obj.depare_polyshape.boundary(b);
                if size(ring, 1) < 3
                    continue;
                end
                next_ring = [ring(2:end, :); ring(1, :)];
                seg_starts = [seg_starts; ring];
                seg_ends   = [seg_ends;   next_ring];
            end

            seg_starts_m = (seg_starts - anchor) .* obj.bank_m_per_deg;
            seg_ends_m   = (seg_ends   - anchor) .* obj.bank_m_per_deg;
            obj.bank_seg_starts_m = seg_starts_m;
            obj.bank_seg_ends_m   = seg_ends_m;
        end

        function clearance_m = bank_clearance(obj, point_lonlat)
            if isempty(obj.bank_seg_starts_m)
                clearance_m = inf;
                return;
            end
            p_m = (point_lonlat - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
            v = obj.bank_seg_ends_m - obj.bank_seg_starts_m;
            w = p_m - obj.bank_seg_starts_m;
            seg_len_sq = sum(v .* v, 2);
            seg_len_sq(seg_len_sq < eps) = eps;
            t = sum(w .* v, 2) ./ seg_len_sq;
            t = max(0, min(1, t));
            proj = obj.bank_seg_starts_m + t .* v;
            clearance_m = min(vecnorm(p_m - proj, 2, 2));
        end

        function ok = isLineInDepare(obj, p1_lonlat, p2_lonlat)
            % Sample the straight line p1->p2 at ~10 m spacing and check
            % that the line is mostly inside depare_polyshape. 
            if isempty(obj.depare_polyshape.Vertices)
                ok = true;
                return;
            end
            diff_m = (p2_lonlat - p1_lonlat) .* obj.bank_m_per_deg;
            len_m = norm(diff_m);
            n_samp = max(2, ceil(len_m / 10));
            ts = linspace(0, 1, n_samp)';
            samples = p1_lonlat + ts .* (p2_lonlat - p1_lonlat);
            in = isinterior(obj.depare_polyshape, samples(:, 1), samples(:, 2));
            n_out = sum(~in);
            % Allow up to 2 samples (~20 m total) outside depare. Catches
            % small chart-edge gaps and pinch points without permitting
            % real land crossings on long shortcuts.
            ok = n_out <= 2;
        end

        function obj = smoothArcsAtCorners(obj)
            % This function identifies junction points in the path and replaces sharp corners 
            % with circular arcs of radius R_min. The new arc points are integrated into the path
            % by replacing the original corner point.
            if obj.R_min <= 0 || size(obj.path_points, 1) < 3
                return;
            end
            junction_idx = obj.findJunctionIndices();
            n = size(obj.path_points, 1);
            interior = junction_idx(junction_idx > 1 & junction_idx < n);
            if isempty(interior)
                return;
            end

            new_points = obj.path_points;
            new_depths = obj.path_depths;
            arc_step_m = 10;

            for k = numel(interior):-1:1
                corner_idx = interior(k);
                if corner_idx < 2 || corner_idx > size(new_points, 1) - 1
                    continue;
                end
                p_prev_m   = (new_points(corner_idx - 1, :) - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
                p_corner_m = (new_points(corner_idx,     :) - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
                p_next_m   = (new_points(corner_idx + 1, :) - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
                d_in_raw  = p_corner_m - p_prev_m;
                d_out_raw = p_next_m   - p_corner_m;
                len_in  = norm(d_in_raw);
                len_out = norm(d_out_raw);
                if len_in < eps || len_out < eps
                    continue;
                end
                d_in  = d_in_raw  / len_in;
                d_out = d_out_raw / len_out;
                cos_a = max(-1, min(1, dot(d_in, d_out)));
                alpha = acos(cos_a);
                if alpha < deg2rad(1)
                    continue;
                end
                T = obj.R_min * tan(alpha / 2);

                last_kept_before = 1;
                for i = corner_idx - 1:-1:1
                    p_i_m = (new_points(i, :) - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
                    if dot(p_corner_m - p_i_m, d_in) >= T
                        last_kept_before = i;
                        break;
                    end
                end
                n_now = size(new_points, 1);
                first_kept_after = n_now;
                for i = corner_idx + 1:1:n_now
                    p_i_m = (new_points(i, :) - obj.bank_anchor_lonlat) .* obj.bank_m_per_deg;
                    if dot(p_i_m - p_corner_m, d_out) >= T
                        first_kept_after = i;
                        break;
                    end
                end

                p_in_t_m  = p_corner_m - T * d_in;
                p_out_t_m = p_corner_m + T * d_out;

                cross_z = d_in(1) * d_out(2) - d_in(2) * d_out(1);
                if cross_z >= 0
                    n_in = [-d_in(2),  d_in(1)];
                else
                    n_in = [ d_in(2), -d_in(1)];
                end
                center_m = p_in_t_m + obj.R_min * n_in;

                v_in  = p_in_t_m  - center_m;
                v_out = p_out_t_m - center_m;
                theta_in  = atan2(v_in(2),  v_in(1));
                theta_out = atan2(v_out(2), v_out(1));
                if cross_z >= 0
                    d_theta = theta_out - theta_in;
                    if d_theta < 0
                        d_theta = d_theta + 2 * pi;
                    end
                else
                    d_theta = theta_out - theta_in;
                    if d_theta > 0
                        d_theta = d_theta - 2 * pi;
                    end
                end

                arc_length = abs(d_theta) * obj.R_min;
                n_arc = max(2, ceil(arc_length / arc_step_m));
                thetas = linspace(theta_in, theta_in + d_theta, n_arc)';
                arc_samples_m = center_m + obj.R_min * [cos(thetas), sin(thetas)];
                arc_samples_lonlat = arc_samples_m ./ obj.bank_m_per_deg + obj.bank_anchor_lonlat;

                % Reject the arc if any sample leaves the depare
                % polygon and keep the original corner instead.
                if ~isempty(obj.depare_polyshape.Vertices) && ...
                        ~all(isinterior(obj.depare_polyshape, arc_samples_lonlat(:, 1), arc_samples_lonlat(:, 2)))
                    continue;
                end

                arc_depths = zeros(n_arc, 1);
                for s = 1:n_arc
                    arc_depths(s) = obj.lookupDepth(arc_samples_lonlat(s, :));
                end

                new_points = [new_points(1:last_kept_before, :); arc_samples_lonlat; new_points(first_kept_after:end, :)];
                new_depths = [new_depths(1:last_kept_before);    arc_depths;          new_depths(first_kept_after:end)];
            end

            obj.path_points = new_points;
            obj.path_depths = new_depths;
        end

        function obj = auditPathClearance(obj)
            % This function audits the bank clearance of each waypoint in the
            % planned path and issues a warning if any waypoint has clearance
            % below the threshold (vessel_beam/2 + safety_margin_m).
            n = size(obj.path_points, 1);
            if n == 0
                obj.path_clearances = zeros(0, 1);
                return;
            end
            clearances = zeros(n, 1);
            for k = 1:n
                clearances(k) = obj.bank_clearance(obj.path_points(k, :));
            end
            obj.path_clearances = clearances;

            threshold = obj.vessel_beam / 2 + obj.safety_margin_m;
            [min_c, idx] = min(clearances);
            if min_c < threshold
                warning('maps:planner:bankClearance', ...
                    ['Planned path waypoint %d has bank clearance %.2f m, ' ...
                     'below threshold %.2f m (vessel_beam/2 + safety_margin_m). ' ...
                     'Position [lon lat] = [%.6f %.6f].'], ...
                    idx, min_c, threshold, ...
                    obj.path_points(idx, 1), obj.path_points(idx, 2));
            end
        end

        function in_polygon = is_point_inside_polygon(obj, given_point)
            in_polygon = isinterior(polyshape(obj.polygon), given_point(1), given_point(2));
        end

        function in_polygon_land = is_point_inside_polygon_land(obj, given_point)
            in_polygon_land = isinterior(polyshape(obj.polygon_land), given_point(1), given_point(2));
        end
    end
end
