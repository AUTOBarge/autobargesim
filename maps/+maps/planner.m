classdef planner
    % ClassName: maps.planner
    %
    % Descirption:
    % The planner class is used to generate paths within a polygon area.
    % This class includes functionalities to randomly generate start and end points,
    % build a graph, compute the best path and way points, and plot.
    %
    % Properties:
    %   - pgon_memory: Structure storing shp.file data
    %   - segments: Cell array of segment data
    %   - G: Graph object representing the connectivity between segments
    %   - unique_coords: Matrix of unique coordinates of the nodes in graph G
    %   - edges: List of edges of the graph
    %   - best_path: Best path node indices
    %   - path_points: Coordinate points of the best path
    %   - given_point1: Start point coordinates
    %   - given_point2: End point coordinates
    %   - all_points: Collection of all segment coordinate points
    %   - polygon: Vertices of the polygon
    %
    % Author:
    %   Zhongbi Luo
    %
    % Date:
    %   2024-05-26
    %
    % Version:
    %   1.0

    
    properties
        pgon_memory
        segments
        G
        unique_coords
        edges
        best_path
        path_points
        given_point1
        given_point2
        all_points
        polygon
    end
    
     methods
        function obj = planner(pgon_memory)
            % planner constructor
            obj.pgon_memory = pgon_memory;
            obj = obj.extractData();
            obj = obj.buildGraph();
            obj = obj.checkAndConnectComponents();
        end

        function obj = plan_path(obj, given_point1, given_point2)
            % Plan path method
            obj.given_point1 = given_point1;
            obj.given_point2 = given_point2;
            
            % Check if the given points are inside the polygon
            in_polygon1=obj.is_point_inside_polygon(given_point1);
            in_polygon2=obj.is_point_inside_polygon(given_point2);
            if in_polygon1 && in_polygon2
                disp('Starting point and ending point set successfully');
                disp(['Starting point: ', num2str(given_point1)]);
                disp(['Ending point: ', num2str(given_point2)]);
            else
                disp('The points are set outside the boundary, please reset.');
                if ~in_polygon1
                disp(['Starting point is outside the boundary: ', num2str(given_point1)]);
                end
                if ~in_polygon2
                disp(['Ending point is outside the boundary: ', num2str(given_point2)]);
                end
            end


            [obj, start_segment, end_segment, start_point, end_point] = obj.findNearestPoints(given_point1, given_point2);
            
            if start_segment == end_segment
            % Determine whether the start and end point are on the same segment
                obj = obj.planDirectPath(start_segment, start_point, end_point);
            else
                obj = obj.planShortestPath(start_segment, end_segment, start_point, end_point);
            end
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
                plot(obj.path_points(:, 1), obj.path_points(:, 2), 'k.', 'MarkerSize', 10);
                title('Graph Network with All Information');
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

            hold off;
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

            index2 = find(strcmp({obj.pgon_memory.name}, 'depare'));
            polygon_data = obj.pgon_memory(index2).polygons;
            obj.polygon = polygon_data.Vertices; 

            x = lines_data(1, :);
            y = lines_data(2, :);

            segments = {};
            start_idx = 1;
            for i = 1:length(x)
                if isnan(x(i))
                    if start_idx < i-1
                        segment.x = x(start_idx:i-1);
                        segment.y = y(start_idx:i-1);
                        % remove overlapped points
                        [unique_points, unique_indices] = unique([segment.x', segment.y'], 'rows', 'stable');
                        segment.x = unique_points(:, 1)';
                        segment.y = unique_points(:, 2)';
                        segments{end+1} = segment;
                    end
                    start_idx = i + 1;
                elseif i == length(x)
                    if start_idx <= i
                        segment.x = x(start_idx:i);
                        segment.y = y(start_idx:i);
                        % remove overlapped points
                        [unique_points, unique_indices] = unique([segment.x', segment.y'], 'rows', 'stable');
                        segment.x = unique_points(:, 1)';
                        segment.y = unique_points(:, 2)';
                        segments{end+1} = segment;
                    end
                end
            end
            obj.segments = segments;

            endpoints = [];
            all_points = [];
            for i = 1:length(segments)
                segment = segments{i};
                endpoints = [endpoints; segment.x(1), segment.y(1); segment.x(end), segment.y(end)];
                all_points = [all_points; [segment.x(:), segment.y(:)]];
            end
            obj.unique_coords = unique(endpoints, 'rows', 'stable');
            obj.all_points=all_points;
        end

        function obj = buildGraph(obj)
            % Build graph method
            % Constructs the graph object based on segment data
            edges = [];
            for i = 1:length(obj.segments)
                segment = obj.segments{i};
                start_node = [segment.x(1), segment.y(1)];
                end_node = [segment.x(end), segment.y(end)];
                start_index = find(ismember(obj.unique_coords, start_node, 'rows'));
                end_index = find(ismember(obj.unique_coords, end_node, 'rows'));
                edges = [edges; start_index, end_index];

            end
            obj.edges = edges;
            obj.G = graph(edges(:, 1), edges(:, 2));
        end

        function obj = checkAndConnectComponents(obj)
            % Check and connect components method
            % Checks the connectivity of the graph and connects different components
            bins = conncomp(obj.G);
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
                obj.edges = [obj.edges; min_pair];
                obj.G = graph(obj.edges(:, 1), obj.edges(:, 2));
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
        end

        function obj = planShortestPath(obj, start_segment, end_segment, start_point, end_point)
            % Plan shortest path method
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

            paths = {
                shortestpath(obj.G, start_index1, end_index1),
                shortestpath(obj.G, start_index1, end_index2),
                shortestpath(obj.G, start_index2, end_index1),
                shortestpath(obj.G, start_index2, end_index2)
            };

            path_indices = [
                start_index1, end_index1;
                start_index1, end_index2;
                start_index2, end_index1;
                start_index2, end_index2
            ];

            min_path_length = inf;
            best_path = [];
            best_path_indices = [];
            distances = pdist2(obj.unique_coords, obj.unique_coords);
            for i = 1:length(paths)
                path = paths{i};
                path_length = sum(distances(sub2ind(size(distances), path(1:end-1), path(2:end))));
                if path_length < min_path_length
                    min_path_length = path_length;
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

            obj.path_points = [start_path]; % Reset path_points to build it correctly
            for i = 1:length(obj.best_path)-1
                current_node = obj.best_path(i);
                next_node = obj.best_path(i+1);
        
                % Find the segment that connects these nodes
                for j = 1:length(obj.segments)
                    segment = obj.segments{j};
                    start_node = find(ismember(obj.unique_coords, [segment.x(1), segment.y(1)], 'rows'));
                    end_node = find(ismember(obj.unique_coords, [segment.x(end), segment.y(end)], 'rows'));
                    if (start_node == current_node && end_node == next_node)
                        % Direction is correct
                        segment_points = [segment.x(:), segment.y(:)];
                    elseif (start_node == next_node && end_node == current_node)
                        % Direction is reversed
                        segment_points = flipud([segment.x(:), segment.y(:)]);
                    else
                        continue;
                    end
                    obj.path_points = [obj.path_points; segment_points];
                    break;
                end
            end
            obj.path_points = [obj.path_points; end_path];
            obj = obj.removeDuplicatePoints();

        end
        
        function obj = removeDuplicatePoints(obj)
            % Remove duplicate points from path_points, allowing each point to appear only once
            % and maintaining the original order of appearance
        
            [~, unique_indices] = unique(obj.path_points, 'rows', 'stable');
            
            obj.path_points = obj.path_points(unique_indices, :);
        end

        function in_polygon = is_point_inside_polygon(obj, given_point)
            in_polygon = isinterior(polyshape(obj.polygon), given_point(1), given_point(2));
        end
    end
end
