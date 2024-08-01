classdef processor

    % ClassName: maps.processor
    %
    % Descirption:
    %   PROCESSOR Class for processing and visualizing shapefiles
    %   This class reads, processes, and plots shapefiles from a specified folder.
    %   It can handle points, lines, and polygons, organizing them by desired names.
    %
    % Properties:
    %   - folder: Folder containing the shapefiles
    %   - files: List of shapefiles in the folder
    %   - desirename: Desired names for categorizing the shapefiles
    %   - pgon_memory: Memory structure storing processed geometries
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
        folder       
        files        
        desirename   
        pgon_memory  
    end
    
    
    methods
        function obj = processor(desirename,folder)
            if nargin > 0
                % processor constructor
                obj.desirename = desirename;
                obj.folder = folder;
                obj.files=dir(fullfile(folder,'*.shp'));
                obj.pgon_memory = obj.set(desirename, obj.files, folder);
            else
                disp('No input arguments provided.');
            end
        end
        
        function pgon_memory = set(~, desirename, files, folder)
            % set method
            numDesireNames = numel(desirename); % Obtain the number of desiremane files
            pgon_memory = struct('name', cell(numDesireNames, 1), 'points', cell(numDesireNames, 1), 'lines', cell(numDesireNames, 1), 'polygons', cell(numDesireNames, 1)); % 初始化结构体数组
            
            for idx = 1:numDesireNames
                result = maps.processor.shape_multi(desirename, files, folder, desirename(idx));
                % store desirename
                pgon_memory(idx).name = desirename{idx};
                % Assuming maps.shape_multi also returns a struct, we assign each field of this struct to pgon_memory
                pgon_memory(idx).points = result.points;
                pgon_memory(idx).lines = result.lines;
                pgon_memory(idx).polygons = result.polygons;
            end
        end
        
        function plot(obj)
            % plot method
            for idx = 1:length(obj.pgon_memory)
                maps.processor.plotGeometries(obj.pgon_memory(idx));
            end
        end
    end

        methods (Access = private, Static)
            function result = shape_multi(desirename, files, folder, desirename_char)
                  categories = struct('name', {}, 'shpFiles', {});  % Create an empty structure array to store the categories
                for i = 1:length(files)
                    filename = files(i).name;  % Get the file name
                    [~, name, ~] = fileparts(filename);  % Extract the file name (without extension)
                    
                    % Check if the file name contains 'desirename'
                    for ii= 1:length(desirename)    
                        if contains(name, desirename(ii))
                            category = char(desirename(ii));
                    filePath = fullfile(folder, filename);  % Construct the full file path        
                    % Check if the category structure already exists
                    idx = find(strcmp({categories.name}, category));
                    if isempty(idx)  % If the category doesn't exist, add a new structure
                        idx = numel(categories) + 1;
                        categories(idx).name = category;
                        categories(idx).shpFiles = {filePath};
                    else  % If the category already exists, add the file path to the corresponding structure
                        categories(idx).shpFiles = [categories(idx).shpFiles, {filePath}];
                    end
                        else
                        continue;  % Skip the file if it doesn't match the desired categories
                        end
                    end
                end
                    for  uu = 1:length(categories)
                            if strcmp( categories(uu).name,char(desirename_char) )
                                A = categories(uu).shpFiles;
                                result = maps.processor.shapecompute_multi(A);
                            else
                                continue
                            end
                    end
            end

            function memory = shapecompute_multi(A) %output.structure "memory"
                        memory.points = [];
                        memory.lines = [];
                        memory.polygons = polyshape();
            for i=1:length(A) %Iterate through specific categories and determine points, lines and polygon
                mapdata=shaperead(char(A(1,i)));
                [n_mapdata,~]=size(mapdata);
                if n_mapdata == 0
                    continue
                end

                if strcmp(mapdata(1).Geometry, 'Point') 
                    index_polt = 1;          
                    num_lines = length(mapdata);
                    X = cell(1, num_lines);
                    Y = cell(1, num_lines);                  
                    for nn = 1:num_lines
                        X{nn} = mapdata(nn).X;
                        Y{nn} = mapdata(nn).Y;
                    end 
                    memory.points = [memory.points, [cell2mat(X); cell2mat(Y)]]; 
                end

                if strcmp(mapdata(1).Geometry, 'Line') 
                    index_polt = 2;          
                    num_lines = length(mapdata);
                    X = cell(1, num_lines);
                    Y = cell(1, num_lines);                  
                    for nn = 1:num_lines
                        X{nn} = mapdata(nn).X;
                        Y{nn} = mapdata(nn).Y;
                    end 
                    memory.lines = [memory.lines, [cell2mat(X); cell2mat(Y)]]; 
                end

                if strcmp( mapdata(1).Geometry,'Polygon' )
                    index_polt=3;
                    pgon = polyshape(mapdata(1).X, mapdata(1).Y);
                    if n_mapdata==1
                        memory.polygons=union(memory.polygons,pgon);
                    else
                    for i1 = 2:n_mapdata
                        pgon_x = polyshape(mapdata((i1)).X, mapdata((i1)).Y);
                        pgon = union(pgon,pgon_x);
                        memory.polygons=union(memory.polygons,pgon);
                    end
                    end
                end
            end
            end

            function plotGeometries(geom)
                axis auto;
                hold on;
                if isfield(geom, 'polygons') && ~isempty(geom.polygons.Vertices) && ~isempty(geom.polygons)
                    % plot polygons
                    plot(geom.polygons);
                end
                
                if isfield(geom, 'lines') && ~isempty(geom.lines)
                    % plot lines
                    line_pl = line(geom.lines(1,:), geom.lines(2,:));
                    line_pl.LineStyle = '--';
                    line_pl.LineWidth = 1;
                end
            
                if isfield(geom, 'points') && ~isempty(geom.points)
                    % plot points
                    sz = 5;
                    scatter(geom.points(1,:), geom.points(2,:), sz, 'MarkerEdgeColor', [0 .5 .5], ...
                        'MarkerFaceColor', [0 .7 .7], 'LineWidth', 1.5);
                end
                hold off;
                end
    end
           
end

