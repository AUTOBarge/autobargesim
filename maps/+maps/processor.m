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
      function obj = processor(desirename, folder)
            if nargin > 0
                obj.desirename = desirename;
                obj.folder = folder;
                obj.files = dir(fullfile(folder, '*.shp'));
                
                % check mat file
                matFilePath = fullfile(folder, 'processed_data.mat');
                if isfile(matFilePath)
                    % load
                    loadedData = load(matFilePath);
                    
                    % consistency
                    matFileNames = {loadedData.files.name};
                    currentFileNames = {obj.files.name};
                    if isequal(sort(matFileNames), sort(currentFileNames)) && ...
                       isequal(loadedData.folder, folder) && ...
                       isequal(loadedData.desirename, desirename)
                        obj.pgon_memory = loadedData.pgon_memory;
                        disp('Loaded data from existing mat file.');
                    else
                        obj.pgon_memory = obj.set(desirename, obj.files, folder);
                        files = obj.files;
                        pgon_memory = obj.pgon_memory;
                        save(matFilePath, 'pgon_memory', 'files', 'folder', 'desirename');
                        disp('Processed files and saved data to mat file.');
                    end
                else
                    obj.pgon_memory = obj.set(desirename, obj.files, folder);
                 
                    files = obj.files;
                    pgon_memory = obj.pgon_memory;
                    save(matFilePath, 'pgon_memory', 'files', 'folder', 'desirename');
                    disp('Processed files and saved data to mat file.');
                end
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
                pgon_memory(idx).info = result.info;
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
                                result = maps.processor.shapecompute_multi(A, categories(uu).name);
                            else
                                continue
                            end
                    end
        end

        function memory = shapecompute_multi(A, categoryName)
            memory.points = [];
            memory.lines = [];
            memory.polygons = polyshape();
            memory.info = [];
            
            for i = 1:length(A)
                mapdata = shaperead(char(A(1, i)));
                [n_mapdata, ~] = size(mapdata);
                if n_mapdata == 0
                    continue
                end

                if strcmp(mapdata(1).Geometry, 'Point')
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
                    num_lines = length(mapdata);
                    X = cell(1, num_lines);
                    Y = cell(1, num_lines);                  
                    for nn = 1:num_lines
                        X{nn} = mapdata(nn).X;
                        Y{nn} = mapdata(nn).Y;
                            if strcmp(categoryName, 'wtwaxs')
                                fullSourceFile = char(A(1, i));
                                pattern = '(?<=output_)(\w+)(?=_wtwaxs)';
                                sourceFile = regexp(fullSourceFile, pattern, 'match', 'once');
                                memory.info = [memory.info, struct('X', mapdata(nn).X, 'Y', mapdata(nn).Y, 'sourceFile', sourceFile)];
                            end
                     end

                    memory.lines = [memory.lines, [cell2mat(X); cell2mat(Y)]]; 
                end

                if strcmp(mapdata(1).Geometry, 'Polygon')
                    warning('off', 'MATLAB:polyshape:repairedBySimplify');
                    for i1 = 1:n_mapdata
                        pgon_x = polyshape(mapdata(i1).X, mapdata(i1).Y);
                        if strcmp(categoryName, 'depare')

                                fullSourceFile = char(A(1, i));
                                pattern = '(?<=output_)(\w+)(?=_depare)';
                                sourceFile = regexp(fullSourceFile, pattern, 'match', 'once');
                                BoundingBox = mapdata(i1).BoundingBox;
                            if isfield(mapdata(i1), 'SOUACC')
                                souacc = mapdata(i1).SOUACC;
                            else
                                souacc = NaN;
                            end
                            if isfield(mapdata(i1), 'VERDAT')
                                verdat = mapdata(i1).VERDAT;
                            else
                                verdat = NaN;
                            end
                            % Store detailed info in the 'info' field
                            memory.info = [memory.info, struct('polygon', pgon_x, 'sourceFile', sourceFile, 'SOUACC', souacc, 'VERDAT', verdat, 'boundingbox', BoundingBox)];
                        end
                        memory.polygons = union(memory.polygons, pgon_x); % Store combined polygons
                    end
                end
            end
        end

            function plotGeometries(geom)
                % Defines the color mapping
                colorMap = containers.Map({'depare', 'bridge', 'wtwaxs', 'lndare'}, ...
                                      {[0.12, 0.56, 1.0], 'none', [0.75, 0.75, 0.75], [0.85, 0.65, 0.41]});
                axis auto;
                hold on;
   if isfield(geom, 'polygons') && ~isempty(geom.polygons.Vertices) && ~isempty(geom.polygons)
        category = geom.name;

        % Special handling for 'depare' category
        if strcmp(category, 'depare')
            info_data = geom.info;
            max_depth = max([info_data.SOUACC, info_data.VERDAT]);
            min_depth = 0; % Fixed as 0 to avoid negative normalization problems

            % Traverse each info_data structure
            for i = 1:numel(info_data)
                polygon = info_data(i).polygon;
                souacc = info_data(i).SOUACC;
                verdat = info_data(i).VERDAT;

                % Choose suitable color
                if ~isnan(souacc)
                    depth_value = souacc;
                elseif ~isnan(verdat)
                    depth_value = verdat;
                else
                    depth_value = NaN; % If there is no depth, set NAN
                end

                % Choose color based on the depth
                if ~isnan(depth_value)
                    if depth_value < 0
                        color = [0.85, 0.65, 0.13]; % Earthy yellow is used for negative values
                    else
                        
                        normalized_depth = (depth_value - min_depth) / (max_depth - min_depth);
                        color = [0, 1 - normalized_depth, 1]; % Blue base, the deeper, the darker the color
                    end
                else
                    color = [0.5, 0.5, 0.5]; % If there is no depth info, skip it using gray color
                end

                % plot polygon
                pgon_dp = plot(polygon, 'FaceColor', color, 'EdgeColor', 'none');
            end
        else
            % For other categories
            if isKey(colorMap, category)
                if strcmp(category, 'bridge')
                    pgon_bg = plot(geom.polygons, 'FaceColor', [0.75, 0.75, 0.75], 'FaceAlpha', 0.3);
                else
                    pgon_bg = plot(geom.polygons, 'FaceColor', colorMap(category));
                end
            else
                pgon_bg = plot(geom.polygons);
            end
        end
    end
            
            if isfield(geom, 'lines') && ~isempty(geom.lines)
                line_pl = line(geom.lines(1, :), geom.lines(2, :));
                line_pl.LineStyle = '--';
                line_pl.LineWidth = 1;
            end
        
            if isfield(geom, 'points') && ~isempty(geom.points)
                sz = 5;
                scatter(geom.points(1, :), geom.points(2, :), sz, 'MarkerEdgeColor', [0 .5 .5], ...
                    'MarkerFaceColor', [0 .7 .7], 'LineWidth', 1.5);
            end
            hold off;
                end
    end
           
end