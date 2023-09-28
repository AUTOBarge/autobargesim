<<<<<<< HEAD:maps/maps.m
classdef maps
=======
classdef ENC
>>>>>>> ffbecf63d90dfcd88c778b38b14097d2dd1060eb:Map Class/ENC_class/ENC.m
    

    methods (Static)
%%%%%%%%%%%%%%%%%%%%%%%%%%
    function pgon_memory = shape_multi(desirename,files,folder,desirename_char) %Multifile read
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
<<<<<<< HEAD:maps/maps.m
                        pgon_memory = maps.shapecompute_multi(A);
=======
                        pgon_memory = ENC.shapecompute_multi(A);
>>>>>>> ffbecf63d90dfcd88c778b38b14097d2dd1060eb:Map Class/ENC_class/ENC.m
                    else
                        continue
                    end
            end
           
    end
    end
%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private, Static)
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
    end
end


