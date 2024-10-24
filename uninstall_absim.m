function uninstall_absim()
    folders = {'system', 'control', 'guidance', 'maps', 'mission_planner', 'model&actuator','colav'};
    
    currentFolder = pwd;
    
    % Adding the folders and subfolders to the MATLAB path
    try
        fprintf('========= Uninstalling AUTOBargeSim Version: %s =========\n', absimver);
        for i = 1:length(folders)
            folderPath = fullfile(currentFolder, folders{i});
            if contains(path, folderPath)
                fprintf('Removing path: %s\n', folderPath);
                rmpath(genpath(folderPath));
            end
        end
        
        % Saving the path changes
        status = savepath;
        if status == 0
            fprintf('Path changes have been saved successfully.\n');
        else
            error('Failed to save the path changes. You may need administrator rights.');
        end
        
        fprintf('========= Uninstallation Successful =========\n');
    catch ME
        % Error handling
        fprintf('An error occurred: %s\n', ME.message);
        fprintf('Ensure you have the necessary permissions and that the folders exist.\n');
    end
end