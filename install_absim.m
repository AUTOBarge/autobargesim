function install_absim()
%To install the toolbox, simply run this matlab file or type 'install_absim' in the MATLAB command window.
    clear;
    clc;

    fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
    fprintf('AUTOBargeSim: MATLAB toolbox for the design and analysis of \nthe guidance and control system for autonomous inland vessels.\n');
    fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');

    folders = {'system', 'control', 'guidance', 'maps', 'mission_planner', 'model&actuator','colav'};
    
    currentFolder = pwd;

    if ~isempty(absimroot)
        fprintf('WARNING: A previous version of AUTOBargeSim has been detected. The uninstallation of the previous version will be carried out automatically before proceeding with the new installation.\n');
        uninstall_absim;
    end

    % Adding the folders and subfolders to the MATLAB path
    try
        fprintf('========= Installing AUTOBargeSim =========\n');
        fprintf('Adding root: %s\n', absimroot);
        addpath(absimroot);
        for i = 1:length(folders)
            folderPath = fullfile(currentFolder, folders{i});
            fprintf('Adding path: %s\n', folderPath);
            addpath(genpath(folderPath));
        end
        fprintf('Folders and subfolders have been added to the MATLAB path.\n');
        
        % Adding third-party package (Casadi)
         casadiInstalled = input('Would you like to install the Casadi package? \n (Casadi is required for using some features of the AUTOBargeSim toolbox) (y/n): ', 's');
         if strcmpi(casadiInstalled, 'y')
			casadiPath = fullfile(currentFolder,'prerequisites','casadi','casadi-3.6.4-windows64-matlab2018b');
			addpath(genpath(casadiPath));
			fprintf('Casadi has been added to the MATLAB path.\n');
         else
            fprintf('Proceeding without installing the Casadi package... .\n');
         end
        
		% Checking for the installation of MATLAB Control System Toolbox
        if license('test', 'Control_Toolbox')
            fprintf('Control System Toolbox is available.\n');
        else
            error('Control System Toolbox is not available. Please install it first.');
        end
        
        % Saving the path changes
        status = savepath;
        if status == 0
            fprintf('Path changes have been saved successfully.\n');
        else
            error('Failed to save the path changes. You may need administrator rights.');
        end

        fprintf('========= Installation Successful! =========\n');
        
    catch ME
        % Error handling
        fprintf('An error occurred: %s\n', ME.message);
        fprintf('Ensure you have the necessary permissions and that the folders exist.\n');
    end
end
