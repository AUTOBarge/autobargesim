clear
clc

% Starting
disp('======== RELEASE LOG ========')
disp('Initializing the relase process...');

% Checklist
questions = {
    'Did you commit all files?', ...
    'Did you update the software version?', ...
    'Does the release folder name have the format: autobargesim-v{version_number}', ...
};
for i = 1:length(questions)
    userResponse = questdlg(questions{i}, 'Release checklist', 'Yes', 'No', 'Yes');
    
    if strcmp(userResponse, 'No')
        disp('Checklist failed! Existing release...');
        return
    end
end

% Copy all files and folders
disp('Copying files and folders to the release directory');
fromRepo = uigetdir(pwd, 'Select repository folder');
toRepo = uigetdir(pwd, 'Select release folder');
items = dir(fromRepo);
for i = 1:length(items)
    itemName = items(i).name;
    
    if strcmp(itemName, '.') || strcmp(itemName, '..')
        continue;
    end
    
    fullPath = fullfile(fromRepo, itemName);
    
    if ~strcmp(itemName, '.git')
        if items(i).isdir
            mkdir(fullfile(toRepo, itemName)); 
            copyfile(fullPath, fullfile(toRepo, itemName));
            fprintf('Copied folder: %s\n', fullPath);
        else
            % If it's a file, copy it directly
            copyfile(fullPath, toRepo);
            fprintf('Copied file: %s\n', fullPath);
        end
    end
end

% Delete unnecessary files
disp('Deleting unnecessary files and folders');
items = dir(toRepo);
for i = 1:length(items)
    itemName = items(i).name;

    if contains(itemName, '-do_not_publish')
        fullPath = fullfile(toRepo, itemName);
        if items(i).isdir
            rmdir(fullPath, 's');
            fprintf('Deleted folder: %s\n', fullPath);
        else
            delete(fullPath);
            fprintf('Deleted file: %s\n', fullPath);
        end
    end
end
fullPath = fullfile(toRepo, '.gitignore');
delete(fullPath);
fprintf('Deleted file: %s\n', fullPath);
fullPath = fullfile(toRepo, '.git');
rmdir(fullPath, 's');
fprintf('Deleted folder: %s\n', fullPath);

disp('Release completed!')
disp('=============================')