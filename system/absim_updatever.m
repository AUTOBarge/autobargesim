function absim_updatever(version_number)
    validateattributes(version_number, {'char'}, {'row'}, 'writever', 'input', 1);

    matObj = matfile(fullfile(absimroot, 'info.mat'), 'Writable', true);
    metadata = matObj.metadata;
    metadata.version = version_number;
    matObj.metadata = metadata;
end

