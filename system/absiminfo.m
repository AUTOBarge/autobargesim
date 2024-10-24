function absiminfo
    info = load(fullfile(absimroot, 'info.mat'));
    software = info.metadata.software;
    version = info.metadata.version;
    fprintf('%s (%s) library is installed!\n', software, version);
end

