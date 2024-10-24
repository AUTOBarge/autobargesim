function version = absimver()
    info = load(fullfile(absimroot, 'info.mat'));
    version = info.metadata.version;
end

