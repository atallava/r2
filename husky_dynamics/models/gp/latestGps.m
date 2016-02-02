function latestGps()
    files = dir(pwd);
    fnames = {files.name};
    patterns = {'gp_lin_vel','gp_lat_vel','gp_ang_vel'};
    for i = 1:length(patterns)
        list = {};
        numlist = [];
        for j = 1:length(fnames)
            posn = strfind(fnames{j},patterns{i});
            if ~isempty(posn)
                tag = fnames{j}(length(patterns{i})+2:end-4);
                list{end+1} = tag;
                numlist(end+1) = str2double(tag);
            end
        end
        [~,id] = max(numlist);
        fprintf('%s_%s\n',patterns{i},list{id});
    end
end