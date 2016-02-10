function latestTimeStampedFile(patterns,dirName)
    %LATESTTIMESTAMPEDFILE
    %
    % LATESTTIMESTAMPEDFILE(patterns)
    % LATESTTIMESTAMPEDFILE(patterns,dirName)
    %
    % patterns - String/ cell of strings.
    % dirName  - String. Defaults to pwd.

    if nargin < 2
        dirName = pwd;
    end
    if ischar(patterns)
        patterns = {patterns};
    end
    files = dir(dirName);
    fnames = {files.name};
    
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