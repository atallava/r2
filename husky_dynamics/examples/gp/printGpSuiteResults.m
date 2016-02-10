% pretty print suite results
gpWrapperFnameTemplate = ['gp' '_'...
    'ang_vel' '_' ...
    'suite' '_' ...
    '0204'];
dirName = 'data';

% get the gp wrapper files
res = dir(dirName);
fnames = {res.name};
gpWrapperFnames = {};
for i = 1:length(fnames)
    fname = fnames{i};
    posn = strfind(fname,gpWrapperFnameTemplate);
    if ~isempty(posn)
        posn = strfind(fname,'_err');
        % don't add the test error files
        if isempty(posn)
            gpWrapperFnames{end+1} = fname;
        end
    end
end

nGpWrappers = length(gpWrapperFnames);
errWrappers = zeros(1,nGpWrappers);
stt = cell(1,nGpWrappers);
errStt = cell(1,nGpWrappers);
for i = 1:nGpWrappers
    fname = gpWrapperFnames{i};
    fname = [dirName '/' fname];
    stt{i} = load(fname);
    fprintf('Gp wrapper:\n');
    disp(fname);
    
    % print hyp
    fprintf('Mean function:\n');
    disp(stt{i}.meanFunc);
    fprintf('Mean hyp:\n');
    disp(stt{i}.hyp.mean);
    fprintf('Cov function:\n');
    disp(stt{i}.covFunc);
    fprintf('Cov hyp:\n');
    disp(stt{i}.hyp.cov);
    fprintf('Training nll: %4f.\n',stt{i}.nll);
    
    % print error
    errFname = [dirName '/' stripFileExtension(gpWrapperFnames{i}) '_err'];
    errStt{i} = load(errFname);
    fprintf('Error dataset:\n');
    disp(errStt{i}.datasetFname);
    fprintf('mean err: %.4f, std: %.4f\n',mean(errStt{i}.errVec),std(errStt{i}.errVec));
    errWrappers(i) = mean(errStt{i}.errVec);
    
    % separator
    fprintf('\n%s\n\n',repmat('-',1,50));
end

[~,minId] = min(errWrappers);
fprintf('Min error: %.4f\n',errWrappers(minId));
fprintf('Mean function:\n');
disp(stt{minId}.meanFunc);
fprintf('Cov function:\n');
disp(stt{minId}.covFunc);
fprintf('\n%s\n\n',repmat('-',1,50));


