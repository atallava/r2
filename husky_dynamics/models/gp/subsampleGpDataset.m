function subsampleGpDataset(fnameIn,arg2,fnameOut)
    %SUBSAMPLEGPDATASET 
    %
    % SUBSAMPLEGPDATASET(fname)
    % SUBSAMPLEGPDATASET(fname,fracn)
    % SUBSAMPLEGPDATASET(fname,maxElements)
    % SUBSAMPLEGPDATASET(fname,arg2,fnameOut)
    %
    % fnameIn - String.
    % arg2  - Scalar. If <= 1, interpreted as fraction to retain. If integer > 1 it
    % is the number of elements to keep.
    % fnameOut - String. Defaults to [fnameIn '_subsampled'].
    
    if nargin < 2
        arg2 = 3000;
    end
    data = load(fnameIn);
    nElements = size(data.x,1);
    if arg2 <= 1
        nSubsampled = floor(arg2*nElements);
    else
        condn = mod(arg2,10) == 0;
        assert(condn,'subsampleGpDataset:invalidInput','If arg2 > 1, it must be an integer.');
        nSubsampled = arg2;
    end
    if nSubsampled > nElements
        nSubsampled = nElements;
    end
    ids = randsample(1:nElements,nSubsampled);
    data.x = data.x(ids,:);
    data.y = data.y(ids,:);
    
    if nargin < 3
        fnameOut = [fnameIn '_subsampled'];
    end
    save(fnameOut,'-struct','data');
    fprintf('New file saved as %s.\n',fnameOut);
end