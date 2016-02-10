function subsampleDataset(fnameIn,arg2,fnameOut)
    %SUBSAMPLEDATASET 
    %
    % SUBSAMPLEDATASET(fname)
    % SUBSAMPLEDATASET(fname,fracn)
    % SUBSAMPLEDATASET(fname,maxElements)
    % SUBSAMPLEDATASET(fname,arg2,fnameOut)
    %
    % fnameIn - String.
    % arg2  - Scalar. If <= 1, interpreted as fraction to retain. If integer > 1 it
    % is the number of elements to keep.
    % fnameOut - String. Defaults to [fnameIn '_subsampled'].
    
    fnameIn = stripFileExtension(fnameIn);
    if nargin < 2
        arg2 = 3000;
    end
    fileStt = load(fnameIn);
    nElements = length(fileStt.dataset);
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
    fileStt.dataset = fileStt.dataset(ids);
    
    if nargin < 3
        fnameOut = [fnameIn '_subsampled'];
    end
    save(fnameOut,'-struct','fileStt');
    fprintf('File saved as %s.\n',fnameOut);
end