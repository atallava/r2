function dataset = convertTrajToDynDataset(trajFname,datasetFname)
    %CONVERTTRAJTODYNDATASET
    %
    % dataset = CONVERTTRAJTODYNDATASET(trajFname)
    % dataset = CONVERTTRAJTODYNDATASET(trajFname,datasetFname)
    %
    % trajFname    - Trajectory file name.
    % datasetFname - File to save dataset in. No save by default.
    %
    % dataset      - Struct with fields
    % ('stateInit','statesDot','stateFinal',dt');

    if nargin < 2
        saveFlag = false;
    else
        saveFlag = true;
    end
    
    load(trajFname,'physicalParams','controller','dt','state0','nSteps',...
        't','states','statesDot','controls');
    
    dataset = struct('stateInit',{},'statesDot',{},'stateFinal',{},'dt',{});
    
    dataset(1).stateInit = state0;
    dataset(1).stateFinal = states(1,:);
    dataset(1).statesDot = statesDot(1,:);
    dataset(1).dt = dt;

    nStates = size(states,1);
    for i = 2:nStates
        dataset(i).stateInit = states(i-1,:);
        dataset(i).stateFinal = states(i,:);
        dataset(i).statesDot = statesDot(i,:);
        dataset(i).dt = dt;
    end
    
    % save
    if saveFlag
        save(datasetFname);
    end    
end