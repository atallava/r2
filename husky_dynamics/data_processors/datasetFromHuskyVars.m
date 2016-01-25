function dataset = datasetFromHuskyVars(state,controls,t,period)
    %DATASETFROMHUSKYVARS
    %
    % dataset = DATASETFROMHUSKYVARS(state,controls,t,period)
    %
    % state    - 
    % controls -
    % t        -
    % period   - Scalar.
    %
    % dataset  - Struct array with fields
    % ('stateInit','stateFinal','controls','dt')
    
    nMeasurements = length(t);
    % set start time to 0
    t = t-t(1);
    
    dataset = struct('stateInit',{},'stateFinal',{},'controls',{},'dt',{});
    nElements = 0;
    
    m = floor(t(end)/period);
    tWithPeriod = [0:m]*period;
    nodes = interp1(t,1:nMeasurements,tWithPeriod,'nearest');
    nodes = unique(nodes);
    for i = 1:length(nodes)
        id1 = nodes(i);
        % end of timestamps
        if id1 == nMeasurements
            break;
        end
        if i == length(nodes)
            id2 = nMeasurements;
        else
            id2 = nodes(i+1);
        end
        % empty piece of data, skip
        if id1 == id2
            continue;
        end
        nElements = nElements+1;
        dataset(nElements).stateInit = state(id1,:);
        dataset(nElements).stateFinal = state(id2,:);
        dataset(nElements).controls = controls(id1:id2-1,:);
        dataset(nElements).dt = diff(t(id1:id2));
    end
end