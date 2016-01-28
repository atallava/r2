function dataset = huskyVarsToDataset(state,controls,t,period)
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
    % TODO: take care of breaks in t
    dt = diff(t);
    if range(dt) > min(dt)
        error('huskyVarsToDataset:invalidTimesteps','t has breaks.');
    end
    
    % TODO: threshold for xy transition
    % assumes contiguous time sequence
    maxSegmentLength = 5000;
    if nMeasurements > maxSegmentLength
        id = randsample(1:nMeasurements-maxSegmentLength,1);
        ids = id:(id+maxSegmentLength-1);
    else
        ids = 1:nMeasurements;
    end
    xyState = state(ids,1:2);
    % get xy norms between successive states
    dXy2 = diff(xyState,1);
    dXy2 = dXy2.^2; dXy2 = sum(dXy2,2); 
    dXy2Thresh = quantile(dXy2,0.8); % arbitrary
    
    thState = state(ids,3);
    dTh = thDiff(thState); dTh = abs(dTh);
    dThThresh = quantile(dTh,0.8);
    
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
        % TODO: checks based on state jumps. If any transition in segment
        % jumps, don't add to dataset
        xyState = state(id1:id2,1:2);
        dXy2 = diff(xyState,1);
        dXy2 = dXy2.^2; dXy2 = sum(dXy2,2);
        thState = state(id1:id2,3);
        dTh = thDiff(thState); dTh = abs(dTh);
        if any(dXy2 > dXy2Thresh) || any(dTh > dThThresh)
            continue;
        end
        
        nElements = nElements+1;
        dataset(nElements).stateInit = state(id1,:);
        dataset(nElements).stateFinal = state(id2,:);
        dataset(nElements).controls = controls(id1:id2-1,:);
        dataset(nElements).dt = diff(t(id1:id2));
    end
end