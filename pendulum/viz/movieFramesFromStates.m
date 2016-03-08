function F = movieFramesFromStates(states,physicalParams)
    if isfield(physicalParams,'l')
        l = physicalParams.l;
    else
        error('l not a field in physicalParams.');
    end
    
    % subsample states 
    maxStates = 1e3;
    if size(states,1) > maxStates
        startId = randsample(1:(size(states,1)-maxStates),1);
        ids = startId:(startId+maxStates-1);
        ids = floor(ids);
        states = states(ids,:);
    end
    
    pivotPt = [l l];
    nStates = size(states,1);
    massPts = zeros(nStates,2);
    massPts(:,1) = l+l.*sin(states(:,1));
    massPts(:,2) = l-l.*cos(states(:,1));
    
    radius = 0.05*l; 
    
    F(nStates) = struct('cdata',[],'colormap',[]);
    
    for i = 1:nStates
        hf = figure;
        set(hf,'visible','off');
        box on;
        plot([pivotPt(1) massPts(i,1)],...
                [pivotPt(2) massPts(i,2)],'k','linewidth',1.5);
        viscircles(massPts(i,:),radius);

        axis equal;
        padding = 0.05*l;
        xlim([-padding 2*l+padding]);
        ylim([-padding 2*l+padding]);
        xlabel('x');
        ylabel('y');
        
        F(i) = getframe(gca);
        close(hf);
        clear hf;
    end
end