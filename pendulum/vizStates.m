function hf = vizStates(states,physicalParams)
    if isfield(physicalParams,'l')
        l = physicalParams.l;
    else
        error('l not a field in physicalParams.');
    end
    
    pivotPt = [l l];
    nStates = size(states,1);    
    massPts = zeros(nStates,2);
    massPts(:,1) = l+l.*sin(states(:,1));
    massPts(:,2) = l-l.*cos(states(:,1));
    
    hf = figure;
    hold on;
    axis equal;
    padding = 0.05*l;
    xlim([-padding 2*l+padding]);
    ylim([-padding 2*l+padding]);
    box on;
    transparencies = linspace(0.2,1,nStates);
    hPlots = cell(1,nStates);
    for i = 1:nStates
        hPlots{i} = plot([pivotPt(1) massPts(i,1)],...
            [pivotPt(2) massPts(i,2)],'k','linewidth',1.5);
        hPlots{i}.Color(4) = transparencies(i);
    end
    radii = 0.05*l*ones(nStates,1);
    viscircles(massPts,radii);
end