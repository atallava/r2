function [A,B,Q] = affinifyLinearization(A,B,ABResidual,Q)
    % dynamics
    A = [A ABResidual'; ...
        0 0 1];
    B = [B; 0];
    
    % costs
    Q = [Q zeros(2,1); 0 0 1];
end