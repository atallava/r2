function [K,V] = solveLqr(A,B,T,Q,R)
    %SOLVELQR
    %
    % [K,V] = SOLVELQR(A,B,T,Q,R)
    %
    % A -
    % B -
    % T -
    % Q -
    % R -
    %
    % K - Cell array.
    % V - Cell array.

    % cost matrices
    % check for > 0
    condn = all(eig(Q) > 0);
    assert(condn,'Q is not positive definite.\n');
    condn = all(eig(R) >= 0);
    assert(condn,'R is not positive semi-definite.\n');
    
    % value function
    V = cell(1,T);
    % gain
    K = cell(1,T-1);
    
    V{end} = Q;
    for t = (T-1):-1:1
        % linear controller
        K{t} = -(R+B'*V{t+1}*B)\...
            (B'*V{t+1}*A);
        
        % sanity check
        flag = isnan(K{t}(:));
        condn = ~any(flag);
        assert(condn,'K values are nan.');
        flag = isinf(K{t}(:));
        condn = ~any(flag);
        assert(condn,'K values are inf.');
        
        % quadratic value function
        V{t} = Q+K{t}'*R*K{t}+...
            (A+B*K{t})'*V{t+1}*(A+B*K{t});
        
        % sanity check
        flag = isnan(V{t}(:));
        condn = ~any(flag);
        assert(condn,'V values are nan.');
        flag = isinf(V{t}(:));
        condn = ~any(flag);
        assert(condn,'V values are inf.');
    end
end