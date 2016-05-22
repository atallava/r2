function maxTerms = calcMaxTermsForAlphaEstimate(k,N)
%CALCMAXTERMSFORALPHAESTIMATE 
% Solves equation max_n n+k(n) <= N, n belongs to integers.
% 
% maxTerms = CALCMAXTERMSFORALPHAESTIMATE(k,N)
% 
% k        - Function handle.
% N        - Scalar.
% 
% maxTerms - Scalar.

fun = @(n) n+k(n)-N;
maxTerms = fzero(fun,N*0.5);
maxTerms = floor(maxTerms);
end