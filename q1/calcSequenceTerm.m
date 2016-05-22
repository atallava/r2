function term = calcSequenceTerm(C,alpha,limit,n)
%CALCSEQUENCETERM Calculate n-th term in a polynomially converging
% sequence.
% 
% term = CALCSEQUENCETERM(C,alpha,limit,n)
% 
% C     - 
% alpha - 
% limit - 
% n     - 
% 
% term  - 

avgSequenceN1 = limit-C*(n+1)^(-alpha);
avgSequenceN = limit-C*n^(-alpha);
term = avgSequenceN1*(n+1)-avgSequenceN*n;
end