function [se,seVec] = predSE(fun,x,y,varargin)
    %PREDSE 
    %
    % [se,seVec] = PREDSE(fun,x,y,varargin)
    %
    % fun      - Function handle. y' = fun(x,varargin{:})
    % x        -
    % y        -
    % varargin -
    %
    % se       - MSE.
    % seVec    - 
    
   yPred = fun(x,varargin{:});
   seVec = (yPred-y).^2;
   se = mean(seVec);
end