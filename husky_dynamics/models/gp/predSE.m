function [se,seVec] = predSE(fun,x,y,varargin)
   yPred = fun(x,varargin{:});
   seVec = (yPred-y).^2;
   se = mean(seVec);
end