function [Rv, Sv, pNoiseAdaptParams] = annealingNoiseCovariance(P, Rv, Sv, pNoiseAdaptParams, paramSize, optionKF)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Annealing of for the process noise covariance matrix.
if optionKF.anneal == true
        pNoiseAdaptParams.cov = max(pNoiseAdaptParams.annealFactor*pNoiseAdaptParams.cov , pNoiseAdaptParams.variance);
end
% Else do not change the process noise covariance matrix
Rv(end-paramSize+1:end,end-paramSize+1:end)= diag(pNoiseAdaptParams.cov); % Only the parameter covariance is annealed
Sv(end-paramSize+1:end,end-paramSize+1:end)= diag(sqrt(pNoiseAdaptParams.cov)); % Only the parameter covariance is annealed

end

