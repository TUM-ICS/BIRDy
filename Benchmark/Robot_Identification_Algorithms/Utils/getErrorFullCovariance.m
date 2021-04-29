function [Omega] = getErrorFullCovariance(W, Y_tau, nbDOF)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Gives the error covariance from the observation matrix following the
% approach of Han et al. (2020). This is used for IRLS computation.

[neq,np] = size(W);

Residual = zeros(nbDOF,neq/nbDOF);

for i=1:nbDOF
    % Calculate OLS solution:
    Beta_LS = W(i:nbDOF:end,:)\Y_tau(i:nbDOF:end);
    
    % Compute error covariance:
    Residual(i,:) = (Y_tau(i:nbDOF:end) - W(i:nbDOF:end,:)*Beta_LS).';	
    
end

Omega = (Residual*Residual.')/(neq-np); % Compute the resulting error covariance estimate

end
