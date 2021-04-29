function [Omega_new] = updateErrorFullCovariance(W_hash, Y_tau_hash, Beta_LS, Omega_old_sqrt, nbDOF)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Gives the updated error covariance from the observation matrix following the
% approach of Han et al. (2020). This is used for IRLS computation.

[neq,np] = size(W_hash);

Residual = zeros(nbDOF,neq/nbDOF);

for i=1:nbDOF
    % Compute error covariance:
    Residual(i,:) = (Y_tau_hash(i:nbDOF:end) - W_hash(i:nbDOF:end,:)*Beta_LS).';	
end

Omega_new = Omega_old_sqrt*(Residual*Residual.')*Omega_old_sqrt/(neq-np); % Compute the resulting error covariance estimate

end