function [sig2_error] = getErrorCovariance(W, Y_tau, nbDOF) 

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Gives the error covariance from the observation matrix following the
% approach of Gautier and Poignet (2001). This is used for weighted least
% squares computation. 

sig2_error = zeros(nbDOF,1);

for i=1:nbDOF
    [~,Ri] = qr(W(i:nbDOF:end,:),0);
    Diag_Ri = abs(diag(Ri));
    Diag_Ri_red = find(Diag_Ri >= 1.0);
    
    % Keep relevant columns:
    W_reduced_i = W(i:nbDOF:end,Diag_Ri_red);
    [neq,np] = size(W_reduced_i);
    
    % Calculate OLS solution:
    Beta_LS = W_reduced_i\Y_tau(i:nbDOF:end);
    
    % Compute error covariance:
    Residual = Y_tau(i:nbDOF:end) - W_reduced_i*Beta_LS;
	normRes = norm(Residual);		
    
    sig2_error(i) = normRes^2/(neq-np); % Compute the resulting error covariance estimate
end

end