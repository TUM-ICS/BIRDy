function [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Q, nbSamples, nbDOF) %#codegen

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the weighted W and Y_tau where each sample has been weighted by a
% matrix Q

W_star = zeros(size(W));
Y_tau_star = zeros(size(Y_tau)); 

for i=1:nbSamples
    W_star(nbDOF*(i-1)+1:nbDOF*i,:) = Q*W(nbDOF*(i-1)+1:nbDOF*i,:);
    Y_tau_star(nbDOF*(i-1)+1:nbDOF*i) = Q*Y_tau(nbDOF*(i-1)+1:nbDOF*i);
end

end
