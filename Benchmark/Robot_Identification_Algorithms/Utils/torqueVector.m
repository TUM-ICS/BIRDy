function [Y_tau] = torqueVector(Tau) %#codegen

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building the sampled torque vector.

[nbDOF,nbSamples]=size(Tau);
Y_tau = zeros(nbDOF*nbSamples, 1);

for i=1:nbSamples
    for j=1:nbDOF
        Y_tau((i-1)*nbDOF+j)=Tau(j,i);
    end
end

end
