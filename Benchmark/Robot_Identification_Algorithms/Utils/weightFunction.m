function [Psi] = weightFunction(Residual, threshold)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Gives the updated weight vector defined in the approach of Han et al. (2020). This is used for IRLS computation.

Psi = (Residual<threshold*ones(size(Residual)))&(Residual>-threshold*ones(size(Residual)));

end