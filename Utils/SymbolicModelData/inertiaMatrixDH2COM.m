function [I] = inertiaMatrixDH2COM(L, Mi, Moment)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function computes the inertia matrix I of a robot link at its CoM, using the Inertia
% matrix L of the same link but computed at the joint centre position.

I = L - Skew(Moment)'*Skew(Moment)./Mi;

end
