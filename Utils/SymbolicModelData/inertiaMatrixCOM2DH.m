function [L] = inertiaMatrixCOM2DH(I, Mi, Moment)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function computes the inertia matrix L of a robot link at the joint centre position, using the Inertia
% matrix I of the same link but computed at its CoM.

L = I + Skew(Moment)'*Skew(Moment)./Mi;

end