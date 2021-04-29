function [P_i] = pseudoInertiaMatrix(Li, Mi, li, Iai, Fvi, Fci)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function returns the pseudo-inertia matrix P_i from a link mass Mi,
% link inertia Li wrt DH frame and moment vector li.

% P_i is used to verify the full physical consistency of a robot link. As a 
% criterion for full physical consistency, the matrix P_i should be strictly 
% positive definite (c.f. Wensing et al. 2018 and Cristovao et al. 2018)

P_i = [[(trace(Li)/2)*eye(3)-Li, li; li', Mi], zeros(4,3);...
    zeros(3,4), diag([Iai; Fvi; Fci])];

end