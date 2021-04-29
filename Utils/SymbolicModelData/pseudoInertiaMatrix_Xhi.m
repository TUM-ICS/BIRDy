function [P_i] = pseudoInertiaMatrix_Xhi(Xhi_i)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function returns the pseudo-inertia matrix P_i from the parameter
% vector "Xhi_i" of a given robot link.

% P_i is used to verify the full physical consistency of a robot link. As a 
% criterion for full physical consistency, the matrix P_i should be strictly 
% positive definite (c.f. Wensing et al. 2018 and Cristovao et al. 2018)

% Xhi_i = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; numericalParameters.Ia(i); numericalParameters.friction.Fv(i); numericalParameters.friction.Fc(i)];

Li = inertiaMatrix(Xhi_i(1), Xhi_i(2), Xhi_i(3), Xhi_i(4), Xhi_i(5), Xhi_i(6));
li = Xhi_i(7:9);
Mi = Xhi_i(10);
Iai = Xhi_i(11);
Fvi = Xhi_i(12);
Fci = Xhi_i(13);

P_i = [[(trace(Li)/2)*eye(3)-Li, li; li', Mi], zeros(4,3);...
    zeros(3,4), diag([Iai; Fvi; Fci])];

end