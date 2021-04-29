function [P_i] = buildLMI(Beta,Xhi_d,K_d,Perm,nbDOF,numParam)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function returns the pseudo-inertia matrix P_i from the link base parameter
% vector "Beta", the corresponding unidentifiable parameter vector "Xhi_d"
% and the permutation and transform matrices (K_d,Perm) computed during the base
% parameter generation step and associated to "Beta". This allows defining a bijective
% mapping between the base parameters vector "Beta" and the full parameter
% vector "Xhi" as proposed in Cristovao et al (2014).

% P_i is used to verify the full physical consistency of a robot link. As a
% criterion for full physical consistency, the matrix P_i should be strictly
% positive definite (c.f. Wensing et al. 2018 and Cristovao et al. 2018)

nb = length(Beta);
nd = length(Xhi_d);
Xhi_reconstructed = Perm*[eye(nb), -K_d;zeros(nd, nb), eye(nd)]*[Beta;Xhi_d];

% Xhi_i = [XXi; XYi; XZi; YYi; YZi; ZZi; numericalParameters.GeometryCOM(:,i); numericalParameters.Mass(i); numericalParameters.Ia(i); numericalParameters.friction.Fv(i); numericalParameters.friction.Fc(i)];
% Xhi_reconstructed = [XXi; YYi; ZZi; XYi; XZi; YZi; numericalParameters.Mass(i)*numericalParameters.GeometryCOM(:,i); numericalParameters.Mass(i); numericalParameters.Ia(i); numericalParameters.friction.Fv(i); numericalParameters.friction.Fc(i)];

% TODO: use the param number in Xhi

% P_i = zeros(7,7,nbDOF);

counter =0;
for i = 1:nbDOF
    Mi = Xhi_reconstructed(counter+10);
    Iai = Xhi_reconstructed(counter+11);
    Fvi = Xhi_reconstructed(counter+12);
    Fci = Xhi_reconstructed(counter+13);
    li = Xhi_reconstructed(counter+7:counter+9); % MXi MYi MZi
    Li = inertiaMatrix(Xhi_reconstructed(counter+1),Xhi_reconstructed(counter+2),Xhi_reconstructed(counter+3),Xhi_reconstructed(counter+4),Xhi_reconstructed(counter+5),Xhi_reconstructed(counter+6));
    counter = counter + numParam(i);
    P_i(:,:,i) = [[(trace(Li)/2)*eye(3)-Li, li; li', Mi], zeros(4,3);...
    zeros(3,4), diag([Iai; Fvi; Fci])];
end

end
