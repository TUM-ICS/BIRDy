function [L] = inertiaTensorCOM2DH(I, Mi, Moment)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function computes the inertia tensor L of a robot link at the joint centre position, using the Inertia
% tensor I of the same link but computed at its CoM.

for i=1:numel(Mi)
    L(:,:,i) = I(:,:,i) + Skew(Moment(:,i))'*Skew(Moment(:,i))./Mi(i);
end

end