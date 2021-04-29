function [I] = inertiaTensorDH2COM(L, Mi, Moment)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function computes the inertia tensor I of a robot link at its CoM, using the Inertia
% tensor L of the same link but computed at the joint centre position.

for i=1:numel(Mi)
    I(:,:,i) = L(:,:,i) - Skew(Moment(:,i))'*Skew(Moment(:,i))./Mi(i);
end

end