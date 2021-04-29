function [ Rz ] = RotZ(gammaZ, degrad)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the rotation matrix around the Z axis with angle gammaZ.

if nargin > 1 && strcmp(degrad, 'deg')
    gammaZ = gammaZ *pi/180;
end

cg=cos(gammaZ);
sg=sin(gammaZ);

Rz=[cg -sg 0;
    sg  cg 0;
    0   0  1];

end

