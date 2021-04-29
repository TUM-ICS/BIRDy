function [ Rx ] = RotX(alphaX, degrad)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the rotation matrix around the X axis with angle alphaX.

if nargin > 1 && strcmp(degrad, 'deg')
    alphaX = alphaX *pi/180;
end

ca=cos(alphaX);
sa=sin(alphaX);

Rx=[1 0   0;
    0 ca -sa;
    0 sa  ca];

end

