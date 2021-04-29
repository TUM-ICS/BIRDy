function [ Ry ] = RotY(betaY, degrad)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the rotation matrix around the Y axis with angle betaY.

if nargin > 1 && strcmp(degrad, 'deg')
    betaY = betaY *pi/180;
end

cb=cos(betaY);
sb=sin(betaY);

Ry=[cb  0 sb;
    0   1 0 ;
    -sb 0 cb];

end

