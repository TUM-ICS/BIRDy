function [S] = Skew(v)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the skew matrix for cross product.

S = [0,-v(3), v(2);
    v(3),0,-v(1);
    -v(2), v(1), 0];
end

