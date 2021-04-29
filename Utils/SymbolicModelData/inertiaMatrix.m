function [I] = inertiaMatrix(XXi, XYi, XZi, YYi, YZi, ZZi)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function simply bilds an inertia matrix from the values of inertias

I = [XXi, XYi, XZi; ...
    XYi, YYi, YZi; ...
    XZi, YZi, ZZi];
end