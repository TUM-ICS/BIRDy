function [Xhi, InertiaDH, InertiaCOM, numberParam] = aggregateNumericalParameters(nbDOF, frictionIdentModel, numericalParameters, linkRadius, linkLength, eulerAngles)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function builds the dynamic parameter vector and inertia matrices of
% a robot from its numerical parameters.

if nargin < 4 % Inertias are provided manually
    cylinderInertia = false;
else
    cylinderInertia = true;
end
InertiaDH = zeros(3,3,nbDOF);
InertiaCOM = zeros(3,3,nbDOF);

for i=1:nbDOF
    if cylinderInertia == true % Inertias are not provided manually and have to be computed considering a cylindrical link geometry:
        R = RotZ(eulerAngles(1,i))*RotY(eulerAngles(2,i))*RotX(eulerAngles(3,i));
        I = computeZAxisCylinderInertiaMatrix(linkRadius(i), linkLength(i), numericalParameters.Mass(i)); % Inertia of a cylinder of axis Z around DH frame
        InertiaDH(:,:,i) = R*I*R';
        InertiaCOM(:,:,i) = inertiaMatrixDH2COM(InertiaDH(:,:,i), numericalParameters.Mass(i), numericalParameters.Moment(:,i));   % Transform the inertia matrix to match the orientation of the cylinder
    else
        if numericalParameters.InertiaDH(1,:,i) == zeros(3)
            error('The inertia matrix of your robot cannot be zero !');
        end
        InertiaDH(:,:,i) = numericalParameters.InertiaDH(:,:,i); % Around DH frames
        InertiaCOM(:,:,i) = inertiaMatrixDH2COM(InertiaDH(:,:,i), numericalParameters.Mass(i), numericalParameters.Moment(:,i)); % Around COM frames
    end
end

% Generate the vector Xhi of standard parameters:
[Xhi, numberParam, ~] = getStandardParameterVector(numericalParameters.InertiaDH, numericalParameters.Moment, numericalParameters.Mass, numericalParameters.Ia, frictionIdentModel, numericalParameters.friction);

end

function [XXi, XYi, XZi, YYi, YZi, ZZi] = computeZAxisCylinderInertia(radius, length, mass)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function gives the different inertia values of a cylinder defined by its
% radius, length and mass. 

XXi = 0.0833333 * mass * (3 * radius * radius + length * length);
XYi = 0;
XZi = 0;
YYi = 0.0833333 * mass * (3 * radius * radius + length * length);
YZi = 0;
ZZi = 0.5 * mass * radius * radius;
end

function [I] = computeZAxisCylinderInertiaMatrix(radius, length, mass)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function gives the inertia matrix of a cylinder defined by its
% radius, length and mass. 

XXi = 0.0833333 * mass * (3 * radius * radius + length * length);
XYi = 0;
XZi = 0;
YYi = 0.0833333 * mass * (3 * radius * radius + length * length);
YZi = 0;
ZZi = 0.5 * mass * radius * radius;
I = [XXi XYi XZi; ...
    XYi YYi YZi; ...
    XZi YZi ZZi];
end

