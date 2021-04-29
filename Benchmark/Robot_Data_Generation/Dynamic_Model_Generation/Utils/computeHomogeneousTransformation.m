function HT = computeHomogeneousTransformation(highIndex, lowIndex, robotModelParameters, compute_COM_Transform, HT_cmi_dhi, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Compute the Homogeneous transformation between lines i and j of the
% Denavit-Hartenberg (DH) table "dhTable" using either distal or proximal
% convention.

if nargin < 3 || isempty(robotModelParameters.dhConvention)
    robotModelParameters.dhConvention = 'distal'; % Use distal convention by default
end

if nargin < 4 || isempty(compute_COM_Transform)
    compute_COM_Transform = false; % Compute link transform by default
end


HT = sym(eye(4));


for k = lowIndex+1:1:highIndex
    TransX = [1 0 0 robotModelParameters.dhParameter.a(k);...
        0 1 0 0;...
        0 0 1 0;...
        0 0 0 1];
    
    TransZ = [1 0 0 0;...
        0 1 0 0;...
        0 0 1 robotModelParameters.dhParameter.d(k);...
        0 0 0 1];
    
    rotX = [1 0 0 0;...
        0 cos(robotModelParameters.dhParameter.alpha(k)) -sin(robotModelParameters.dhParameter.alpha(k)) 0;...
        0 sin(robotModelParameters.dhParameter.alpha(k)) cos(robotModelParameters.dhParameter.alpha(k)) 0;...
        0 0 0 1];
    
    rotZ = [cos(robotModelParameters.dhParameter.theta(k)) -sin(robotModelParameters.dhParameter.theta(k)) 0 0;...
        sin(robotModelParameters.dhParameter.theta(k)) cos(robotModelParameters.dhParameter.theta(k)) 0 0;...
        0 0 1 0;...
        0 0 0 1];
    
    switch (robotModelParameters.dhConvention)
        case 'distal'
            % Distal convention:
            HT = HT*(rotZ*TransZ*TransX*rotX);
        case 'proximal'
            % Proximal convention:
            HT = HT*(rotX*TransX*rotZ*TransZ);
        otherwise
            error('Unknown DH convention !');
    end
    if compute_COM_Transform == true &&  k == highIndex
        HT = HT*HT_cmi_dhi;
    end
end

% HT = combine(simplify(HT));


end


