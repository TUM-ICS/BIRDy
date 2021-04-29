function J = computeGeometricJacobian(robot, jointId, HT_base_world, HT_dhi_world, compute_COM_Jacobian, HT_cmi_world, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot, Gordon Cheng
%
% This function computes row by row, the Geometric Jacobian matrix associated
% with a given robot link in world frame.
%
% HT_link is a tensor containing the HT matrices of each robot link

if nargin < 5 || isempty(compute_COM_Jacobian) || isempty(HT_cmi_world)
    compute_COM_Jacobian = false; % Compute the link jacobian
end

J= sym(zeros(6, robot.nbDOF));

switch robot.dhConvention
    case 'distal'
        for id = 1:jointId
            if id == 1
                Z_im1 = HT_base_world(1:3,1:3)*[0;0;1];
                if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                    P = HT_cmi_world(1:3,4,jointId)-HT_base_world(1:3,4);
                else
                    P = HT_dhi_world(1:3,4,jointId)-HT_base_world(1:3,4);
                end
            else
                Z_im1 = HT_dhi_world(1:3,1:3,id-1)*[0;0;1];
                if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                    P = HT_cmi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id-1);
                else
                    P = HT_dhi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id-1);
                end
            end
            
            if(strcmp(robot.jointType(id,:), 'revol')) % Revolute joint
                J(1:3,id) = cross(Z_im1,P);
                J(4:6,id) = Z_im1;
            elseif(strcmp(robot.jointType(id,:), 'prism')) % Prismatic joint
                J(1:3,id) = Z_im1;
                J(4:6,id) = [0;0;0];
            else
                error('Error in the symbolic expression of the robot Jacobian matrix ! There must be exactly one joint parameter per row of the DH table !')
            end
        end
    case 'proximal'
        for id = 1:jointId
            Z_im1 = HT_dhi_world(1:3,1:3,id)*[0;0;1];
            if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                P = HT_cmi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id);
            else
                P = HT_dhi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id);
            end
            
            if(strcmp(robot.jointType(id,:), 'revol')) % Revolute joint
                J(1:3,id) = cross(Z_im1,P);
                J(4:6,id) = Z_im1;
            elseif(strcmp(robot.jointType(id,:), 'prism')) % Prismatic joint
                J(1:3,id) = Z_im1;
                J(4:6,id) = [0;0;0];
            else
                error('Error in the symbolic expression of the robot Jacobian matrix ! There must be exactly one joint parameter per row of the DH table !')
            end
        end
    otherwise
        error('Unknown DH convention !')
end

% J = combine(simplify(J));
end
