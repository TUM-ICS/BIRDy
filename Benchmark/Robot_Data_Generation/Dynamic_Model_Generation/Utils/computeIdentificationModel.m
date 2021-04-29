function [Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = computeIdentificationModel(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, HT_dhi_dhi_1, robot, Tau, Tau_friction_aug, options)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

if nargin<11 || (isequal(robot.dhConvention,'distal') && strcmp(options.method, 'baseParameters'))
    options.method = 'baseParametersNum';
end

switch robot.frictionIdentModel
    % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
    % Nonlinear friction models require state dependant parameter identification
    case 'no'
        Tau = Tau + Tau_friction_aug(:,1);
    case 'Viscous'
        Tau = Tau + Tau_friction_aug(:,2);
    case 'Coulomb'
        Tau = Tau + Tau_friction_aug(:,3);
    case 'ViscousCoulomb'
        Tau = Tau + Tau_friction_aug(:,4);
    case 'ViscousCoulombOff'
        Tau = Tau + Tau_friction_aug(:,5);
    case 'Stribeck'
        Tau = Tau + Tau_friction_aug(:,6);
    case 'LuGre'
        Tau = Tau + Tau_friction_aug(:,7);
    otherwise
        Tau = Tau + Tau_friction_aug(:,1);
end

% robot.symbolicParameters.Gravity = [sym(zeros(2,1));robot.symbolicParameters.Gravity(3)]; % ASSUMES THAT THE BASE Z AXIS OF THE ROBOT IS COLINEAR WITH GRAVITY. OTHERWISE REMOVE !

fprintf('Computing Identification Model using the Algorithm: %s\n', options.method)

[Y_r, Xhi_r] = blockTriangular(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, robot);
[Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = baseParametersNumericalSimplification(robot, Y_r, Xhi_r, 1e3);
end

%% Compute the regressor in a block upper-triangular form as explained in [jung et al. 2018]:

function [Y, Xhi] = blockTriangular(J_dhi_world, Jd_dhi_world, J_cmi_world, Jd_cmi_world, HT_cmi_world, robot)

% Generate the parameter vector defined in [jung et al. 2018]:
Xhi = robot.symbolicParameters.Xhi;
nbparam = robot.numericalParameters.numParam;

% Compute the Block-Upper-Triangular Model Regressor:

counter = 0;
Y = sym(zeros(robot.nbDOF,numel(Xhi)));
for i = 1:robot.nbDOF
    Ri = HT_cmi_world(1:3,1:3,i);
    W_cmi_world = J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp;
    Wp_cmi_world = Jd_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp + J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qpp;
    Vp_dhi_world = Jd_dhi_world(1:3,:,i)*robot.symbolicParameters.Qp + J_dhi_world(1:3,:,i)*robot.symbolicParameters.Qpp;
    
    switch robot.frictionIdentModel
        % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
        % Nonlinear friction models require state dependant parameter identification
        case 'no'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        case 'Viscous'
            H = sym(zeros(robot.nbDOF, 3));
            H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) 1];
        case 'Coulomb'
            H = sym(zeros(robot.nbDOF, 3));
            H(i,:) = [robot.symbolicParameters.Qpp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
        case 'ViscousCoulomb'
            
            if strcmp(robot.name, 'TX40') || strcmp(robot.name, 'RX90') % Take the coupling between joints 5 and 6 into account
                if i == 6
                    H = sym(zeros(robot.nbDOF, 5));
                    H(i-1,:) = [robot.symbolicParameters.Qpp(i) 0 0 robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                    H(i,:) = [robot.symbolicParameters.Qpp(i)+robot.symbolicParameters.Qpp(i-1) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) robot.symbolicParameters.Qp(i-1) tanh(100*robot.symbolicParameters.Qp(i-1))];
                else
                    H = sym(zeros(robot.nbDOF, 3));
                    H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                end
            else
                H = sym(zeros(robot.nbDOF, 3));
                H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
            end
            
        case 'ViscousCoulombOff'
            
            if strcmp(robot.name, 'TX40') || strcmp(robot.name, 'RX90') % Take the coupling between joints 5 and 6 into account
                if i == 6
                    H = sym(zeros(robot.nbDOF, 6));
                    H(i-1,:) = [robot.symbolicParameters.Qpp(i) 0 0 robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                    H(i,:) = [robot.symbolicParameters.Qpp(i)+robot.symbolicParameters.Qpp(i-1) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1 robot.symbolicParameters.Qp(i-1) tanh(100*robot.symbolicParameters.Qp(i-1))];
                else
                    H = sym(zeros(robot.nbDOF, 4));
                    H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
                end
            else
                H = sym(zeros(robot.nbDOF, 4));
                H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
            end
            
        case 'Stribeck'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        case 'LuGre'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        otherwise
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
    end
    a1 = J_dhi_world(1:3,:,i)'*(Vp_dhi_world + robot.symbolicParameters.Gravity);
    a2 = (J_dhi_world(1:3,:,i)'*Skew(Wp_cmi_world) + J_dhi_world(1:3,:,i)'*Skew(W_cmi_world)*Skew(W_cmi_world) - J_cmi_world(4:6,:,i)'*Skew(Vp_dhi_world + robot.symbolicParameters.Gravity))*Ri;
    a3 = J_cmi_world(4:6,:,i)'*(Ri*B(Ri'*Wp_cmi_world)+Skew(W_cmi_world)*Ri*B(Ri'*W_cmi_world));
    a4 = H;
    Y(:,counter+1:counter+nbparam(i)) = [a3, a2, a1, a4];
    counter = counter + nbparam(i);
end

fprintf('Generating funcion handle for the regressor...\n')
matlabFunction(Y,'File','Benchmark/Robot_Generated_Data/Y_r_handle', 'Vars', {robot.symbolicParameters.Q, robot.symbolicParameters.Qp, robot.symbolicParameters.Qpp, robot.symbolicParameters.Geometry, robot.symbolicParameters.Gravity},'Optimize',true);

end

%% Numerical generation of Base Parameters using a QR decomposition of the regression matrix on multiple random epochs [Khalil 91]:

function [Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = baseParametersNumericalSimplification(robot, Y, Xhi, samples)

disp('Generating the base parameters using QR decomposition...');
Geometry = robot.numericalParameters.Geometry;
Gravity = robot.numericalParameters.Gravity;
Obs = zeros(samples*robot.nbDOF,numel(Xhi));
% Generating an observation matrix:
fprintf('Sampling...')
for i = 1:samples
    if mod(i,50)==0
        fprintf('.')
    end
    Q = randn(robot.nbDOF,1);
    Qp = randn(robot.nbDOF,1);
    Qpp = randn(robot.nbDOF,1);
    Obs(robot.nbDOF*(i-1)+1:robot.nbDOF*i,:) = Y_r_handle(Q,Qp,Qpp,Geometry,Gravity);
end
fprintf('\nNullspace base extraction using QR decomposition...\n')

% Evaluate the nulspace of the observation matrix (USE A ROUND FUNCTION TO AVOID THE 10^-18 FACTORS IN THE BASE PARAM EXPRESSION):
rk = rank(Obs,1e-7);
[~,qr_R,qr_P]=qr(Obs);
qr_Rb = qr_R(1:rk,1:rk);
qr_Rd = qr_R(1:rk,rk+1:end);
Kd = round(qr_Rb\qr_Rd,10);

% Permutation matrices:
qr_Pb = qr_P(:,1:rk);
qr_Pd = qr_P(:,rk+1:end);

% Reordering as [Y_b Y_d]*[Xhi_b;Xhi_d] = Y*Xhi = Tau
Xhi_b = qr_Pb'*Xhi;
Xhi_d = qr_Pd'*Xhi;
Y_b = Y*qr_Pb;
Y_d = Y*qr_Pd;

% Generate a set of base parameters Beta such as Y_b*Beta = Y*Xhi = Tau:
Beta = Xhi_b + Kd*Xhi_d;

fprintf('\nBased on the analysis of the standard observation matrix, a set of %d independant base parameters could be computed...\n', rk)
delete Benchmark/Robot_Generated_Data/Y_r_handle.m
end

function [B] = B(v)
B = [[v(1); 0; 0], [v(2); v(1); 0], [v(3); 0; v(1)], [0; v(2); 0], [0; v(3); v(2)], [0; 0; v(3)]];
end

