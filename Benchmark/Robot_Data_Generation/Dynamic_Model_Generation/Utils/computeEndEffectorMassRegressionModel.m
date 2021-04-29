function [Y_m,Beta_m] = computeEndEffectorMassRegressionModel(robot)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% 
% This function computes the regressionmodel of an end-effector mass. This
% is used for drive gain identification purpose. 

% Parse the pre-existing symbolic robot files:

HT_cmi_world_Moment = sym('A', [4 4 robot.nbDOF]);
J_dhi_world = sym('A', [6 robot.nbDOF robot.nbDOF]);
Jd_dhi_world = sym('A', [6 robot.nbDOF robot.nbDOF]);
J_cmi_world_Moment = sym('A', [6 robot.nbDOF robot.nbDOF]);
Jd_cmi_world_Moment = sym('A', [6 robot.nbDOF robot.nbDOF]);

for i=1:robot.nbDOF
    HT_cmi_world_Moment(:,:,i) = feval(sprintf('HT_cm%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry, robot.symbolicParameters.Moment*diag(1./robot.symbolicParameters.Mass));
    J_dhi_world(:,:,i) = feval(sprintf('J_dh%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry);
    Jd_dhi_world(:,:,i) = feval(sprintf('Jd_dh%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Qp, robot.symbolicParameters.Geometry);
    J_cmi_world_Moment(:,:,i) = feval(sprintf('J_cm%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry, robot.symbolicParameters.Moment*diag(1./robot.symbolicParameters.Mass));
    Jd_cmi_world_Moment(:,:,i) = feval(sprintf('Jd_cm%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Qp, robot.symbolicParameters.Geometry, robot.symbolicParameters.Moment*diag(1./robot.symbolicParameters.Mass));
end

[Y_r, Xhi_r] = blockTriangularDriveGains(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, robot);
[Y_m, Beta_m] = baseParametersNumericalSimplification(robot, Y_r, Xhi_r, 1000);

end


%% Compute the regressor in a block upper-triangular form as explained in [jung et al. 2018]:

function [Y, Xhi_ef] = blockTriangularDriveGains(J_dhi_world, Jd_dhi_world, J_cmi_world, Jd_cmi_world, HT_cmi_world, robot)

% Generate the parameter vector defined in [jung et al. 2018]:
Xhi_ef = robot.symbolicParameters.Xhi_ef;

% Compute the Block-Upper-Triangular Model Regressor:
Y = sym(zeros(robot.nbDOF,numel(Xhi_ef)));
for i = robot.nbDOF
    Ri = HT_cmi_world(1:3,1:3,i);
    W_cmi_world = J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp;
    Wp_cmi_world = Jd_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp + J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qpp;
    Vp_dhi_world = Jd_dhi_world(1:3,:,i)*robot.symbolicParameters.Qp + J_dhi_world(1:3,:,i)*robot.symbolicParameters.Qpp;
    
    a1 = J_dhi_world(1:3,:,i)'*(Vp_dhi_world + robot.symbolicParameters.Gravity);
    a2 = (J_dhi_world(1:3,:,i)'*Skew(Wp_cmi_world) + J_dhi_world(1:3,:,i)'*Skew(W_cmi_world)*Skew(W_cmi_world) - J_cmi_world(4:6,:,i)'*Skew(Vp_dhi_world + robot.symbolicParameters.Gravity))*Ri;
    a3 = J_cmi_world(4:6,:,i)'*(Ri*B(Ri'*Wp_cmi_world)+Skew(W_cmi_world)*Ri*B(Ri'*W_cmi_world));
    Y = [a3, a2, a1];
end

fprintf('Generating funcion handle for the regressor...\n')
matlabFunction(Y,'File','Benchmark/Robot_Generated_Data/Y_r_handle', 'Vars', {robot.symbolicParameters.Q, robot.symbolicParameters.Qp, robot.symbolicParameters.Qpp, robot.symbolicParameters.Geometry, robot.symbolicParameters.Gravity},'Optimize',true);

end

%% Numerical generation of Base Parameters using a QR decomposition of the regression matrix on multiple random epochs [Khalil 91]:

function [Y_b, Beta] = baseParametersNumericalSimplification(robot, Y_r, Xhi_r, samples)

disp('Generating the base parameters using QR decomposition...');
Geometry = robot.numericalParameters.Geometry;
Gravity = robot.numericalParameters.Gravity;
Obs = zeros(samples*robot.nbDOF,numel(Xhi_r));
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
Xhi_b = qr_Pb'*Xhi_r;
Xhi_d = qr_Pd'*Xhi_r;
Y_b = Y_r*qr_Pb;
Y_d = Y_r*qr_Pd;

% Generate a set of base parameters Beta such as Y_b*Beta = Y*Xhi = Tau:
Beta = Xhi_b + Kd*Xhi_d;

fprintf('\nBased on the analysis of the standard observation matrix, a set of %d independant base parameters could be computed...\n', rk)
delete Benchmark/Robot_Generated_Data/Y_r_handle.m
end

function [B] = B(v)
B = [[v(1); 0; 0], [v(2); v(1); 0], [v(3); 0; v(1)], [0; v(2); 0], [0; v(3); v(2)], [0; 0; v(3)]];
end
