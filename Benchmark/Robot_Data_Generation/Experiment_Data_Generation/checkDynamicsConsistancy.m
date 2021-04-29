function [statusFlag] = checkDynamicsConsistancy(robotName, tol)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Check whether the functions used to compute the robot dynamics are
% accurate or not.

if nargin < 2
    tol = 1e-5;
end

options.loadSymbolic = false;
options.checkPhysicality = false;
options.noiseLevel = 0;
[robot] = loadRobotModelParameters(robotName, options);

dt = 0.001;
Z = zeros(robot.nbDOF,1);
Q = randn(robot.nbDOF,1);
Qp = randn(robot.nbDOF,1);
Qpp = randn(robot.nbDOF,1);

Beta = feval(sprintf('Regressor_Beta_%s', robotName),robot.numericalParameters.Xhi);
Yr = feval(sprintf('Regressor_Y_%s', robotName),Q, Qp, Qpp, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity);
Friction = feval(sprintf('Friction_%s', robotName),Qp, robot.numericalParameters.friction.Fv, robot.numericalParameters.friction.Fc, robot.numericalParameters.friction.Fvm, robot.numericalParameters.friction.Fcm, robot.numericalParameters.friction.Fs, robot.numericalParameters.friction.Vs, robot.numericalParameters.friction.Es, robot.numericalParameters.friction.Sigma_0, robot.numericalParameters.friction.Sigma_1, robot.numericalParameters.friction.Sigma_2, robot.numericalParameters.friction.Tau_off, Z, dt);
M = feval(sprintf('Inertia_M_%s', robotName),Q, robot.numericalParameters.Geometry, robot.numericalParameters.Xhi);
C = feval(sprintf('CorCen_C_%s', robotName),Q, Qp, robot.numericalParameters.Geometry, robot.numericalParameters.Xhi);
G = feval(sprintf('Gravity_G_%s', robotName),Q, robot.numericalParameters.Geometry, robot.numericalParameters.Xhi, robot.numericalParameters.Gravity);
M_beta = feval(sprintf('Inertia_M_Beta_%s', robotName),Q, robot.numericalParameters.Geometry, Beta);
h_beta = feval(sprintf('CorCen_h_Beta_%s', robotName),Q, Qp, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Beta);

switch robot.frictionDatagenModel
    % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
    % Nonlinear friction models require state dependant parameter identification
    case 'no'
        Tau_friction = Friction(:,1);
    case 'Viscous'
        Tau_friction = Friction(:,2);
    case 'Coulomb'
        Tau_friction = Friction(:,3);
    case 'ViscousCoulomb'
        Tau_friction = Friction(:,4);
    case 'ViscousCoulombOff'
        Tau_friction = Friction(:,5);
    case 'Stribeck'
        Tau_friction = Friction(:,6);
    case 'LuGre'
        Tau_friction = Friction(:,7);
    otherwise
        Tau_friction = Friction(:,1);
end

disp('Looking for inconsistency between the dynamic model and the identification model:');
Tau_regressor = Yr*Beta;
Tau_manual = M*Qpp + C*Qp + G + Tau_friction;

Err_tau = Tau_regressor-Tau_manual;
Err_M = M_beta-M;
Err_h =  h_beta  - (C*Qp + G + Tau_friction);

fprintf('     Tolerance = %d\n',tol);
if norm(Err_tau)>tol
    fprintf('     norm(torqueError) = %d > Tolerance ==> NOT OK\n',norm(Err_tau));
    statusFlag1 = false;
else
    fprintf('     norm(torqueError) = %d < Tolerance ==> OK\n',norm(Err_tau));
    statusFlag1 = true;
end
if norm(Err_M)>tol
    fprintf('     norm(inertiaMatrixError) = %d > Tolerance ==> NOT OK\n',norm(Err_M));
    statusFlag2 = false;
else
    fprintf('     norm(inertiaMatrixError) = %d < Tolerance ==> OK\n',norm(Err_M));
    statusFlag2 = true;
end
if norm(Err_h)>tol
    fprintf('     norm(corcenError) = %d > Tolerance ==> NOT OK\n',norm(Err_h));
    statusFlag3 = false;
else
    fprintf('     norm(corcenError) = %d < Tolerance ==> OK\n',norm(Err_h));
    statusFlag3 = true;
end

statusFlag = statusFlag1 && statusFlag2 && statusFlag3;

end

