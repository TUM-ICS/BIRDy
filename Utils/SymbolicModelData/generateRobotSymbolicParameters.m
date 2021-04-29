function  symbolicParameters = generateRobotSymbolicParameters(nbDOF, frictionIdentModel)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function generates the symbolic parameters required to describe a robot.
% These parameters will be used to build the symbolic kinematic, dynamic,
% friction and identification models of the robot.

% Allocating memory (Joint variables):
symbolicParameters.Q = sym(zeros(nbDOF,1));                                     % Joint position [rad]
symbolicParameters.Qp = sym(zeros(nbDOF,1));                                    % Joint velocity [rad/s]
symbolicParameters.Qpp = sym(zeros(nbDOF,1));                                   % Joint acceleration [rad/s^2]
symbolicParameters.Tau = sym(zeros(nbDOF,1));                                   % Joint torque [N.m]

% Allocating memory (Kinematic and Dynamic variables):
symbolicParameters.Geometry = sym(zeros(nbDOF,4));                              % Position of the robot DH links [m]
symbolicParameters.GeometryCOM = sym(zeros(3,nbDOF));                           % Position of the links' center of mass w.r.t the link's DH frame[m]
symbolicParameters.Moment = sym(zeros(3,nbDOF));                                % First Moment
symbolicParameters.Mass = sym(zeros(nbDOF,1));                                  % Mass of the robot links [kg]
symbolicParameters.InertiaDH = sym(zeros(3,3,nbDOF));                           % Inertia tensor around dh frame
symbolicParameters.InertiaCOM = sym(zeros(3,3,nbDOF));                          % Inertia tensor around COM
symbolicParameters.Ia = sym(zeros(nbDOF,1));                                    % Actuator and transmission inertias

% Allocating memory (Noise variables): uncomment in case you want to generate symbolic expressions for the EKF Jacobians:
% symbolicParameters.Noise = sym(zeros(12*nbDOF,1));
% symbolicParameters.Beta = sym(zeros(12*nbDOF,1));

% Allocating memory (Friction parameters):
symbolicParameters.Z = sym(zeros(nbDOF,1));                                     % LuGre State
symbolicParameters.friction.Fv = sym(zeros(nbDOF,1));                        	% Viscous Friction
symbolicParameters.friction.Fc = sym(zeros(nbDOF,1));                           % Coulomb Friction
symbolicParameters.friction.Fs = sym(zeros(nbDOF,1));                         	% Static Friction: only in Stribeck and LuGre models
symbolicParameters.friction.Vs = sym(zeros(nbDOF,1));                         	% Stribeck velocity: only in Stribeck and LuGre models
symbolicParameters.friction.Es = sym(zeros(nbDOF,1));                        	% Exponent: only in Stribeck and LuGre models
symbolicParameters.friction.Sigma_0 = sym(zeros(nbDOF,1));                      % Contact stiffness: only in LuGre model
symbolicParameters.friction.Sigma_1 = sym(zeros(nbDOF,1));                      % Damping coefficient of the bristle: only in LuGre model
symbolicParameters.friction.Sigma_2 = sym(zeros(nbDOF,1));                      % Viscous friction coefficient of the bristle: only in LuGre model
symbolicParameters.friction.Z0 = sym(zeros(nbDOF,1));                           % Initial deflection of the contacting asperities: only in LuGre model
symbolicParameters.friction.Tau_off = sym(zeros(nbDOF,1));                     	% Resulting nonlinear friction torque: only in Stribeck and LuGre models
symbolicParameters.friction.Fvm = sym(zeros(nbDOF,1));                        	% Coupling Viscous Friction
symbolicParameters.friction.Fcm = sym(zeros(nbDOF,1));                        	% Coupling Coulomb Friction

% Gravity:
gx = sym('gx','real');
gy = sym('gy','real');
gz = sym('gz','real');
symbolicParameters.Gravity = [gx;gy;gz];                                        % Gravity vector in world frame

% Control Period
symbolicParameters.dt = sym('dt','real');

% Symbolic parameters of a mass fixed to the end-effector (for robot drive-gains identification):
symbolicParameters.M_ef = sym('M_ef','real');
symbolicParameters.GeometryCOM_ef(1,1) = sym('X_ef','real');
symbolicParameters.GeometryCOM_ef(2,1) = sym('Y_ef','real');
symbolicParameters.GeometryCOM_ef(3,1) = sym('Z_ef','real');
symbolicParameters.Moment_ef(1,1) = sym('MX_ef','real');
symbolicParameters.Moment_ef(2,1) = sym('MY_ef','real');
symbolicParameters.Moment_ef(3,1) = sym('MZ_ef','real');
XX_ef = sym('XX_ef','real');
XY_ef = sym('XY_ef','real');
XZ_ef = sym('XZ_ef','real');
YY_ef = sym('YY_ef','real');
YZ_ef = sym('YZ_ef','real');
ZZ_ef = sym('ZZ_ef','real');
symbolicParameters.InertiaDH_ef = inertiaMatrix(XX_ef, XY_ef, XZ_ef, YY_ef, YZ_ef, ZZ_ef);
symbolicParameters.InertiaCOM_ef = inertiaMatrixDH2COM(symbolicParameters.InertiaDH_ef, symbolicParameters.M_ef, symbolicParameters.Moment_ef);
symbolicParameters.Xhi_ef = [XX_ef; XY_ef; XZ_ef; YY_ef; YZ_ef; ZZ_ef; symbolicParameters.Moment_ef; symbolicParameters.M_ef];

for i=1:nbDOF
    % Joint variables:
    symbolicParameters.Q(i,1) = sym(sprintf('q%d',i),'real');
    symbolicParameters.Qp(i,1) = sym(sprintf('qp%d',i),'real');
    symbolicParameters.Qpp(i,1) = sym(sprintf('qpp%d',i),'real');
    symbolicParameters.Tau(i,1) = sym(sprintf('tau%d',i),'real');
    
    % Robot link geometry [Beta d a alpha]:
    symbolicParameters.Geometry(i,1) = sym(sprintf('theta%d',i),'real');
    symbolicParameters.Geometry(i,2) = sym(sprintf('d%d',i),'real');
    symbolicParameters.Geometry(i,3) = sym(sprintf('a%d',i),'real');
    symbolicParameters.Geometry(i,4) = sym(sprintf('alpha%d',i),'real');
    
    % Robot link center of mass geometry:
    symbolicParameters.GeometryCOM(1,i) = sym(sprintf('X%d',i),'real');
    symbolicParameters.GeometryCOM(2,i) = sym(sprintf('Y%d',i),'real');
    symbolicParameters.GeometryCOM(3,i) = sym(sprintf('Z%d',i),'real');
    
    % Actuator and transmission inertias:
    symbolicParameters.Ia(i) = sym(sprintf('Ia%d',i),'real');
    
    % Robot link masses:
    symbolicParameters.Mass(i) = sym(sprintf('M%d',i),'real');
    
    % Robot link first moment:
    symbolicParameters.Moment(1,i) = sym(sprintf('MX%d',i),'real');
    symbolicParameters.Moment(2,i) = sym(sprintf('MY%d',i),'real');
    symbolicParameters.Moment(3,i) = sym(sprintf('MZ%d',i),'real');
    
    % Robot link inertias (around the link DH frame):
    XXi = sym(sprintf('XX%d',i),'real');
    XYi = sym(sprintf('XY%d',i),'real');
    XZi = sym(sprintf('XZ%d',i),'real');
    YYi = sym(sprintf('YY%d',i),'real');
    YZi = sym(sprintf('YZ%d',i),'real');
    ZZi = sym(sprintf('ZZ%d',i),'real');
    symbolicParameters.InertiaDH(:,:,i) = inertiaMatrix(XXi, XYi, XZi, YYi, YZi, ZZi);
    symbolicParameters.InertiaCOM(:,:,i) = inertiaMatrixDH2COM(symbolicParameters.InertiaDH(:,:,i), symbolicParameters.Mass(i), symbolicParameters.Moment(:,i));
    
    % Symbolic Friction parameters:
    symbolicParameters.Z(i,1) = sym(sprintf('Z%d',i),'real');
    symbolicParameters.friction.Fv(i,1) = sym(sprintf('Fv%d',i),'real');                % Viscous Friction
    symbolicParameters.friction.Fc(i,1) = sym(sprintf('Fc%d',i),'real');            	% Coulomb Friction
    symbolicParameters.friction.Fs(i,1) = sym(sprintf('Fs%d',i),'real');             	% Static Friction: only in Stribeck and LuGre models
    symbolicParameters.friction.Vs(i,1) = sym(sprintf('Vs%d',i),'real');            	% Stribeck velocity: only in Stribeck and LuGre models
    symbolicParameters.friction.Es(i,1) = sym(sprintf('Es%d',i),'real');                % Exponent: only in Stribeck and LuGre models
    symbolicParameters.friction.Sigma_0(i,1) = sym(sprintf('Sigma_0%d',i),'real');  	% Contact stiffness: only in LuGre model
    symbolicParameters.friction.Sigma_1(i,1) = sym(sprintf('Sigma_1%d',i),'real');  	% Damping coefficient of the bristle: only in LuGre model
    symbolicParameters.friction.Sigma_2(i,1) = sym(sprintf('Sigma_2%d',i),'real');   	% Viscous friction coefficient of the bristle: only in LuGre model
    symbolicParameters.friction.Z0(i,1) = sym(sprintf('Z0%d',i),'real');             	% Initial deflection of the contacting asperities: only in LuGre model
    symbolicParameters.friction.Tau_off(i,1) = sym(sprintf('tau_off%d',i),'real');      % Resulting nonlinear friction torque: only in Stribeck and LuGre models
    symbolicParameters.friction.Fvm(i,1) = sym(sprintf('Fvm%d',i),'real');              % Couping Viscous friction parameters as described in Gautier et al. 2011.
    symbolicParameters.friction.Fcm(i,1) = sym(sprintf('Fcm%d',i),'real');              % Couping Coulomb friction parameters as described in Gautier et al. 2011.
end

% Generate the vector Xhi of standard parameters:
[symbolicParameters.Xhi, ~, symbolicParameters.Xhi_aug] = getStandardParameterVector(symbolicParameters.InertiaDH, symbolicParameters.Moment, symbolicParameters.Mass, symbolicParameters.Ia, frictionIdentModel, symbolicParameters.friction);

end
