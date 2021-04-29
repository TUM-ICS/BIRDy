function [robot] = TX40(options, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% The TX40 description function.

% Parameters of the TX-40 (DH parameters in proximal/modified convension):
% Kinematics 	theta [rad] 	a [m]       d [m]       alpha [rad] 	Dynamics 	Mass [kg] 	Center of Mass [m]
% Joint 1       0               0           0        	0               Link 1      8.61        [?,?,?]
% Joint 2       -π/2            0           0           -π/2            Link 2      4.85        [?,?,?]
% Joint 3       π/2             0.225 	    0.035       0               Link 3      4.55        [?,?,?]
% Joint 4       0               0           0.225 	    π/2             Link 4      4.09        [?,?,?]
% Joint 5       0               0           0        	-π/2            Link 5      0.32        [?,?,?]
% Joint 6       π               0           0           π/2             Link 6      0.2        [?,?,?]

% General Informations:
robot.name = 'TX40';                                                    % Name of the robot
robot.nbDOF = 6;                                                        % Number of Degrees Of Freedom (DOF)
robot.rootFrame.name = 'world';                                         % Name of the world reference frame
robot.rootFrame.transform = sym(eye(4));                              	% Homogeneous transform between the world reference frame and the robot base frame
robot.rootFrame.transform(3,4) = 0.32;
robot.jointType = ['revol';'revol';'revol';'revol';'revol';'revol'];    % Type of the robot joints: either 'revol' or 'prism'

% The idea behind 'frictionDatagenModel' and 'frictionIdentModel' is to be able to generate a robot model with linear or nonlinear friction and to identify it with several friction models in order to observe the biases
robot.frictionDatagenModel = 'ViscousCoulomb';                          % Type of friction model used for robot data generation: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'ViscousCoulombOff', 'Stribeck', 'LuGre'
robot.frictionIdentModel = 'ViscousCoulomb';                            % Type of friction model used for robot identification: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'ViscousCoulombOff', 'Stribeck', 'LuGre'

% Symbolic Parameters:
if options.loadSymbolic == true
    robot.symbolicParameters = generateRobotSymbolicParameters(robot.nbDOF, robot.frictionIdentModel);
    
    robot.symbolicParameters.Xhi = [robot.symbolicParameters.Xhi; robot.symbolicParameters.friction.Fvm(6); robot.symbolicParameters.friction.Fcm(6)];
    robot.symbolicParameters.Xhi_aug = [robot.symbolicParameters.Xhi_aug; robot.symbolicParameters.friction.Fvm(6); robot.symbolicParameters.friction.Fcm(6)];
    
    % Denavit-Hartenerg Symbolic Parameters:
    robot.dhParameter.theta = [robot.symbolicParameters.Q(1); robot.symbolicParameters.Q(2)-pi/2; robot.symbolicParameters.Q(3)+pi/2; robot.symbolicParameters.Q(4); robot.symbolicParameters.Q(5); robot.symbolicParameters.Q(6)+pi];
    robot.dhParameter.d = [0; 0; robot.symbolicParameters.Geometry(3,2); robot.symbolicParameters.Geometry(4,2); 0; 0];
    robot.dhParameter.a = [0; 0; robot.symbolicParameters.Geometry(3,3); 0; 0; 0];
    robot.dhParameter.alpha = sym([0; -pi/2; 0; pi/2; -pi/2; pi/2]);
    robot.dhConvention = 'proximal';                                    % Robot Denavit-Hartenerg convention: either 'proximal' or 'distal'
end

% Numerical Values of Dynamic Parameters:
robot.numericalParameters.Geometry = ...
    [0      0       0       0;...
    0       0      	0    	-pi/2;...
    0       0.035  	0.225 	0;...
    0       0.225  	0     	pi/2;...
    0       0      	0     	-pi/2;...
    0       0     	0     	pi/2];                                      % Position of the robot DH links [m]

robot.numericalParameters.GeometryCOM = [[0; 0; -0.05] [3.92e-1; -7.2e-3; 1.62e-1]./4.85 [3.26e-2;2.44e-2;2.65e-1]./4.07 [1.45e-2;-7.24e-3;-5.86e-1]./3.62 [0; -3.06e-3; -1.02e-3]./1.02 [0; 0; 8.4e-3]./0.2];   % Position of the links' center of mass w.r.t the link's DH frame[m]
robot.numericalParameters.Mass = [8.61; 4.85; 4.07; 3.62; 1.02; 0.2];	% Mass of the robot links [kg]
robot.numericalParameters.Gravity = [0;0;9.81];                         % Gravity vector in world frame [m/s²]
robot.numericalParameters.Q0 = zeros(robot.nbDOF,1);                    % Robot initial joint angles
robot.numericalParameters.Moment = robot.numericalParameters.GeometryCOM*diag(robot.numericalParameters.Mass);

%         robot.numericalParameters.friction.Fv = [11.90e0;8.05e0;4.12e0;1.48e0;1.60;0.60];       % Viscous Friction
robot.numericalParameters.friction.Fv = [7.9379;5.6948;1.9804;1.1141;1.60;0.6894];
%         robot.numericalParameters.friction.Fc = [4.21e0;5.55e0;5.06e0;2.07e0;3.75;0.26];	    % Coulomb Friction
robot.numericalParameters.friction.Fc = [6.9822;7.8301;6.1939;2.4063;3.4092;1.8313];
robot.numericalParameters.friction.Fvm = [0;0;0;0;0;0.8];                               % Coupling between joints 5 and 6 described in Gautier et al. 2011.
robot.numericalParameters.friction.Fcm = [0;0;0;0;0;1.6];                               % Coupling between joints 5 and 6 described in Gautier et al. 2011.
robot.numericalParameters.friction.Fs = [4.78e0;7.97e0;5.81e0;2.15e0;0;0];              % Static Friction: only in Stribeck and LuGre models
robot.numericalParameters.friction.Vs = [2.5e-2; 5e-2; 5e-2; 5e-2; 5e-2; 5e-2];      	% Stribeck velocity: only in Stribeck and LuGre models
robot.numericalParameters.friction.Es = [2.09e0; 2.78e0; 2.72e0; 1.60e0; 2; 2];        	% Exponent: only in Stribeck and LuGre models
robot.numericalParameters.friction.Sigma_0 = 0.1*ones(robot.nbDOF,1);                   % Contact stiffness: only in LuGre model
robot.numericalParameters.friction.Sigma_1 = 0.1*ones(robot.nbDOF,1);                   % Damping coefficient of the bristle: only in LuGre model
robot.numericalParameters.friction.Sigma_2 = 0.1*ones(robot.nbDOF,1);               	% Viscous friction coefficient of the bristle: only in LuGre model
robot.numericalParameters.friction.Z0 = zeros(robot.nbDOF,1);                           % Average deflection of the contacting asperities: only in LuGre model
robot.numericalParameters.friction.Tau_off = zeros(robot.nbDOF,1);                      % Friction torque parameter for nonlinear friction models (Stribeck and LuGre)

% Inertias:
robot.numericalParameters.Ia = [3.62e-01; 5.07e-01; 8.29e-2; 3.41e-2; 3.61e-2; 1.14e-2];
robot.numericalParameters.InertiaDH = zeros(3,3,robot.nbDOF);                          % Inertias are here provided manually...
robot.numericalParameters.InertiaDH(:,:,1) = inertiaMatrix(0.424, 0, 0, 0.424, 0, 0.352);
robot.numericalParameters.InertiaDH(:,:,2) = inertiaMatrix(1.63e-2, 7.85e-4, -1.57e-2, 8.81e-02, 3.24e-4, 8.28e-2);
robot.numericalParameters.InertiaDH(:,:,3) = inertiaMatrix(2.23e-2, -1.95e-4, -1.16e-3, 2.24e-2, -2.22e-3, 4.41e-3);
robot.numericalParameters.InertiaDH(:,:,4) = inertiaMatrix(1.09e-1, 2.9e-5, 1.35e-3, 1.08e-01, -1.17e-03, 4.07e-03);
robot.numericalParameters.InertiaDH(:,:,5) = inertiaMatrix(1.01e-3, 0, 0, 1.00e-03, -3.06e-6, 1.01e-3);
robot.numericalParameters.InertiaDH(:,:,6) = inertiaMatrix(3.53e-3, 0, 0, 3.53e-3, 0, 3.53e-4);
robot.numericalParameters.InertiaCOM = inertiaTensorDH2COM(robot.numericalParameters.InertiaDH, robot.numericalParameters.Mass, robot.numericalParameters.Moment);
[robot.numericalParameters.Xhi, ~,~,robot.numericalParameters.numParam] = aggregateNumericalParameters(robot.nbDOF, robot.frictionIdentModel, robot.numericalParameters);
robot.numericalParameters.Xhi = [robot.numericalParameters.Xhi; robot.numericalParameters.friction.Fvm(6); robot.numericalParameters.friction.Fcm(6)]; % TO ACCOUNT FOR COUPLING !
robot.numericalParameters.numParam(end) = robot.numericalParameters.numParam(end)+2;
% PID Control Gains:
robot.controlParameters.controlFrequency = 5e3;                                         % Control frequency [Hz]
robot.controlParameters.samplingFrequency = 1e3;                                        % Sampling frequency [Hz]
%         k = [0.1;0.1;0.2;1;1;1.5];
k = 10*[40;40;20;10;10;1];
%         k = 7*[40;40;40;10;10;5];   % work with the real robot
zeta = robot.numericalParameters.friction.Fv./(2*k.*(robot.numericalParameters.Mass).^(1/2));
omega = (k./robot.numericalParameters.Mass).^(1/2);
robot.controlParameters.antiWindup = 5;                                                 % Anti-windup for the integral component of the PID
robot.controlParameters.Kp = diag(robot.numericalParameters.Mass.*omega.^2);            % Proportional gains
robot.controlParameters.Ki = zeros(robot.nbDOF);                                        % Integral Gains
robot.controlParameters.Kd = diag([50;50;25;20;20;2])*diag(robot.numericalParameters.Mass.*zeta.*omega); % Derivative Gains
robot.controlParameters.gearRatio = diag([32;32;45;48;45;32]);
robot.controlParameters.Ktau = diag([35.24;32.04;23.94;9.05;15.5;6.56]);                % Torque-loop gain (current loopgain times torque constant of the motor times gear ratio)
robot.controlParameters.Ktau_sign = ones(robot.nbDOF,1);

% Noise parameters:
robot.numericalParameters.sd_q = [0.057e-3; 0.057e-3; 0.122e-3; 0.114e-3; 0.122e-3; 0.172e-3].*pi/180; % Noise standard deviation
robot.numericalParameters.sd_tau = 5e-2*ones(robot.nbDOF,1);                % Noise standard deviation


% 455.8254  449.0398  253.7919   71.2182  244.3226  478.4783    2.5361    4.1174    1.1793    0.5462    0.5714    0.1954
robot.controlParameters.Kp = 0.75*diag([455.8254  500  253.7919   200.2182  244.3226  478.4783]);   
robot.controlParameters.Kd = diag([2.5361    4.1174    1.1793    0.5462    0.5714    0.1954]);

% Physical Constraints:
robot.physicalConstraints.limQ_U = [180;125;138;270;133.5;270].*pi/180;     % Joint position upper limit [rad]
robot.physicalConstraints.limQp_U = [287;287;430;410;320;700].*pi/180;      % Joint velocity upper limit [rad/s]
robot.physicalConstraints.limQpp_U = 50*ones(robot.nbDOF,1);                % Joint acceleration upper limit [rad/s^2]
robot.physicalConstraints.limTau_U = 1000*[11.5;11.5;7.1;2.7;2.6;0.6];   	% Joint torque upper limit [N.m]
robot.physicalConstraints.limQ_L = -[180;125;138;270;120;270].*pi/180;      % Joint position lower limit [rad]
robot.physicalConstraints.limQp_L = -[287;287;430;410;320;700].*pi/180;     % Joint velocity lower limit [rad/s]
robot.physicalConstraints.limQpp_L = -50*ones(robot.nbDOF,1);               % Joint acceleration lower limit [rad/s^2]
robot.physicalConstraints.limTau_L = -1000*[11.5;11.5;7.1;2.7;2.6;0.6]; 	% Joint torque lower limit [N.m]

% robot.controlParameters.Gains_L = zeros(2*robot.nbDOF,1);
% robot.controlParameters.Gains_U = [500*ones(robot.nbDOF,1);20*ones(robot.nbDOF,1)];
% robot.controlParameters.Kp
% robot.controlParameters.Kd
end

