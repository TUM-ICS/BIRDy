function [robot] = RV2SQ(options, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% The RV2SQ description function.

% Parameters of the Mitsubishi RV2SQ (DH parameters in proximal/modified convension):
% Kinematics 	theta [rad] 	a [m]       d [m]       alpha [rad] 	Dynamics 	Mass [kg] 	Center of Mass [m]
% Joint 1       0               0           d1        	0               Link 1      4               [?,?,?]
% Joint 2       -π/2            0         	0           -π/2           	Link 2      4.5             [?,?,?]
% Joint 3       +π/2            a3          0           0               Link 3      4               [?,?,?]
% Joint 4       0              -a4          d4          +π/2           	Link 4      3.5             [?,?,?]
% Joint 5       0               0           0        	-π/2            Link 5      0.45            [?,?,?]
% Joint 6       0               0           d6          +π/2          	Link 6      0.05            [?,?,?]

% General Informations:
robot.name = 'RV2SQ';                                                  % Name of the robot
robot.nbDOF = 6;                                                        % Number of Degrees Of Freedom (DOF)
robot.rootFrame.name = 'world';                                         % Name of the world reference frame
robot.rootFrame.transform = sym(eye(4));                              	% Homogeneous transform between the world reference frame and the robot base frame
robot.rootFrame.transform(3,4) = 0;
robot.jointType = ['revol';'revol';'revol';'revol';'revol';'revol'];    % Type of the robot joints: either 'revol' or 'prism'

% The idea behind 'frictionDatagenModel' and 'frictionIdentModel' is to be able to generate a robot model with linear or nonlinear friction and to identify it with several friction models in order to observe the biases
robot.frictionDatagenModel = 'ViscousCoulomb';                          % Type of friction model used for robot data generation: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'Stribeck', 'LuGre'
robot.frictionIdentModel = 'ViscousCoulomb';                            % Type of friction model used for robot identification: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'Stribeck', 'LuGre'

% Symbolic Parameters:
if options.loadSymbolic == true
    robot.symbolicParameters = generateRobotSymbolicParameters(robot.nbDOF, robot.frictionIdentModel);
    
    % Denavit-Hartenerg Symbolic Parameters:
    robot.dhParameter.theta = [robot.symbolicParameters.Q(1); robot.symbolicParameters.Q(2)-pi/2; robot.symbolicParameters.Q(3)+pi/2; robot.symbolicParameters.Q(4); robot.symbolicParameters.Q(5); robot.symbolicParameters.Q(6)];
    robot.dhParameter.d = [robot.symbolicParameters.Geometry(1,2); 0; 0; robot.symbolicParameters.Geometry(4,2); 0; robot.symbolicParameters.Geometry(6,2)];
    robot.dhParameter.a = [0; 0; robot.symbolicParameters.Geometry(3,3); robot.symbolicParameters.Geometry(4,3); 0; 0];
    robot.dhParameter.alpha = sym([0; -pi/2; 0; pi/2; -pi/2; pi/2]);
    robot.dhConvention = 'proximal';                                        % Robot Denavit-Hartenerg convention: either 'proximal' or 'distal'
end

% Numerical Values of Dynamic Parameters:
robot.numericalParameters.Geometry = ...
    [0          0.295       0         	 0;...
    0           0       	0            -pi/2;...
    0           0         	0.23         0;...
    0           0.27      	-0.05        pi/2;...
    0           0          	0           -pi/2;...
    0           0.07       	0            pi/2];                             	% Position of the robot DH links [m]


robot.numericalParameters.GeometryCOM = [[0; 0; -0.05] [0.06; -0.0076; 0] [0; -0.045; 0] [0.01; 0; -0.13] [-0.001; -0.001; 0.001] [0.001; 0; 0.0005]];   % Position of the links' center of mass w.r.t the link's DH frame[m]
robot.numericalParameters.Mass = [4; 5.3; 4.1; 2.8; 0.8; 0.7];	% Mass of the robot links [kg]
robot.numericalParameters.Gravity = [0;0;9.81];                         % Gravity vector in world frame [m/s²]
robot.numericalParameters.Q0 = [0;0;pi/4;0;0;0];                        % Robot initial joint angles
robot.numericalParameters.Moment = robot.numericalParameters.GeometryCOM*diag(robot.numericalParameters.Mass);

robot.numericalParameters.friction.Fv = [4.7618;13.4593;4.1019;1.3165;1.4242;0.9244];       % Viscous Friction
robot.numericalParameters.friction.Fc = [2.8503;7.7407;3.3459;1.6581;2.7170;2.0284];	    % Coulomb Friction
robot.numericalParameters.friction.Fvm = zeros(robot.nbDOF,1);          % Couping between joints.
robot.numericalParameters.friction.Fcm = zeros(robot.nbDOF,1);          % Couping between joints.
robot.numericalParameters.friction.Fs = [4.78e0;7.97e0;5.81e0;2.15e0;0;0];% Static Friction: only in Stribeck and LuGre models
robot.numericalParameters.friction.Vs = [2.5e-2; 5e-2; 5e-2; 5e-2; 5e-2; 5e-2];      	% Stribeck velocity: only in Stribeck and LuGre models
robot.numericalParameters.friction.Es = [2.09e0; 2.78e0; 2.72e0; 1.60e0; 2; 2];        	% Exponent: only in Stribeck and LuGre models
robot.numericalParameters.friction.Sigma_0 = 0.1*ones(robot.nbDOF,1); 	% Contact stiffness: only in LuGre model
robot.numericalParameters.friction.Sigma_1 = 0.1*ones(robot.nbDOF,1);  	% Damping coefficient of the bristle: only in LuGre model
robot.numericalParameters.friction.Sigma_2 = 0.1*ones(robot.nbDOF,1);  	% Viscous friction coefficient of the bristle: only in LuGre model
robot.numericalParameters.friction.Z0 = zeros(robot.nbDOF,1);          	% Average deflection of the contacting asperities: only in LuGre model
robot.numericalParameters.friction.Tau_off = zeros(robot.nbDOF,1);     	% Friction torque parameter for nonlinear friction models (Stribeck and LuGre)

robot.numericalParameters.Ia = [1e-02; 5e-01; 0.3253; 0.0416; 0.0939; 0.0317];
robot.numericalParameters.InertiaCOM = zeros(3,3,robot.nbDOF);                          % Inertias are here provided manually...
robot.numericalParameters.InertiaCOM(:,:,1) = inertiaMatrix(0.0424, 0, 0, 0.0424, 0, 0.0152);
robot.numericalParameters.InertiaCOM(:,:,2) = inertiaMatrix(0.45, 2.03e-03, -2.03e-04, 5e-01, -5.56e-03, 0.85);
robot.numericalParameters.InertiaCOM(:,:,3) = inertiaMatrix(0.177, -0.017, 0.001, 0.105, 0.006, 0.239);
robot.numericalParameters.InertiaCOM(:,:,4) = inertiaMatrix(0.0492, -0.0013, 0.0042, 0.0424, 0.0003, 0.0457);
robot.numericalParameters.InertiaCOM(:,:,5) = inertiaMatrix(0.0676, 0.0001, 0, 0.0274, 0, 0.0718);
robot.numericalParameters.InertiaCOM(:,:,6) = inertiaMatrix(0.0076, 0, 0, 0.0074, 0, 0.0018);
robot.numericalParameters.InertiaDH = inertiaTensorCOM2DH(robot.numericalParameters.InertiaCOM, robot.numericalParameters.Mass, robot.numericalParameters.Moment);
[robot.numericalParameters.Xhi, ~,~,robot.numericalParameters.numParam] = aggregateNumericalParameters(robot.nbDOF, robot.frictionIdentModel, robot.numericalParameters);
% PID Control Gains:
robot.controlParameters.antiWindup = 5;
robot.controlParameters.controlFrequency = 5e3;
robot.controlParameters.samplingFrequency = 140; % Data acquisition can be done at a period of 7.1ms or 140 Hz
k = 1000*[100;300;100;100;50;50];
zeta = robot.numericalParameters.friction.Fv./(2*k.*(robot.numericalParameters.Mass).^(1/2));
omega = (k./robot.numericalParameters.Mass).^(1/2);
robot.controlParameters.Kp = diag(robot.numericalParameters.Mass.*omega.^2);            % Proportional gains
robot.controlParameters.Ki = zeros(robot.nbDOF);                                        % Integral Gains
robot.controlParameters.Kd = 1e5*diag(robot.numericalParameters.Mass.*zeta.*omega);       % Derivative Gains
robot.controlParameters.Ktau = diag([0.0347;0.0845;0.0412;0.0165;0.0173;0.0145]); % Torque-loop gain (current loopgain times torque constant of the motor times gear ratio)
robot.controlParameters.Ktau_sign = [-1; -1; 1; -1; -1; -1];

% Noise parameters:

switch options.noiseLevel
    
    case 'oldNoise'
        robot.numericalParameters.sd_q = [0.057e-3; 0.057e-3; 0.122e-3; 0.114e-3; 0.122e-3; 0.172e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = [0.0371;0.1105;0.0611;0.0149;0.0153;0.0211];   % Noise standard deviation
    
    case 'lowPositionNoise'
        % Standard
        robot.numericalParameters.sd_q = [1e-4; 1e-4; 1e-4; 1e-4; 1e-4; 1e-4].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation 
        
    case 'standardNoise'
        % Standard
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highTorqueNoise'
        % High torque noise
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 10*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highPositionNoise'
        % High position noise
        robot.numericalParameters.sd_q = 10*[1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highPositionTorqueNoise'
        % High torque and position noise
        robot.numericalParameters.sd_q = 10*[1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 10*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    otherwise
        % Standard
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
end


% Physical Constraints:
robot.physicalConstraints.limQ_U = [240;120;160;200;90;360].*pi/180;     % Joint position upper limit [rad] ([240;120;160;200;120;360].*pi/180;)
robot.physicalConstraints.limQp_U = [225;150;275;412;450;720].*pi/180;      % Joint velocity upper limit [rad/s]
robot.physicalConstraints.limQpp_U = 50*ones(robot.nbDOF,1);                % Joint acceleration upper limit [rad/s^2]
robot.physicalConstraints.limTau_U = 10*[11.5;11.5;7.1;4.17;4.17;2.75];   	% Joint torque upper limit [N.m]
robot.physicalConstraints.limQ_L = -[240;120;0;200;90;360].*pi/180;      % Joint position lower limit [rad] (-[240;120;0;200;120;360].*pi/180;)
robot.physicalConstraints.limQp_L = -[225;150;275;412;450;720].*pi/180;     % Joint velocity lower limit [rad/s]
robot.physicalConstraints.limQpp_L = -50*ones(robot.nbDOF,1);               % Joint acceleration lower limit [rad/s^2]
robot.physicalConstraints.limTau_L = -10*[11.5;11.5;7.1;4.17;4.17;2.75]; 	% Joint torque lower limit [N.m]

robot.controlParameters.Kp = 1.0e+05 *diag([5 ;   10  ;  9 ;   8 ;   5  ;  5]);
robot.controlParameters.Kd = 1.0e+02 *diag([5 ;   10  ;  9 ;   8 ;   5  ;  5]);

robot.controlParameters.Kp*robot.controlParameters.Ktau
robot.controlParameters.Kd*robot.controlParameters.Ktau
end



% function [robot] = RV2SQ(options, varargin)
% 
% % Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% %
% % The RV2SQ description function.
% 
% % Parameters of the Mitsubishi RV2SQ (DH parameters in proximal/modified convension):
% % Kinematics 	theta [rad] 	a [m]       d [m]       alpha [rad] 	Dynamics 	Mass [kg] 	Center of Mass [m]
% % Joint 1       0               0           d1        	0               Link 1      4               [?,?,?]
% % Joint 2       -π/2            0         	0           -π/2           	Link 2      4.5             [?,?,?]
% % Joint 3       +π/2            a3          0           0               Link 3      4               [?,?,?]
% % Joint 4       0              -a4          d4          +π/2           	Link 4      3.5             [?,?,?]
% % Joint 5       0               0           0        	-π/2            Link 5      0.45            [?,?,?]
% % Joint 6       0               0           d6          +π/2          	Link 6      0.05            [?,?,?]
% 
% % General Informations:
% robot.name = 'RV2SQ';                                                  % Name of the robot
% robot.nbDOF = 6;                                                        % Number of Degrees Of Freedom (DOF)
% robot.rootFrame.name = 'world';                                         % Name of the world reference frame
% robot.rootFrame.transform = sym(eye(4));                              	% Homogeneous transform between the world reference frame and the robot base frame
% robot.rootFrame.transform(3,4) = 0;
% robot.jointType = ['revol';'revol';'revol';'revol';'revol';'revol'];    % Type of the robot joints: either 'revol' or 'prism'
% 
% % The idea behind 'frictionDatagenModel' and 'frictionIdentModel' is to be able to generate a robot model with linear or nonlinear friction and to identify it with several friction models in order to observe the biases
% robot.frictionDatagenModel = 'ViscousCoulomb';                          % Type of friction model used for robot data generation: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'Stribeck', 'LuGre'
% robot.frictionIdentModel = 'ViscousCoulomb';                            % Type of friction model used for robot identification: 'no', 'Viscous', 'Coulomb', 'ViscousCoulomb', 'Stribeck', 'LuGre'
% 
% % Symbolic Parameters:
% if options.loadSymbolic == true
%     robot.symbolicParameters = generateRobotSymbolicParameters(robot.nbDOF, robot.frictionIdentModel);
%     
%     % Denavit-Hartenerg Symbolic Parameters:
%     robot.dhParameter.theta = [robot.symbolicParameters.Q(1); robot.symbolicParameters.Q(2)-pi/2; robot.symbolicParameters.Q(3)+pi/2; robot.symbolicParameters.Q(4); robot.symbolicParameters.Q(5); robot.symbolicParameters.Q(6)];
%     robot.dhParameter.d = [robot.symbolicParameters.Geometry(1,2); 0; 0; robot.symbolicParameters.Geometry(4,2); 0; robot.symbolicParameters.Geometry(6,2)];
%     robot.dhParameter.a = [0; 0; robot.symbolicParameters.Geometry(3,3); robot.symbolicParameters.Geometry(4,3); 0; 0];
%     robot.dhParameter.alpha = sym([0; -pi/2; 0; pi/2; -pi/2; pi/2]);
%     robot.dhConvention = 'proximal';                                        % Robot Denavit-Hartenerg convention: either 'proximal' or 'distal'
% end
% 
% % Numerical Values of Dynamic Parameters:
% robot.numericalParameters.Geometry = ...
%     [0          0.295       0         	 0;...
%     0           0       	0            -pi/2;...
%     0           0         	0.23         0;...
%     0           0.27      	-0.05        pi/2;...
%     0           0          	0           -pi/2;...
%     0           0.07       	0            pi/2];                             	% Position of the robot DH links [m]
% 
% 
% robot.numericalParameters.GeometryCOM = [[0; 0; -0.05] [0.06; -0.0076; 0] [0; -0.045; 0] [0.01; 0; -0.13] [-0.001; -0.001; 0.001] [0.001; 0; 0.0005]];   % Position of the links' center of mass w.r.t the link's DH frame[m]
% robot.numericalParameters.Mass = [4; 5.3; 4.5; 2.8; 0.8; 0.7];	% Mass of the robot links [kg]
% robot.numericalParameters.Gravity = [0;0;9.81];                         % Gravity vector in world frame [m/s²]
% robot.numericalParameters.Q0 = [0;0;pi/4;0;0;0];                        % Robot initial joint angles
% 
% robot.numericalParameters.friction.Fv = [8.36;11.3;3.6;1.3322;1.30;2.10];       % Viscous Friction
% robot.numericalParameters.friction.Fc = [5.61;7.7407;3.3459;1.6581;2.7170;5.25];	    % Coulomb Friction
% robot.numericalParameters.friction.Fvm = zeros(robot.nbDOF,1);          % Couping between joints.
% robot.numericalParameters.friction.Fcm = zeros(robot.nbDOF,1);          % Couping between joints.
% robot.numericalParameters.friction.Fs = [4.78e0;7.97e0;5.81e0;2.15e0;0;0];% Static Friction: only in Stribeck and LuGre models
% robot.numericalParameters.friction.Vs = [2.5e-2; 5e-2; 5e-2; 5e-2; 5e-2; 5e-2];      	% Stribeck velocity: only in Stribeck and LuGre models
% robot.numericalParameters.friction.Es = [2.09e0; 2.78e0; 2.72e0; 1.60e0; 2; 2];        	% Exponent: only in Stribeck and LuGre models
% robot.numericalParameters.friction.Sigma_0 = 0.1*ones(robot.nbDOF,1); 	% Contact stiffness: only in LuGre model
% robot.numericalParameters.friction.Sigma_1 = 0.1*ones(robot.nbDOF,1);  	% Damping coefficient of the bristle: only in LuGre model
% robot.numericalParameters.friction.Sigma_2 = 0.1*ones(robot.nbDOF,1);  	% Viscous friction coefficient of the bristle: only in LuGre model
% robot.numericalParameters.friction.Z0 = zeros(robot.nbDOF,1);          	% Average deflection of the contacting asperities: only in LuGre model
% robot.numericalParameters.friction.Tau_off = zeros(robot.nbDOF,1);     	% Friction torque parameter for nonlinear friction models (Stribeck and LuGre)
% 
% robot.numericalParameters.Ia = [0.2; 0.5; 0.43; 0.0416; 0.0939; 0.0317];
% robot.numericalParameters.InertiaCOM = zeros(3,3,robot.nbDOF);                          % Inertias are here provided manually...
% robot.numericalParameters.InertiaCOM(:,:,1) = inertiaMatrixDH2COM(inertiaMatrix(0.0424, 0, 0, 0.0424, 0, 0.752), robot.numericalParameters.Mass(1), robot.numericalParameters.Moment(:,1));
% robot.numericalParameters.InertiaCOM(:,:,2) = inertiaMatrixDH2COM(inertiaMatrix(0.01, 2.03e-03, -0.0149, 1e-03, 5.56e-03, 0.95), robot.numericalParameters.Mass(2), robot.numericalParameters.Moment(:,2));
% robot.numericalParameters.InertiaCOM(:,:,3) = inertiaMatrixDH2COM(inertiaMatrix(1.3e-2, -6.97e-03, 2.20e-03, 5e-03, 4.53e-03, 1.58e-01), robot.numericalParameters.Mass(3), robot.numericalParameters.Moment(:,3));
% robot.numericalParameters.InertiaCOM(:,:,4) = inertiaMatrixDH2COM(inertiaMatrix(5.60e-03, -3.66e-03, -2.60e-03, 4.08e-02, -6.64e-03, 3.78e-03), robot.numericalParameters.Mass(4), robot.numericalParameters.Moment(:,4));
% robot.numericalParameters.InertiaCOM(:,:,5) = inertiaMatrixDH2COM(inertiaMatrix(0.98e-02, -9.30e-04, -0.015, 7.00e-03, -0.0081, 3.93e-02), robot.numericalParameters.Mass(5), robot.numericalParameters.Moment(:,5));
% robot.numericalParameters.InertiaCOM(:,:,6) = inertiaMatrixDH2COM(inertiaMatrix(4.71e-03, 0.0026, -0.000070, 3.53e-02, -4.41e-05, 0.0148), robot.numericalParameters.Mass(6), robot.numericalParameters.Moment(:,6));
% robot.numericalParameters.InertiaDH = inertiaTensorCOM2DH(robot.numericalParameters.InertiaCOM, robot.numericalParameters.Mass, robot.numericalParameters.Moment);
% [robot.numericalParameters.Xhi, ~,~,~] = aggregateNumericalParameters(robot.nbDOF, robot.frictionIdentModel, robot.numericalParameters);
% % PID Control Gains:
% robot.controlParameters.antiWindup = 5;
% robot.controlParameters.controlFrequency = 5e3;
% robot.controlParameters.samplingFrequency = 140; % Data acquisition can be done at a period of 7.1ms or 140 Hz
% k = 5e3;
% zeta = robot.numericalParameters.friction.Fv./(2*k.*(robot.numericalParameters.Mass).^(1/2));
% omega = (k./robot.numericalParameters.Mass).^(1/2);
% robot.controlParameters.Kp = diag(robot.numericalParameters.Mass.*omega.^2);            % Proportional gains
% robot.controlParameters.Ki = zeros(robot.nbDOF);                                        % Integral Gains
% robot.controlParameters.Kd = 1e2*diag(robot.numericalParameters.Mass.*zeta.*omega);       % Derivative Gains
% K = [0.0692;
%    0.0843;
%     0.0413;
%    0.0172;
%    0.0169;
%   0.0353];
% robot.controlParameters.Ktau = diag(K); %diag([0.0347;0.0845;0.0412;0.0165;0.0173;0.0145]); % Torque-loop gain (current loopgain times torque constant of the motor times gear ratio)
% robot.controlParameters.Ktau_sign = [-1; -1; 1; -1; -1; -1];
% 
% % Noise parameters:
% robot.numericalParameters.sd_q = [0.057e-3; 0.057e-3; 0.122e-3; 0.114e-3; 0.122e-3; 0.172e-3].*pi/180; % Noise standard deviation
% robot.numericalParameters.sd_tau = [0.0371;0.1105;0.0611;0.0149;0.0153;0.0211];   % Noise standard deviation
% 
% % Physical Constraints:
% robot.physicalConstraints.limQ_U = [240;120;160;200;90;360].*pi/180;     % Joint position upper limit [rad] ([240;120;160;200;120;360].*pi/180;)
% robot.physicalConstraints.limQp_U = [225;150;275;412;450;720].*pi/180;      % Joint velocity upper limit [rad/s]
% robot.physicalConstraints.limQpp_U = 50*ones(robot.nbDOF,1);                % Joint acceleration upper limit [rad/s^2]
% robot.physicalConstraints.limTau_U = 10*[11.5;11.5;7.1;4.17;4.17;2.75];   	% Joint torque upper limit [N.m]
% robot.physicalConstraints.limQ_L = -[240;120;0;200;90;360].*pi/180;      % Joint position lower limit [rad] (-[240;120;0;200;120;360].*pi/180;)
% robot.physicalConstraints.limQp_L = -[225;150;275;412;450;720].*pi/180;     % Joint velocity lower limit [rad/s]
% robot.physicalConstraints.limQpp_L = -50*ones(robot.nbDOF,1);               % Joint acceleration lower limit [rad/s^2]
% robot.physicalConstraints.limTau_L = -10*[11.5;11.5;7.1;4.17;4.17;2.75]; 	% Joint torque lower limit [N.m]
% 
% robot.controlParameters.Gains_L = zeros(2*robot.nbDOF,1);
% robot.controlParameters.Gains_U = [1e5*ones(robot.nbDOF,1);1e5*ones(robot.nbDOF,1)];
% % 9.3729 ;   8.5911  ;  8.0857 ;   7.8392 ;   9.9767  ;  4.6818    0.4386 ;   0.2527 ;   0.6410  ;  0.0429  ;  0.2493   ; 0.0766
% robot.controlParameters.Kp = diag(1.0e+04 *[9.3729 ;   8.5911  ;  8.0857 ;   7.8392 ;   9.9767  ;  4.6818]);
% robot.controlParameters.Kd = diag(1.0e+04 *[0.4386 ;   0.2527 ;   0.6410  ;  0.0429  ;  0.2493   ; 0.0766]);
% end

