function [flag] = computeSymbolicDynamics(robot, options)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function allows to compute the robot kinematic, dynamic and
% identification models in symbolic form. 

disp(' ')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('%%%%%%%          GENERATING ROBOT SYMBOLIC EQUATIONS         %%%%%%%');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(' ')

%% Initialization:

tic

% Start parallel pool to speed-up the computations
poolobj = gcp('nocreate'); % If no pool, create new one.
if isempty(poolobj)
    parpool('local');
end

% Load Kinematic and Dynamic Parameters
disp('Initialize symbolic parameters')

if nargin < 3 % Just compute the models but do not generate optimized function handles.
    options.computeKinematics = true;
    options.computeDynamics = true;
    options.optimizeKinematics = true;
    options.optimizeDynamics = true;
    options.optimizeStateNoiseJacobians = false;
end

robotName = robot.name;
fprintf("Started parametric computation for the %s\n", robotName);

% Robot parameters:

Q = robot.symbolicParameters.Q;
Qp = robot.symbolicParameters.Qp;
Qpp = robot.symbolicParameters.Qpp;
Geometry = robot.symbolicParameters.Geometry;
GeometryCOM = robot.symbolicParameters.GeometryCOM;
Moment = robot.symbolicParameters.Moment;
InertiaCOM = robot.symbolicParameters.InertiaCOM;
Ia = robot.symbolicParameters.Ia;
Mass = robot.symbolicParameters.Mass;
Gravity = robot.symbolicParameters.Gravity;
Xhi= robot.symbolicParameters.Xhi;
Xhi_ef= robot.symbolicParameters.Xhi_ef;
Friction = robot.symbolicParameters.friction;
Z = robot.symbolicParameters.Z;
dt = robot.symbolicParameters.dt;

%% Robot Kinematics:

if options.computeKinematics == true
    
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp('Computing Homogeneous Transformations in symbolic form...')
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    
    % Homogeneous Transformations (symbolic form):
    % HT_a_b: homogeneous transformation of frame b with respect to frame a.
    
    HT_dhi_world = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of DH frame i w.r.t world frame
    HT_cmi_world = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t world frame
    HT_cmi_dhi = sym(zeros(4,4,robot.nbDOF));     % Homogeneous Transformation of Center of Mass i w.r.t DH frame i
    HT_cmi_cmi_1 = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t Center of Mass i-1
    HT_cmi_world_Moment = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t world frame expressed wrt moment standard parameter (MX, MY, MZ)
    HT_cmi_dhi_Moment = sym(zeros(4,4,robot.nbDOF));     % Homogeneous Transformation of Center of Mass i w.r.t DH frame i expressed wrt moment standard parameter (MX, MY, MZ)
    HT_cmi_cmi_1_Moment = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t Center of Mass i-1 expressed wrt moment standard parameter (MX, MY, MZ)
    HT_dhi_dhi_1 = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of DH frame i w.r.t DH frame i-1
    HT_base_world = robot.rootFrame.transform;    % Homogeneous Transformation of Robot base frame w.r.t world (static)
    
    for joint=1:robot.nbDOF
        HT_cmi_dhi(:,:,joint) = eye(4);
        HT_cmi_dhi(1:3,4,joint) = GeometryCOM(1:3, joint);
        HT_cmi_dhi_Moment(:,:,joint) = eye(4);
        HT_cmi_dhi_Moment(1:3,4,joint) = Moment(1:3, joint)./Mass(joint);
        HT_dhi_world(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot);
        HT_cmi_cmi_1(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot, true, HT_cmi_dhi(:,:,joint));
        HT_cmi_cmi_1_Moment(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot, true, HT_cmi_dhi_Moment(:,:,joint));
        HT_dhi_dhi_1(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot);
        HT_cmi_world(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot,true, HT_cmi_dhi(:,:,joint));
        HT_cmi_world_Moment(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot,true, HT_cmi_dhi_Moment(:,:,joint));
    end

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp('Computing Jacobian matrices in symbolic form...')
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    
    % Geometric Jacobian Matrices (symbolic form):
    % J_a_b: Geometric Jacobian Matrix of frame b with respect to frame a.
    
    J_dhi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    J_cmi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    J_cmi_world_Moment = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_dhi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_cmi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_cmi_world_Moment = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    
    parfor joint = 1:robot.nbDOF %#ok<PFUIX>
        
        % Link Jacobians:
        J_dhi_world(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world);
        fprintf('Jacobian link %d, successfully computed\n',joint);
        
        % Center of Mass Jacobians:
        J_cmi_world(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world, true, HT_cmi_world);
        J_cmi_world_Moment(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world, true, HT_cmi_world_Moment);
        fprintf('Jacobian Center of Mass %d, successfully computed\n',joint);
        
        % Time derivative of the Link Jacobians:
        Jd_dhi_world(:,:,joint) = timeDerivative(J_dhi_world(:,:,joint), Q, Qp);
        fprintf('Time derivative of Jacobian link %d, successfully computed\n',joint);
        
        % Time derivative of the Center of Mass Jacobians:
        Jd_cmi_world(:,:,joint) = timeDerivative(J_cmi_world(:,:,joint), Q, Qp);
        Jd_cmi_world_Moment(:,:,joint) = timeDerivative(J_cmi_world_Moment(:,:,joint), Q, Qp);
        fprintf('Time derivative of Jacobian Center of Mass %d, successfully computed\n',joint);
    end
    if options.optimizeKinematics == true
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        disp('Generating Optimized Robot Kinematics...')
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    end

    % Homogeneous Transformations:
    
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Homogeneous_Transforms',robotName),sprintf('HT_dh%d_world_%s.m',0,robotName)),'file') == 2)
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_world_%s.\n',robotName,0,robotName));
        matlabFunction(HT_base_world,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_world_%s',robotName,0,robotName), 'Vars', {Q, Geometry}, 'Outputs', {sprintf('HT%d', joint)});
    end

    parfor joint = 1:robot.nbDOF
        
        % Homogeneous Transformations:
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Homogeneous_Transforms',robotName),sprintf('HT_dh%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(HT_dhi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Geometry}, 'Outputs', {sprintf('HT%d', joint)});
        end
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Homogeneous_Transforms',robotName),sprintf('HT_cm%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_cm%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(HT_cmi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_cm%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Geometry, GeometryCOM}, 'Outputs', {sprintf('HTcm%d', joint)});
        end
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Homogeneous_Transforms',robotName),sprintf('HT_dh%d_dh%d_%s.m',joint,joint-1,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_dh%d_%s.\n',robotName,joint,joint-1,robotName);
            matlabFunction(HT_dhi_dhi_1(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_dh%d_dh%d_%s',robotName,joint,joint-1,robotName), 'Vars', {Q, Geometry}, 'Outputs', {sprintf('HT%d', joint)});
        end
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Homogeneous_Transforms',robotName),sprintf('HT_cm%d_cm%d_%s.m',joint,joint-1,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_cm%d_cm%d_%s.\n',robotName,joint,joint-1,robotName);
            matlabFunction(HT_cmi_cmi_1(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms/HT_cm%d_cm%d_%s',robotName,joint,joint-1,robotName), 'Vars', {Q, Geometry, GeometryCOM}, 'Outputs', {sprintf('HTcm%d', joint)});
        end
        
        % Jacobian Matrices:
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Jacobian_Matrices',robotName),sprintf('J_dh%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/J_dh%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(J_dhi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/J_dh%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Geometry}, 'Outputs', {sprintf('J%d', joint)});
        end
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Jacobian_Matrices',robotName),sprintf('J_cm%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/J_cm%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(J_cmi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/J_cm%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Geometry, GeometryCOM}, 'Outputs', {sprintf('Jcm%d', joint)});
        end
        
        % Time Derivative of Jacobian Matrices:
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Jacobian_Matrices',robotName),sprintf('Jd_dh%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/Jd_dh%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(Jd_dhi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/Jd_dh%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Qp, Geometry}, 'Outputs', {sprintf('Jd%d', joint)});
        end
        if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Jacobian_Matrices',robotName),sprintf('Jd_cm%d_world_%s.m',joint,robotName)),'file') == 2) || options.optimizeKinematics == true
            fprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/Jd_cm%d_world_%s.\n',robotName,joint,robotName);
            matlabFunction(Jd_cmi_world(:,:,joint),'File',sprintf('Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices/Jd_cm%d_world_%s',robotName,joint,robotName), 'Vars', {Q, Qp, Geometry, GeometryCOM}, 'Outputs', {sprintf('Jd%d', joint)});
        end
    end
end

if options.computeDynamics == true
    %% Robot Dynamics:
    
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp('Symbolic Matrix form of the Euler-Lagrange Dynamics Equations...')
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    
    options.algorithm = 'newton'; % 'newton' or 'lagrange'
    options.verif = false;
    [M_symb, C_symb, G_symb, kineticEnergy_symb, potentialEnergy_symb] = computeDynamicModel(robotName, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, Mass, InertiaCOM, Ia, Gravity, Q, Qp, options);

    % Friction model: 
    Tau_Friction_symb_aug = computeFrictionModel(robotName, Qp, Friction, Z, dt);
    
    % Inverse Dynamics:
    Tau_symb = M_symb*Qpp + C_symb*Qp + G_symb;  % Without friction

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp('Computation of the identification Model...')
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    
    options.method = 'baseParametersNum'; % 'oldMethod', 'blockTriangular' or 'baseParametersNum'.
    [Y_b, ~, Beta, ~, Xhi_d, Perm, K_d] = computeIdentificationModel(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, HT_dhi_dhi_1, robot, Tau_symb, Tau_Friction_symb_aug, options);
    [Y_M, Tau_h, Beta_symb] = reformulateRegressor(robot, Y_b, Beta);

    disp('Vector of base parameters: ') 
    disp(Beta)
    
    if robot.nbDOF >= 6
        delete(poolobj); % Avoid RAM memory saturation ! 
    end
    
    if options.optimizeDynamics == true
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        disp('Generating Optimized Robot Dynamics...')
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    end
    
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Inertia_M_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Inertia_M_%s \n',robotName,robotName));
        disp('This usually takes around 10 min for a 6-DOF robot: BE PATIENT !');
        matlabFunction(M_symb,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Inertia_M_%s',robotName,robotName), 'Vars', {Q, Geometry, Xhi}, 'Outputs', {'M'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('CorCen_C_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/CorCen_C_%s \n',robotName,robotName));
        disp('This usually takes more than one hour for a 6-DOF robot: BE PATIENT !');
        matlabFunction(C_symb,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/CorCen_C_%s',robotName,robotName), 'Vars', {Q, Qp, Geometry, Xhi}, 'Outputs', {'C'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Gravity_G_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Gravity_G_%s \n',robotName,robotName));
        disp('This usually takes a couple of seconds for a 6-DOF robot...');
        matlabFunction(G_symb,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Gravity_G_%s',robotName,robotName), 'Vars', {Q, Geometry, Xhi, Gravity}, 'Outputs', {'G'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Friction_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Friction_%s \n',robotName));
        disp('This usually takes a couple of seconds for a 6-DOF robot...');
        matlabFunction(Tau_Friction_symb_aug,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Friction_%s',robotName,robotName), 'Vars', {Qp, Friction.Fv, Friction.Fc, Friction.Fvm, Friction.Fcm, Friction.Fs, Friction.Vs, Friction.Es, Friction.Sigma_0, Friction.Sigma_1, Friction.Sigma_2, Friction.Tau_off, Z, dt}, 'Outputs', {'F'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Y_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Y_%s \n',robotName,robotName));
        disp('This usually takes more than one hour for a 6-DOF robot: BE PATIENT !');
        matlabFunction(Y_b,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Y_%s',robotName,robotName), 'Vars', {Q, Qp, Qpp, Geometry, Gravity}, 'Outputs', {'Y'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Beta_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Beta_%s \n',robotName,robotName));
        disp('This usually takes a couple of seconds for a 6-DOF robot...');
        matlabFunction(Beta,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Beta_%s',robotName,robotName), 'Vars', {Xhi}, 'Outputs', {'Beta'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Xhi_d_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Xhi_d_%s \n',robotName,robotName));
        disp('This usually takes a couple of seconds for a 6-DOF robot...');
        matlabFunction(Xhi_d,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Xhi_d_%s',robotName,robotName), 'Vars', {Xhi}, 'Outputs', {'Xhi_d'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Perm_%s.mat',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Saving expression in Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Perm_%s \n',robotName,robotName));
        save(sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Perm_%s.mat',robotName,robotName),'Perm');
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_K_d_%s.mat',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Saving expression in Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_K_d_%s \n',robotName,robotName));
        save(sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_K_d_%s.mat',robotName,robotName),'K_d');
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('CorCen_h_Beta_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/CorCen_h_Beta_%s \n',robotName,robotName));
        disp('This usually takes more than one hour for a 6-DOF robot: BE PATIENT !');
        matlabFunction(Tau_h,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/CorCen_h_Beta_%s',robotName,robotName), 'Vars', {Q, Qp, Geometry, Gravity, Beta_symb}, 'Outputs', {'h'});
    end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Inertia_M_Beta_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Inertia_M_Beta_%s \n',robotName,robotName));
        disp('This usually takes around 10 min for a 6-DOF robot: BE PATIENT !');
        matlabFunction(Y_M,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Inertia_M_Beta_%s',robotName,robotName), 'Vars', {Q, Geometry, Beta_symb}, 'Outputs', {'M'});
    end
    
    % Computing the regressor of a mass attached to the end-effector of the robot for drive gains identifications purpose, following the work of Gautier and Briot:
    % "Global Identification of Drive-Gains Parameters of Robots Using a Known Payload" (2011). 
    
    [Y_ef,Beta_ef] = computeEndEffectorMassRegressionModel(robot);
    
     if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Y_ef_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Y_ef_%s \n',robotName,robotName));
        disp('This usually takes more than one hour for a 6-DOF robot: BE PATIENT !');
        matlabFunction(Y_ef,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Y_ef_%s',robotName,robotName), 'Vars', {Q, Qp, Qpp, Geometry, Gravity}, 'Outputs', {'Y'});
     end
    if ~(exist(fullfile('Benchmark',sprintf('Robot_Generated_Data/%s/Dynamic_Model',robotName),sprintf('Regressor_Beta_ef_%s.m',robotName)),'file') == 2) || options.optimizeDynamics == true
        fprintf(sprintf('Generating optimized code for Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Beta_ef_%s \n',robotName,robotName));
        disp('This usually takes a couple of seconds for a 6-DOF robot...');
        matlabFunction(Beta_ef,'File',sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Beta_ef_%s',robotName,robotName), 'Vars', {Xhi_ef}, 'Outputs', {'Beta'});
    end
    
end

computationTime = toc;
fprintf("Computation time = %d minutes, %d seconds\n", floor(computationTime/60), ceil(rem(computationTime,60)));
flag = true;

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('DONE: stopping symbolic kinematics and dynamic computation...')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
end
