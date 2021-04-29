function [experimentDataStruct] = generateExperimentData(robot, benchmarkSettings, options, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Experiment data generation

if benchmarkSettings.experimentOnTrueRobot == true
    
    error('TODO: PLEASE REWRITE THIS PART OF THE CODE');
    
    % Load experiment file:
    fileName = 'mitsu-data_1_100_05';
    experimentFile = sprintf('Benchmark/Robot_Generated_Data/%s/Experiment_Data/%s.mat', benchmarkSettings.robotName, fileName);
    load(experimentFile);
    
    nbSamples = size(experimentData.Qm,2);
    t_sample = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, nbSamples); % Sampling epochs
    
    if benchmarkSettings.trajectoryData.t_f < benchmarkSettings.t_f
        error('Experiment data parsing: the selected time horizon (t_f = %d) is greater than that of the trajectory generator (t_f = %d)!\n', benchmarkSettings.t_f, benchmarkSettings.trajectoryData.t_f);
    end
    
    Taum = zeros(robot.nbDOF, nbSamples, 1);
    Qm = zeros(robot.nbDOF, nbSamples, 1);
    Qpm = zeros(robot.nbDOF, nbSamples, 1);
    Qppm = zeros(robot.nbDOF, nbSamples, 1);
    Xm = zeros(3, nbSamples, 1);
    Xpm = zeros(3, nbSamples, 1);
    Qd = zeros(robot.nbDOF, nbSamples, 1);
    Qpd = zeros(robot.nbDOF, nbSamples, 1);
    Qppd = zeros(robot.nbDOF, nbSamples, 1);
    Xd = zeros(3, nbSamples, 1);
    Xpd = zeros(3, nbSamples, 1);
    
    %% Data Parsing:
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_sample, benchmarkSettings.interpolationAlgorithm); % Initial state [Qp;Q]
    [Qd(:,:,1), Qpd(:,:,1), Qppd(:,:,1), Xd(:,:,1), Qm(:,:,1), Xm(:,:,1), Taum(:,:,1)] = parseExperimentData(robot, t_sample, augmentedDesiredState, experimentData, options, varargin);
    
    experimentDataStruct.time = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples);
    experimentDataStruct.Taum = Taum;
    experimentDataStruct.Qm = Qm;
    experimentDataStruct.Qpm = Qpm;
    experimentDataStruct.Qppm = Qppm;
    experimentDataStruct.Xm = Xm;
    experimentDataStruct.Xpm = Xpm;
    experimentDataStruct.Qd = Qd;
    experimentDataStruct.Qpd = Qpd;
    experimentDataStruct.Qppd = Qppd;
    experimentDataStruct.Xd = Xd;
    experimentDataStruct.Xpd = Xpd;
else
    numberOfExperimentsPerInitialPoint = benchmarkSettings.numberOfExperimentsPerInitialPoint;
    Xhi_obj = benchmarkSettings.Xhi_obj;
    nbSamples = benchmarkSettings.nbSamples;
    nbCtrlSamples = benchmarkSettings.nbCtrlSamples;
    t_sample = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, nbSamples); % Sampling epochs
    t_ctrl = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, nbCtrlSamples); % Control epochs
    if benchmarkSettings.trajectoryData.t_f < benchmarkSettings.t_f
        error('Experiment data generation: the desired time horizon of the simulation (t_f = %d) is greater than that of the trajectory generator (t_f = %d)!\n', benchmarkSettings.t_f, benchmarkSettings.trajectoryData.t_f);
    end
    
    sd_q = robot.numericalParameters.sd_q;       % Noise standard deviation
    sd_tau = robot.numericalParameters.sd_tau;     % Noise standard deviation
    Tau = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Tau_friction = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Taum = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Q = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qp = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qpp = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    X = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    Xp = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    Qm = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qpm = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qppm = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Xm = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    Xpm = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    Qd = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qpd = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Qppd = zeros(robot.nbDOF, nbSamples, numberOfExperimentsPerInitialPoint);
    Xd = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    Xpd = zeros(3, nbSamples, numberOfExperimentsPerInitialPoint);
    %% Data Generation:
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_ctrl, benchmarkSettings.interpolationAlgorithm); % Initial state [Qp;Q]
    if numberOfExperimentsPerInitialPoint == 1
        for j = 1:numberOfExperimentsPerInitialPoint
            [Q(:,:,j), Qp(:,:,j), Qpp(:,:,j), X(:,:,j), Xp(:,:,j), Tau(:,:,j), Tau_friction(:,:,j), ...
                Qd(:,:,j), Qpd(:,:,j), Qppd(:,:,j), Xd(:,:,j), Xpd(:,:,j), ...
                Qm(:,:,j), Qpm(:,:,j), Qppm(:,:,j), Xm(:,:,j), Xpm(:,:,j), Taum(:,:,j)] = simulateRobot(augmentedDesiredState, t_sample, benchmarkSettings.dt, t_ctrl, benchmarkSettings.dt_ctrl, Xhi_obj, nbSamples, nbCtrlSamples, robot, sd_q, sd_tau, options, j);
        end
    else
        parfor j = 1:numberOfExperimentsPerInitialPoint
            [Q(:,:,j), Qp(:,:,j), Qpp(:,:,j), X(:,:,j), Xp(:,:,j), Tau(:,:,j), Tau_friction(:,:,j), ...
                Qd(:,:,j), Qpd(:,:,j), Qppd(:,:,j), Xd(:,:,j), Xpd(:,:,j), ...
                Qm(:,:,j), Qpm(:,:,j), Qppm(:,:,j), Xm(:,:,j), Xpm(:,:,j), Taum(:,:,j)] = simulateRobot(augmentedDesiredState, t_sample, benchmarkSettings.dt, t_ctrl, benchmarkSettings.dt_ctrl, Xhi_obj, nbSamples, nbCtrlSamples, robot, sd_q, sd_tau, options, j);
        end
    end
    
    experimentDataStruct.time = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples);
    experimentDataStruct.Tau_friction = Tau_friction;
    experimentDataStruct.Tau = Tau;
    experimentDataStruct.Taum = Taum;
    experimentDataStruct.Q = Q;
    experimentDataStruct.Qp = Qp;
    experimentDataStruct.Qpp = Qpp;
    experimentDataStruct.X = X;
    experimentDataStruct.Xp = Xp;
    experimentDataStruct.Qm = Qm;
    experimentDataStruct.Qpm = Qpm;
    experimentDataStruct.Qppm = Qppm;
    experimentDataStruct.Xm = Xm;
    experimentDataStruct.Xpm = Xpm;
    experimentDataStruct.Qd = Qd;
    experimentDataStruct.Qpd = Qpd;
    experimentDataStruct.Qppd = Qppd;
    experimentDataStruct.Xd = Xd;
    experimentDataStruct.Xpd = Xpd;
end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Closed-Loop Simulation Routine:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Q, Qp, Qpp, X, Xp, Tau, Tau_friction, Qd, Qpd, Qppd, Xd, Xpd, Qm, Qpm, Qppm, Xm, Xpm, Taum] = simulateRobot(augmentedDesiredState, t_sample, dt_sample, t_ctrl, dt_ctrl, Xhi_obj, nbSamples, nbCtrlSamples, robot, sd_q, sd_tau, options, index, varargin)

% This function runs at the robot controller frequency
% robot.controlParameters.controlFrequency and collect experimental data at
% the sampling frequeny robot.controlParameters.samplingFrequency

% Variables memory initialization:

Tau = zeros(robot.nbDOF, nbSamples);
Tau_friction = zeros(robot.nbDOF, nbSamples);
Taum = zeros(robot.nbDOF, nbSamples);
Q = zeros(robot.nbDOF, nbSamples);
Qp = zeros(robot.nbDOF, nbSamples);
Qpp = zeros(robot.nbDOF, nbSamples);
X = zeros(3, nbSamples);
Xp = zeros(3, nbSamples);
Qm = zeros(robot.nbDOF, nbSamples);
Qpm = zeros(robot.nbDOF, nbSamples);
Qppm = zeros(robot.nbDOF, nbSamples);
Xm = zeros(3, nbSamples);
Xpm = zeros(3, nbSamples);
Qd = zeros(robot.nbDOF, nbSamples);
Qpd = zeros(robot.nbDOF, nbSamples);
Qppd = zeros(robot.nbDOF, nbSamples);
Xd = zeros(3, nbSamples);
Xpd = zeros(3, nbSamples);

% Variables initial value:
Qpp(:,1) = augmentedDesiredState(1:robot.nbDOF,1);
Qp(:,1) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,1);
Q(:,1) = augmentedDesiredState(2*robot.nbDOF+1:end,1);
Qppm(:,1) = augmentedDesiredState(1:robot.nbDOF,1);
Qpm(:,1) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,1);
Qm(:,1) = augmentedDesiredState(2*robot.nbDOF+1:end,1);
Qppd(:,1) = augmentedDesiredState(1:robot.nbDOF,1);
Qpd(:,1) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,1);
Qd(:,1) = augmentedDesiredState(2*robot.nbDOF+1:end,1);
measuredState = [Qpm(:,1); Qm(:,1)];
currentState = augmentedDesiredState(robot.nbDOF+1:end,1);
integralError = zeros(robot.nbDOF, 1);

H = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Q(:,1), robot.numericalParameters.Geometry);
X(:,1) = H(1:3,4);
Xp(:,1) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Q(:,1), robot.numericalParameters.Geometry)*Q(:,1);
Hd = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qd(:,1), robot.numericalParameters.Geometry);
Xd(:,1) = Hd(1:3,4);
Xpd(:,1) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Qd(:,1), robot.numericalParameters.Geometry)*Qd(:,1);
Hm = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qm(:,1), robot.numericalParameters.Geometry);
Xm(:,1) = Hm(1:3,4);
Xpm(:,1) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Qm(:,1), robot.numericalParameters.Geometry)*Qm(:,1);
Z = robot.numericalParameters.friction.Z0;
[friction, Z] = generateFriction(Qp(:,1), Z, robot, dt_ctrl);
Tau_friction(:,1) = friction;
sampleIndex = 2;
options.augmentedInitialState=augmentedDesiredState(:,1);
options.index = index;

%% Closed loop dynamics:
for controlIndex = 2:nbCtrlSamples % Runs at controlFrequency and collect data at samplingFrequency
    if ~mod(controlIndex,1000)
        fprintf('Iteration %d of %d, experiment %d\n', controlIndex, nbCtrlSamples, index);
    end
    % Generate contol input:
    [controlInput, integralError] = computeControlInput(dt_ctrl, measuredState, augmentedDesiredState(:,controlIndex), ...
        integralError, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
        robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
        robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, ...
        robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U); % Control input generated with noisy measured state
    
    % Propagate dynamics:
    timeDerivativeState = computeDataGenForwardDynamics(robot.name, currentState, controlInput, Xhi_obj, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, friction); % Real robot state derivative (no added noise) when submitted to 'Tau'
    
    % Time integration between t(i-1) and t(i):
    forwardDynamics = @(State, controlInput) computeDataGenForwardDynamics(robot.name, State, controlInput, Xhi_obj, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, friction);
    
    switch options.integrationAlgorithm
            case 'rk1'
                currentState = rk1_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
            case 'rk2'
                currentState = rk2_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
            case 'rk4'
                currentState = rk4_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
            case 'ode45'
                [~ , Y_i] = ode45(@(t, Y) computeDataGenForwardDynamics(robot.name, Y, controlInput, Xhi_obj, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, friction), [t_ctrl(controlIndex-1) t_ctrl(controlIndex)], currentState);
                currentState = Y_i(end,:).';
            otherwise % 'rk1'
                currentState = rk1_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
    end

    % Real robot state:
    currentState(1:robot.nbDOF) = min(robot.physicalConstraints.limQp_U,max(robot.physicalConstraints.limQp_L,currentState(1:robot.nbDOF))); % Velocity limit saturation
    currentState(robot.nbDOF+1:end) = min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,currentState(robot.nbDOF+1:end))); % Joint limit saturation
    
    % Measured robot state, reinjected into the control loop:
    measured_Q  = noisyMeasurement(currentState(robot.nbDOF+1:end),sd_q);                                      % Noise applied on position measurement (encoder noise)     
    measuredState = [discreteTimeDerivative(measured_Q,measuredState(robot.nbDOF+1:end),dt_ctrl); measured_Q]; % Discrete time derivation to get the velocity from the noisy position (as it is usually done in real digital control systems) 
    
    % Compute friction:
    [friction, Z] = generateFriction(currentState(1:robot.nbDOF), Z, robot, dt_ctrl);
    
    % Store variables in memory at each sampling epoch:
    if t_ctrl(controlIndex) >= t_sample(sampleIndex)
        
        Q(:,sampleIndex) = currentState(robot.nbDOF+1:end);                                               % Real joint position
        Qp(:,sampleIndex) = timeDerivativeState(robot.nbDOF+1:end);                                       % Real joint velocity
        Qpp(:,sampleIndex) = timeDerivativeState(1:robot.nbDOF);                                          % Real joint acceleration
        Qd(:,sampleIndex) = augmentedDesiredState(2*robot.nbDOF+1:end,controlIndex);                                 % Desired joint position
        Qpd(:,sampleIndex) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,controlIndex);                        % Desired joint velocity
        Qppd(:,sampleIndex) = augmentedDesiredState(1:robot.nbDOF,controlIndex);                                     % Desired joint acceleration
        Qm(:,sampleIndex) = measuredState(robot.nbDOF+1:end);                                             % Joint position measured with noisy sensor (re-injected in position control loop)
        Qpm(:,sampleIndex) = measuredState(1:robot.nbDOF);                                                % Discrete time derivative of noisy joint position measurement (re-injected in position control loop)
        Qppm(:,sampleIndex) = discreteTimeDerivative(Qpm(:,sampleIndex),Qpm(:,sampleIndex-1),dt_sample);  % Double discrete time derivative of noisy joint position measurement
        Tau(:,sampleIndex) = controlInput;                                                              % Real control Torque sent to the robot (without added sensor noise but computed using noisy position measurement)
        Taum(:,sampleIndex) = noisyMeasurement(controlInput, sd_tau);                                   % Control Torque measured with noisy sensor (not re-injected in position control but visible in measured identification data)
        Tau_friction(:,sampleIndex) = friction;
        H = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Q(:,sampleIndex), robot.numericalParameters.Geometry);
        X(:,sampleIndex) = H(1:3,4);                                                                      % Real Cartesian Position
        Xp(:,sampleIndex) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Q(:,sampleIndex), robot.numericalParameters.Geometry)*Qp(:,sampleIndex);   % Real Cartesian Velocity
        Hd = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qd(:,sampleIndex), robot.numericalParameters.Geometry);
        Xd(:,sampleIndex) = Hd(1:3,4);                                                                    % Desired Cartesian Position
        Xpd(:,sampleIndex) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Qd(:,sampleIndex), robot.numericalParameters.Geometry)*Qpd(:,sampleIndex);% Desired Cartesian Velocity
        Hm = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qm(:,sampleIndex), robot.numericalParameters.Geometry);
        Xm(:,sampleIndex) = Hm(1:3,4);                                                                    % Measured Cartesian Position (with noisy sensor)
        Xpm(:,sampleIndex) = [eye(3),zeros(3)]*feval(sprintf('J_dh%d_world_%s', robot.nbDOF,robot.name),Qm(:,sampleIndex), robot.numericalParameters.Geometry)*Qpm(:,sampleIndex);% Measured Cartesian Velocity (with noisy sensor)
        
        sampleIndex = sampleIndex+1;
        if sampleIndex > numel(t_sample)
            sampleIndex = numel(t_sample);
        end
        
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Friction Data Simulation Routine:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Tau_friction, Z] = generateFriction(Qp, Z, robot, dt)

Friction = feval(sprintf('Friction_%s', robot.name), Qp, robot.numericalParameters.friction.Fv, robot.numericalParameters.friction.Fc, robot.numericalParameters.friction.Fvm, robot.numericalParameters.friction.Fcm, robot.numericalParameters.friction.Fs, robot.numericalParameters.friction.Vs, robot.numericalParameters.friction.Es, robot.numericalParameters.friction.Sigma_0, robot.numericalParameters.friction.Sigma_1, robot.numericalParameters.friction.Sigma_2, robot.numericalParameters.friction.Tau_off, Z, dt);

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

Z = Friction(:,8);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Real Data Extraction Routine:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Qd, Qpd, Qppd, Xd, Qm, Xm, Taum] = parseExperimentData(robot, t_sample, augmentedDesiredState, experimentData, options, varargin)

% This function parse the file containing the results of the experiments performed on the
% real robot.

% Variables memory initialization:
nbSamples = numel(t_sample);
Xm = zeros(3, nbSamples);
Xd = zeros(3, nbSamples);

Qd = augmentedDesiredState(2*robot.nbDOF+1:end,:);                                 % Desired joint position
Qpd = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,:);                        % Desired joint velocity
Qppd = augmentedDesiredState(1:robot.nbDOF,:);                                     % Desired joint acceleration
Qm = experimentData.Qm;                                             % Joint position measured with noisy sensor (re-injected in position control loop)
Taum = experimentData.Taum;                                   % Control Torque measured with noisy sensor (not re-injected in position control but visible in measured identification data)

% Store variables in memory at each sampling epoch:
for sampleIndex = 1:nbSamples
    Hd = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qd(:,sampleIndex), robot.numericalParameters.Geometry);
    Xd(:,sampleIndex) = Hd(1:3,4);                                                                    % Desired Cartesian Position
    Hm = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Qm(:,sampleIndex), robot.numericalParameters.Geometry);
    Xm(:,sampleIndex) = Hm(1:3,4);                                                                    % Measured Cartesian Position (with noisy sensor)
end

end

