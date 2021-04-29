function [t_sample, State_sampled, Qpp_sampled] = integrateClosedLoopNoisyDynamics(augmentedDesiredState, Beta, robotName, Geometry, Gravity, t_i, t_f, nbCtrlSamples, nbSamples, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, integrationAlgorithm, sd_q) %#codegen

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Integrate the closed-loop robot noisy dynamics between epochs t_i and t_f using Runge-Kutta 1, 2 or 4 steps. 

nbDOF = size(Geometry,1);
t_ctrl = linspace(t_i, t_f, nbCtrlSamples);  % Control epochs
dt_ctrl = (t_f-t_i)/nbCtrlSamples;
t_sample = linspace(t_i, t_f, nbSamples); % Sampling epochs
sampleIndex = 2;

currentState = augmentedDesiredState(nbDOF+1:end,1); % X = [Qp;Q]
measuredState = currentState;
State_sampled = zeros(2*nbDOF, nbSamples); 
State_sampled(:,1) = augmentedDesiredState(nbDOF+1:end,1);
Qpp_sampled = zeros(nbDOF, nbSamples);
Qpp_sampled(:,1) = augmentedDesiredState(1:nbDOF,1);
integralError = zeros(nbDOF,1); % integral of error in position

forwardDynamics = @(State, controlInput) computeForwardDynamics(robotName, State, controlInput, Beta, Geometry, Gravity);

for controlIndex = 2:nbCtrlSamples
    [controlInput, integralError] = computeControlInput(dt_ctrl, measuredState, augmentedDesiredState(:,controlIndex), integralError, Kp, Ki, Kd, Ktau,...
        antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U);
    
    timeDerivativeState = forwardDynamics(currentState, controlInput);
    
    switch integrationAlgorithm
	case 'rk1'
	    currentState = rk1_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
	case 'rk2'
	    currentState = rk2_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
	case 'rk4'
	    currentState = rk4_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
	case 'ode45'
	    [~ , Y_i] = ode45(@(t, y) forwardDynamics(y, controlInput), [t_ctrl(controlIndex-1) t_ctrl(controlIndex)], currentState);
        currentState = Y_i(end,:)';
	otherwise % 'rk1'
	    currentState = rk1_IntegrationStep(forwardDynamics,currentState,controlInput,dt_ctrl);
    end
    
    % Real robot state:
    currentState(1:nbDOF) = min(limQp_U,max(limQp_L,currentState(1:nbDOF))); % Velocity limit saturation
    currentState(nbDOF+1:end) = min(limQ_U,max(limQ_L,currentState(nbDOF+1:end))); % Joint limit saturation
    
    Qm = noisyMeasurement(currentState(nbDOF+1:end),sd_q);
    
    % Measured robot state, reinjected into the control loop:                                     
    measuredState = [discreteTimeDerivative(Qm,measuredState(nbDOF+1:end),dt_ctrl); Qm]; % Discrete time derivation to get the velocity from the noisy position (as it is usually done in real digital control systems) 

    % Store variables in memory at each sampling epoch:
    
    if t_ctrl(controlIndex) >= t_sample(sampleIndex)
        Qpp_sampled(:,sampleIndex) = timeDerivativeState(1:nbDOF);
        State_sampled(1:nbDOF,sampleIndex) = timeDerivativeState(nbDOF+1:end);
        State_sampled(nbDOF+1:end,sampleIndex) = currentState(nbDOF+1:end);
        sampleIndex = sampleIndex+1;
        if sampleIndex > numel(t_sample)
            sampleIndex = numel(t_sample);
        end
    end
end
end
