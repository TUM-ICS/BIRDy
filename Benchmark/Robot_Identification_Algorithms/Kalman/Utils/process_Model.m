function [x] = process_Model(t_km1, t_k, x_km1, u_km1, q_km1, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

dt = t_k - t_km1;

% Here x_km1 refers to the augmented state [Qp;Q;Beta]:
[nbDOF, ~] = size(Geometry); 
Beta = x_km1(2*nbDOF+1:end);

% Forward dynamic model of the robot:
forwardDynamics = @(State, controlInput) computeForwardDynamics(robotName, State, controlInput, Beta, Geometry, Gravity);

if useComputedTorque == true % Use the computed torque as a control input:
    
    nbCtrlSamples=size(Xd,2);
    x = x_km1;
    integralError = zeros(nbDOF,1); % integral of error in position

    for i = 2:nbCtrlSamples
        
        [controlInput, integralError] = computeControlInput(dt_control, x(1:2*nbDOF), Xd(:,i), integralError, Kp, Ki, Kd, Ktau,...
            antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U);
        
        switch integrationAlgorithm
            case 'rk1'
                x = [rk1_IntegrationStep(forwardDynamics,x(1:2*nbDOF),controlInput,dt_control);Beta];
            case 'rk2'
                x = [rk2_IntegrationStep(forwardDynamics,x(1:2*nbDOF),controlInput,dt_control);Beta];
            case 'rk4'
                x = [rk4_IntegrationStep(forwardDynamics,x(1:2*nbDOF),controlInput,dt_control);Beta];
            case 'ode45'
                [~ , Y_i] = ode45(@(t, y) forwardDynamics(y, controlInput), [t_km1+(i-1)*dt_control t_km1+i*dt_control], x(1:2*nbDOF));
                x = [Y_i(end,:)';Beta];
            otherwise % 'rk1'
                x = [rk1_IntegrationStep(forwardDynamics,x(1:2*nbDOF),controlInput,dt_control);Beta];
        end
    end
    
else % Use the measured torque as a control input:
    
    switch integrationAlgorithm
        case 'rk1'
            x = [rk1_IntegrationStep(forwardDynamics,x_km1(1:2*nbDOF),u_km1,dt);Beta];
        case 'rk2'
            x = [rk2_IntegrationStep(forwardDynamics,x_km1(1:2*nbDOF),u_km1,dt);Beta];
        case 'rk4'
            x = [rk4_IntegrationStep(forwardDynamics,x_km1(1:2*nbDOF),u_km1,dt);Beta];
        case 'ode45'
            [~ , Y_i] = ode45(@(t, y) forwardDynamics(y, u_km1), [t_km1 t_k], x_km1(1:2*nbDOF));
            x = [Y_i(end,:)';Beta];
        otherwise % 'rk1'
            x = [rk1_IntegrationStep(forwardDynamics,x_km1(1:2*nbDOF),u_km1,dt);Beta];
    end
    
end

x = x + q_km1;

end