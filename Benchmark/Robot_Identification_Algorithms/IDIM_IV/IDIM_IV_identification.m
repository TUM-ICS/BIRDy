function [Beta_IV, it, Betas, flag]=IDIM_IV_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Xhi_0, Beta_0, optionsIDIM_IV)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Identify the dynamics parameters of the robot using the Instrumental Variables method:

%% Data Decimation and Filtering:

[~, ~, Qpp, Qp, Q, ~, Tau_decim, ~, ~, ~, optionsIDIM_IV] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_IV, expNb, 'IDIM_IV');

%% Populating the Instrument Matrix X and Torque Vector Y_tau:

if strcmp(benchmarkSettings.codeImplementation,'optim')
    % Compute the sampled vector:
    Y_tau = torqueVector_mex(Tau_decim);
    % Compute the observation matrix:
    W_ndec = observationMatrix_mex(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);
elseif strcmp(benchmarkSettings.codeImplementation,'classic')
    % Compute the sampled vector:
    Y_tau = torqueVector(Tau_decim);
    % Compute the observation matrix:
    W_ndec = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);
else
    error("IDIM_IV: unknown option");
end

% Decimation filter
W=zeros(robot.nbDOF*optionsIDIM_IV.nbSampleDecim, robot.paramVectorSize); % Observation matrix
for col=1:robot.paramVectorSize
    for i = 1:robot.nbDOF
        W(i:robot.nbDOF:end,col) = decimate(W_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
    end
end

%% Starting Conditions:

Z=zeros(robot.nbDOF*optionsIDIM_IV.nbSampleDecim, robot.paramVectorSize); % Instrument matrix
t_ctrl = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_ctrl, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
Beta = Beta_0;
eps_curr_it = 1;
Betas = zeros(robot.paramVectorSize, optionsIDIM_IV.stopCrit.Max_it+1);
crit_1=1;
crit_2=1;
thresholdOmega = 1e-2;
thresholdWeight = 1;

%% IV algorithm:

it = 1;
Betas(:,it) = Beta;

while (( (crit_1 >= optionsIDIM_IV.stopCrit.tol_1) && (crit_2 >= optionsIDIM_IV.stopCrit.tol_2 ) ) && (it<optionsIDIM_IV.stopCrit.Max_it))
    
    fprintf('Iteration IDIM-IV %d \n', it);
    
    % Simulation of the robot:
    if strcmp(benchmarkSettings.codeImplementation,'optim')
        % Simulation of the robot using the current parameters:
        [~, State_it, Qpp_itt] = integrateClosedLoopDynamics_mex(augmentedDesiredState, Beta, robot.name, robot.numericalParameters.Geometry, ...
            robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
            robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
            robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
            robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    elseif strcmp(benchmarkSettings.codeImplementation,'classic')
        % Simulation of the robot using the current parameters:
        [~, State_it, Qpp_itt] = integrateClosedLoopDynamics(augmentedDesiredState, Beta, robot.name, robot.numericalParameters.Geometry, ...
            robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
            robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
            robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
            robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    else
        error("IDIM_IV: unknown implementation option");
    end

    Qp_it = State_it(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    Q_it = State_it(robot.nbDOF+1:end,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    Qpp_it = Qpp_itt(:,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    
    if strcmp(benchmarkSettings.codeImplementation,'optim')
        % Building of the observation matrix W for the current iteration of Beta:
        Z_ndec = observationMatrix_mex(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_it, Qp_it, Qpp_it);
    elseif strcmp(benchmarkSettings.codeImplementation,'classic')
        % Building of the observation matrix W for the current iteration of Beta:
        Z_ndec = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_it, Qp_it, Qpp_it);
    else
        error("IDIM_IV: unknown option");
    end
    
    % Decimation filter
    for col=1:robot.paramVectorSize
        for i = 1:robot.nbDOF
            Z(i:robot.nbDOF:end,col) = decimate(Z_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
        end
    end
    
    switch optionsIDIM_IV.alg
        
        case 'OLS'
            
            Beta_next_it = LeastSquares(robot, W, Y_tau, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, false, Xhi_0, Beta_0, Z);
            
        case 'WLS'
            
            [sig2_error] = getErrorCovariance(W, Y_tau, robot.nbDOF);
            Omega_inv_sqrt = diag(sqrt(1./sig2_error));
            if strcmp(benchmarkSettings.codeImplementation,'optim')
                [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
            elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
            else
                error("IDIM_IV: unknown option");
            end
            
            Beta_next_it = LeastSquares(robot, W_star, Y_tau_star, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, false, Xhi_0, Beta_0, Z);
            
        case 'IRLS' % Iteratively Reweighted Least Squares
            
            weightsConverged = false;
            omegaConverged = false;
            
            weightVector = ones(optionsIDIM_IV.nbSampleDecim*robot.nbDOF,1);
            Omega = eye(robot.nbDOF); % Initially, Omega is an identity matrix. As iterations go on within this loop, Omega converges to a constant matrix whose diagonal elements denote the estimated variance of the corresponding joint/channel.
            
            while weightsConverged == false
                
                while omegaConverged == false
                    [oU,oS,oV] = svd(Omega);
                    Omega_inv_sqrt = oU*sqrt(inv(oS))*oV.';
                    if strcmp(benchmarkSettings.codeImplementation,'optim')
                        [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
                    elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                        [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
                    else
                        error("IDIM_IV: unknown option");
                    end
                    
                    W_hash = repmat(weightVector,1,robot.paramVectorSize).*W_star;
                    Y_tau_hash = weightVector.*Y_tau_star;
                    
                    Beta_next_it = LeastSquares(robot, W_hash, Y_tau_hash, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, false, Xhi_0, Beta_0, Z);
                    
                    % Update Omega
                    Omega_old = Omega;
                    Omega = updateErrorFullCovariance(W_hash, Y_tau_hash, Beta_next_it, inv(Omega_inv_sqrt), robot.nbDOF);
                    
                    % Does Omega converge ?
                    deltaOmega = norm(Omega - Omega_old,'fro');
                    if deltaOmega<thresholdOmega
                        omegaConverged = true;
                    else
                        omegaConverged = false;
                    end
                end
                
                % Update weightVector
                weightVector_old = weightVector;
                weightVector = min(weightVector,weightFunction(Y_tau_hash - W_hash*Beta_next_it, 3));
                
                % Does weightVector converge ?
                deltaWeight = norm(weightVector - weightVector_old);
                if deltaWeight<thresholdWeight
                    weightsConverged = true;
                else
                    weightsConverged = false;
                end
                
            end
            
        case 'PC-OLS'
            
            Beta_next_it = LeastSquares(robot, W, Y_tau, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, true, Xhi_0, Beta_0, Z);
            
        case 'PC-WLS'
            
            [sig2_error] = getErrorCovariance(W, Y_tau, robot.nbDOF);
            Omega_inv_sqrt = diag(sqrt(1./sig2_error));
            if strcmp(benchmarkSettings.codeImplementation,'optim')
                [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
            elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
            else
                error("IDIM_IV: unknown option");
            end
            
            Beta_next_it = LeastSquares(robot, W_star, Y_tau_star, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, true, Xhi_0, Beta_0, Z);
            
        case 'PC-IRLS'
            
            weightsConverged = false;
            omegaConverged = false;
            
            weightVector = ones(optionsIDIM_IV.nbSampleDecim*robot.nbDOF,1);
            Omega = eye(robot.nbDOF); % Initially, Omega is an identity matrix. As iterations go on within this loop, Omega converges to a constant matrix whose diagonal elements denote the estimated variance of the corresponding joint/channel.
            
            while weightsConverged == false
                
                while omegaConverged == false
                    [oU,oS,oV] = svd(Omega);
                    Omega_inv_sqrt = oU*sqrt(inv(oS))*oV.';
                    if strcmp(benchmarkSettings.codeImplementation,'optim')
                        [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
                    elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                        [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_IV.nbSampleDecim, robot.nbDOF);
                    else
                        error("IDIM_IV: unknown option");
                    end
                    
                    W_hash = repmat(weightVector,1,robot.paramVectorSize).*W_star;
                    Y_tau_hash = weightVector.*Y_tau_star;
                    
                    Beta_next_it = LeastSquares(robot, W_hash, Y_tau_hash, optionsIDIM_IV.solver, optionsIDIM_IV.regularizerType, optionsIDIM_IV.gammaReg, true, Xhi_0, Beta_0, Z);
                    
                    % Update Omega
                    Omega_old = Omega;
                    Omega = updateErrorFullCovariance(W_hash, Y_tau_hash, Beta_next_it, inv(Omega_inv_sqrt), robot.nbDOF);
                    
                    % Does Omega converge ?
                    deltaOmega = norm(Omega - Omega_old,'fro');
                    if deltaOmega<thresholdOmega
                        omegaConverged = true;
                    else
                        omegaConverged = false;
                    end
                end
                
                % Update weightVector
                weightVector_old = weightVector;
                weightVector = min(weightVector,weightFunction(Y_tau_hash - W_hash*Beta_next_it, 3));
                
                % Does weightVector converge ?
                deltaWeight = norm(weightVector - weightVector_old);
                if deltaWeight<thresholdWeight
                    weightsConverged = true;
                else
                    weightsConverged = false;
                end
                
            end
            
        otherwise
            
            error("IDIM_IV: unknown option");
    end
    
    % Computation of the new error:
    eps_next_it = norm(Y_tau - W*Beta_next_it);
    
    % Stop criteria:
    crit_1 = abs(norm(eps_next_it) - norm(eps_curr_it))/norm(eps_curr_it);
    crit_2 = max(abs((Beta_next_it-Beta)./Beta));
    
    % Actualization of the values:
    eps_curr_it = eps_next_it;
    it = it+1;
    Beta =  Beta_next_it;
    Betas(:,it)=Beta;
end

if crit_1 < optionsIDIM_IV.stopCrit.tol_1
    flag = 1;
    fprintf("crit_1 < tol_1\n");
else
    if crit_2 < optionsIDIM_IV.stopCrit.tol_2
        flag = 2;
        fprintf("crit_2 < tol_2\n");
    else
        if it >= optionsIDIM_IV.stopCrit.Max_it
            flag = 3;
            fprintf("Reached iteration limit\n");
        else
            flag = 0;
            fprintf("Error...\n");
        end
    end
end

Beta_IV = Beta;

%% Debug plot:

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_IV, optionsIDIM_IV, expNb , 'IDIM\_IV');

end
