function [Beta_LS, cond, sig_min] = IDIM_LS_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Xhi_0, Beta_0, optionsIDIM_LS)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the IDIM-LS method.

%% Data Decimation and Filtering:

[~, ~, Qpp, Qp, Q, ~, Tau_decim, ~, ~, ~, optionsIDIM_LS] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_LS, expNb, 'IDIM_LS');

%% Populating the Observation Matrix W and Torque Vector:

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
    error("IDIM_LS: unknown option");
end

% Decimation filter
W=zeros(robot.nbDOF*optionsIDIM_LS.nbSampleDecim, robot.paramVectorSize); % Observation matrix
for col=1:robot.paramVectorSize
    for i = 1:robot.nbDOF
        W(i:robot.nbDOF:end,col) = decimate(W_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
    end
end

S=svd(W,0);
cond = max(S)/min(S);
sig_min = min(S);
thresholdOmega = 1e-2;
thresholdWeight = 1;

switch optionsIDIM_LS.alg
    
    case 'OLS' % Ordinary Least Squares
        
        Beta_LS = LeastSquares(robot, W, Y_tau, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, false, Xhi_0, Beta_0);
 
    case 'WLS' % Weighted least squares
        
        [sig2_error] = getErrorCovariance(W, Y_tau, robot.nbDOF);
        Omega_inv_sqrt = diag(sqrt(1./sig2_error));
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
        else
            error("IDIM_LS: unknown option");
        end
        
        Beta_LS = LeastSquares(robot, W_star, Y_tau_star, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, false, Xhi_0, Beta_0);
        
    case 'IRLS' % Iteratively Reweighted Least Squares
        
        weightsConverged = false;
        omegaConverged = false;
        
        weightVector = ones(optionsIDIM_LS.nbSampleDecim*robot.nbDOF,1);
        Omega = eye(robot.nbDOF); % Initially, Omega is an identity matrix. As iterations go on within this loop, Omega converges to a constant matrix whose diagonal elements denote the estimated variance of the corresponding joint/channel.
        
        while weightsConverged == false
            
            while omegaConverged == false
                [oU,oS,oV] = svd(Omega);
                Omega_inv_sqrt = oU*sqrt(inv(oS))*oV.';
                if strcmp(benchmarkSettings.codeImplementation,'optim')
                    [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
                elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                    [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
                else
                    error("IDIM_LS: unknown option");
                end
                
                W_hash = repmat(weightVector,1,robot.paramVectorSize).*W_star;
                Y_tau_hash = weightVector.*Y_tau_star;
                
                Beta_LS = LeastSquares(robot, W_hash, Y_tau_hash, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, false, Xhi_0, Beta_0);
                
                % Update Omega
                Omega_old = Omega;
                Omega = updateErrorFullCovariance(W_hash, Y_tau_hash, Beta_LS, inv(Omega_inv_sqrt), robot.nbDOF);
                
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
            weightVector = min(weightVector,weightFunction(Y_tau_hash - W_hash*Beta_LS, 3));
            
            % Does weightVector converge ?
            deltaWeight = norm(weightVector - weightVector_old);
            if deltaWeight<thresholdWeight
                weightsConverged = true;
            else
                weightsConverged = false;
            end
            
        end
        
    case 'PC-OLS' % Physically Consistent OLS, constrained with LMI
        
        Beta_LS = LeastSquares(robot, W, Y_tau, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, true, Xhi_0, Beta_0);
        
    case 'PC-WLS' % Physically Consistent WLS, constrained with LMI
        
        [sig2_error] = getErrorCovariance(W, Y_tau, robot.nbDOF);
        Omega_inv_sqrt = diag(sqrt(1./sig2_error));
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
        else
            error("IDIM_LS: unknown option");
        end
        
        Beta_LS = LeastSquares(robot, W_star, Y_tau_star, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, true, Xhi_0, Beta_0);
        
    case 'PC-IRLS' % Physically Consistent IRLS, constrained with LMI
        
        weightsConverged = false;
        omegaConverged = false;
        
        weightVector = ones(optionsIDIM_LS.nbSampleDecim*robot.nbDOF,1);
        Omega = eye(robot.nbDOF); % Initially, Omega is an identity matrix. As iterations go on within this loop, Omega converges to a constant matrix whose diagonal elements denote the estimated variance of the corresponding joint/channel.
        
        while weightsConverged == false
            
            while omegaConverged == false
                [oU,oS,oV] = svd(Omega);
                Omega_inv_sqrt = oU*sqrt(inv(oS))*oV.';
                if strcmp(benchmarkSettings.codeImplementation,'optim')
                    [W_star, Y_tau_star] = weightedObsservationTorque_mex(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
                elseif strcmp(benchmarkSettings.codeImplementation,'classic')
                    [W_star, Y_tau_star] = weightedObsservationTorque(W, Y_tau, Omega_inv_sqrt, optionsIDIM_LS.nbSampleDecim, robot.nbDOF);
                else
                    error("IDIM_LS_LMI: unknown option");
                end
                W_hash = repmat(weightVector,1,robot.paramVectorSize).*W_star;
                Y_tau_hash = weightVector.*Y_tau_star;
                
                Beta_LS = LeastSquares(robot, W_hash, Y_tau_hash, optionsIDIM_LS.solver, optionsIDIM_LS.regularizerType, optionsIDIM_LS.gammaReg, true, Xhi_0, Beta_0);
                
                % Update Omega
                Omega_old = Omega;
                Omega = updateErrorFullCovariance(W_hash, Y_tau_hash, Beta_LS, inv(Omega_inv_sqrt), robot.nbDOF);
                
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
            weightVector = min(weightVector,weightFunction(Y_tau_hash - W_hash*Beta_LS, 3));
            
            % Does weightVector converge ?
            deltaWeight = norm(weightVector - weightVector_old);
            if deltaWeight<thresholdWeight
                weightsConverged = true;
            else
                weightsConverged = false;
            end
            
        end
        
    case 'TLS' % Total least squares
        
        if strcmp(optionsIDIM_LS.solver,'svd') % Classic TLS solution
            
            Z = [W Y_tau];
            [~,~,V] = svd(Z,0);
            Beta_LS = -V(1:end-1,end)./V(end,end);
            
        elseif strcmp(optionsIDIM_LS.solver,'svd_ridge') % Minimum-norm TLS solution
            
            Z = [W Y_tau];
            s = svd(Z,0);
            Beta_LS = (W.'*W - min(s)^2*eye(robot.paramVectorSize))\(W.'*Y_tau);
            
        elseif strcmp(optionsIDIM_LS.solver,'recursive') % Classic TLS solution formulated in a recursive manner
            
            Beta_LS = Beta_0;
            it = 1;
            J = jacobianTLS(Beta_LS, W, Y_tau);
            f = costTLS(Beta_LS, W, Y_tau);
            stop_1 = norm(J.'*f);
            
            while it <= optionsIDIM_LS.stopCrit.Max_it && stop_1 > optionsIDIM_LS.stopCrit.tol_1
                Beta_LS = Beta_LS + -J\f;
                it = it+1;
                J = jacobianTLS(Beta_LS, W, Y_tau);
                f = costTLS(Beta_LS, W, Y_tau);
                stop_1 = norm(J.'*f);
            end
            
        else
            error("IDIM_TLS: unknown option");
        end
        
    otherwise
        error("IDIM_LS: unknown algorithm");
end

%% Debug plot:

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_LS, optionsIDIM_LS, expNb, 'IDIM\_LS');

end

function [f] = costTLS(x, A, b)

mu = 1/sqrt(1+x.'*x);
f = mu*[A b]*[x;-1];

end

function [J] = jacobianTLS(x, A, b)

mu = 1/sqrt(1+x.'*x);
J = mu*(A-mu^2*(A*x-b)*x.');

end
