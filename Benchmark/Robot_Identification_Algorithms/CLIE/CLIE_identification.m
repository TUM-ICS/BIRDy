function [Beta_CLIE, fval, output, exitflag, lambda, jacobian] = CLIE_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsCLIE)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the Closed-Loop Input Error (CLIE) method.

%% Data Decimation and Filtering:

[~, ~, ~, ~, ~, ~, Tau_decim, ~, ~, ~, optionsCLIE] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsCLIE, expNb, 'CLIE');

%% CLIE Identification:

% Interesting: allows to visualize the optimization landscape for two variables... Takes a lot of time to execute !
% fcontour(@(x, y) compute_CLIE_Cost(robot, benchmarkSettings, [x;y;Beta_0(3:end)], Tau_decim, optionsCLIE, t_ndecim),[benchmarkSettings.Beta_L(1) benchmarkSettings.Beta_U(1) benchmarkSettings.Beta_L(2) benchmarkSettings.Beta_U(2)], 'MeshDensity',10)

if optionsCLIE.isParamWise == true % Parameter-wise identification (takes a very long time !!!)
    Beta_it  = Beta_0;
    Beta_next_it =  Beta_0;
    Betas = zeros(robot.paramVectorSize, optionsCLIE.stopCrit.Max_it+1);
    
    F = compute_CLIE_Cost(robot, benchmarkSettings, Beta_0, Tau_decim, optionsCLIE);
    
    stop_1 = 1;
    stop_2 = 1;
    it = 1;
    while it <= optionsCLIE.stopCrit.Max_it && stop_1 > optionsCLIE.stopCrit.tol_1 && stop_2 > optionsCLIE.stopCrit.tol_2
        Betas(:,it)=Beta_it;
        lambda.upper = 0;
        lambda.lower = 0;
        jacobian = 0;
        for coeff_beta=1:robot.paramVectorSize
            fprintf('Iteration %d, Parameter %d', it, coeff_beta)
            switch optionsCLIE.paramwiseOptimizer
                case 'simplex'
                    if optionsCLIE.debug == true
                        options = optimset('Display', 'iter', 'PlotFcns', {@optimplotx, @optimplotfunccount});
                    else
                        options = optimset('Display', 'iter');
                    end
                    [Beta_next_it(coeff_beta),fval,exitflag,output] = fminsearch(@(val_beta) compute_CLIE_Cost(robot, benchmarkSettings, Beta_it, Tau_decim, optionsCLIE, coeff_beta, val_beta), Beta_next_it(coeff_beta), options);
                case 'lsqnonlin'
                    if optionsCLIE.debug == true
                        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
                    else
                        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt');
                    end
                    [Beta_next_it(coeff_beta),resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(val_beta) compute_CLIE_Cost(robot, benchmarkSettings, Beta_it, Tau_decim, optionsCLIE, coeff_beta, val_beta), Beta_next_it(coeff_beta), [], [], options);
                otherwise
                    error("CLIE: unknown paramwiseOptimizer option");
            end
        end
        
        % Updating stop criteria:
        stop_1 = norm(Beta_next_it-Beta_it)/norm(Beta_it);
        G = compute_CLIE_Cost(robot, benchmarkSettings, Beta_next_it, Tau_decim, optionsCLIE);
        stop_2 = abs(G-F);
        F = G;
        
        Beta_it = Beta_next_it;
        it = it+1;
    end
    
    Beta_CLIE = Beta_next_it;
    
    if it == optionsCLIE.stopCrit.Max_it+1
        flag = 1;
    elseif stop_1 <= optionsCLIE.stopCrit.tol_1
        flag = 2;
    elseif stop_2 <= optionsCLIE.stopCrit.tol_2
        flag = 3;
    else
        flag = 0;
    end
    
else % Global parameter vector identification
    
    numcores = feature('numcores');
    population = 20*numcores;
    
    lambda.upper = 0;
    lambda.lower = 0;
    jacobian = 0;
    switch optionsCLIE.globalOptimizer
        case 'simplex'
            if optionsCLIE.debug == true
                options = optimset('Display', 'iter', 'MaxIter', optionsCLIE.stopCrit.Max_it, 'TolFun', optionsCLIE.stopCrit.tol_1, 'TolX', optionsCLIE.stopCrit.tol_2, 'PlotFcns', {@optimplotx, @optimplotfunccount, @optimplotfval});
            else
                options = optimset('Display', 'iter', 'MaxIter', optionsCLIE.stopCrit.Max_it, 'TolFun', optionsCLIE.stopCrit.tol_1, 'TolX', optionsCLIE.stopCrit.tol_2);
            end
            [Beta_CLIE,fval,exitflag,output] = fminsearch(@(beta) compute_CLIE_Cost(robot, benchmarkSettings, beta, Tau_decim, optionsCLIE), Beta_0, options);
        case 'lsqnonlin'
            if optionsCLIE.debug == true
                options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsCLIE.stopCrit.Max_it, 'FunctionTolerance', optionsCLIE.stopCrit.tol_1, 'StepTolerance', optionsCLIE.stopCrit.tol_2,'UseParallel', true, 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
            else
                options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsCLIE.stopCrit.Max_it, 'FunctionTolerance', optionsCLIE.stopCrit.tol_1, 'StepTolerance', optionsCLIE.stopCrit.tol_2,'UseParallel', true);
            end
            [Beta_CLIE,resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(beta) compute_CLIE_Cost(robot, benchmarkSettings, beta, Tau_decim, optionsCLIE), Beta_0,[],[], options);
        case 'pso'
            initialPopulationMatrix = repmat(Beta_0',population,1);
            initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
            if optionsCLIE.debug == true
                options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'PlotFcn', @pswplotbestf, 'UseParallel', true);
            else
                options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'UseParallel', true);
            end
            [Beta_CLIE,fval,exitflag,output] = particleswarm(@(beta) compute_CLIE_Cost(robot, benchmarkSettings, beta', Tau_decim, optionsCLIE), robot.paramVectorSize, benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, options);
            Beta_CLIE = Beta_CLIE.';
        case 'ga'
            initialPopulationMatrix = repmat(Beta_0',population,1);
            initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
            if optionsCLIE.debug == true
                options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', optionsCLIE.stopCrit.tol_1, 'Display', 'iter', 'MaxGenerations', optionsCLIE.stopCrit.Max_it, 'PlotFcn', {@gaplotbestindiv,@gaplotdistance,@gaplotrange}, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
            else
                options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', optionsCLIE.stopCrit.tol_1, 'Display', 'iter', 'MaxGenerations', optionsCLIE.stopCrit.Max_it, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
            end
            [Beta_CLIE,fval,exitflag,output] = ga(@(beta) compute_CLIE_Cost(robot, benchmarkSettings, beta', Tau_decim, optionsCLIE), robot.paramVectorSize, [], [], [], [], benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, [], options);
            Beta_CLIE = Beta_CLIE.';
    end
end

%% Debug plot

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_CLIE, optionsCLIE, expNb, 'CLIE');

end


function [costVariable] = compute_CLIE_Cost(robot, benchmarkSettings, beta, Tau_decim, optionsCLIE, coeff_beta, val_beta)

if nargin >= 10
    Beta = beta;
    Beta(coeff_beta) = val_beta;
else
    Beta = beta;
end

if strcmp(benchmarkSettings.codeImplementation,'optim')
    % Compute the sampled vector:
    Y_tau = torqueVector_mex(Tau_decim);
elseif strcmp(benchmarkSettings.codeImplementation,'classic')
    % Compute the sampled vector:
    Y_tau = torqueVector(Tau_decim);
else
    error("CLIE: unknown option");
end

W = zeros(robot.nbDOF*optionsCLIE.nbSampleDecim, robot.paramVectorSize);
t_ctrl = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_ctrl, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];

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
    error("CLIE: unknown implementation option");
end

Qp_it = State_it(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
Q_it = State_it(robot.nbDOF+1:end,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
Qpp_it = Qpp_itt(:,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);

if strcmp(benchmarkSettings.codeImplementation,'optim')
    % Building of the observation matrix W for the current iteration of Beta:
    W_ndec = observationMatrix_mex(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_it, Qp_it, Qpp_it);
elseif strcmp(benchmarkSettings.codeImplementation,'classic')
    % Building of the observation matrix W for the current iteration of Beta:
    W_ndec = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_it, Qp_it, Qpp_it);
else
    error("CLIE: unknown option");
end

% Decimation filter
for col=1:robot.paramVectorSize
    for i = 1:robot.nbDOF
        W(i:robot.nbDOF:end,col) = decimate(W_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
    end
end

Y_tau_reconstructed = W*Beta;
err = Y_tau_reconstructed - Y_tau;

if strcmp(optionsCLIE.globalOptimizer, 'lsqnonlin')
    costVariable = err(:);
else
    costVariable = err(:)'*err(:);
end

end

