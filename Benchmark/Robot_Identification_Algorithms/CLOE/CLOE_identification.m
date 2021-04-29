function [Beta_CLOE, fval, output, exitflag, lambda, jacobian] = CLOE_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsCLOE)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the Closed-Loop Output Error (CLOE) method.

%% Data Decimation and Filtering:

[~, ~, ~, ~, ~, ~, ~, Qpp_decim, Qp_decim, Q_decim, optionsCLOE] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsCLOE, expNb, 'CLOE');

%% CLOE Identification:

% Interesting: allows to visualize the optimization landscape for two variables... Takes a lot of time to execute !
% fcontour(@(x, y) compute_CLOE_Cost(robot, benchmarkSettings, [x;y;Beta_0(3:end)], Q, Qp, Qpp, optionsCLOE),[benchmarkSettings.Beta_L(1) benchmarkSettings.Beta_U(1) benchmarkSettings.Beta_L(2) benchmarkSettings.Beta_U(2)], 'MeshDensity',10)

if optionsCLOE.isParamWise == true % Parameter-wise identification (takes a very long time !!!)
    Beta_it  = Beta_0;
    Beta_next_it =  Beta_0;
    Betas = zeros(robot.paramVectorSize, optionsCLOE.stopCrit.Max_it+1);
    
    F = compute_CLOE_Cost(robot, benchmarkSettings, Beta_0, Q_decim, Qp_decim, Qpp_decim, optionsCLOE);
    
    stop_1 = 1;
    stop_2 = 1;
    it = 1;
    while it <= optionsCLOE.stopCrit.Max_it && stop_1 > optionsCLOE.stopCrit.tol_1 && stop_2 > optionsCLOE.stopCrit.tol_2
        Betas(:,it)=Beta_it;
        lambda.upper = 0;
        lambda.lower = 0;
        jacobian = 0;
        for coeff_beta=1:robot.paramVectorSize
            fprintf('Iteration %d, Parameter %d', it, coeff_beta)
            switch optionsCLOE.paramwiseOptimizer
                case 'simplex'
                    if optionsCLOE.debug == true
                        options = optimset('Display', 'iter', 'PlotFcns', {@optimplotx, @optimplotfunccount, @optimplotfval});
                    else
                        options = optimset('Display', 'iter', 'PlotFcns');
                    end
                    [Beta_next_it(coeff_beta),fval,exitflag,output] = fminsearch(@(val_beta) compute_CLOE_Cost(robot, benchmarkSettings, Beta_it, Q_decim, Qp_decim, Qpp_decim, optionsCLOE, coeff_beta, val_beta), Beta_next_it(coeff_beta), options);
                case 'lsqnonlin'
                    if optionsCLOE.debug == true
                        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
                    else
                        options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt');
                    end
                    [Beta_next_it(coeff_beta),resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(val_beta) compute_CLOE_Cost(robot, benchmarkSettings, Beta_it, Q_decim, Qp_decim, Qpp_decim, optionsCLOE, coeff_beta, val_beta), Beta_next_it(coeff_beta), [], [], options);
                otherwise
                    error("CLOE: unknown paramwiseOptimizer option");
            end
        end
        
        % Updating stop criteria:
        stop_1 = norm(Beta_next_it-Beta_it)/norm(Beta_it);
        G = compute_CLOE_Cost(robot, benchmarkSettings, Beta_next_it, Q_decim, Qp_decim, Qpp_decim, optionsCLOE);
        stop_2 = abs(G-F);
        F = G;
        
        Beta_it = Beta_next_it;
        it = it+1;
    end
    
    Beta_CLOE = Beta_next_it;
    
    if it == optionsCLOE.stopCrit.Max_it+1
        flag = 1;
    elseif stop_1 <= optionsCLOE.stopCrit.tol_1
        flag = 2;
    elseif stop_2 <= optionsCLOE.stopCrit.tol_2
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
    switch optionsCLOE.globalOptimizer
        case 'simplex'
            if optionsCLOE.debug == true
                options = optimset('Display', 'iter', 'MaxIter', optionsCLOE.stopCrit.Max_it, 'TolFun', optionsCLOE.stopCrit.tol_1, 'TolX', optionsCLOE.stopCrit.tol_2, 'PlotFcns', {@optimplotx, @optimplotfunccount, @optimplotfval});
            else
                options = optimset('Display', 'iter', 'MaxIter', optionsCLOE.stopCrit.Max_it, 'TolFun', optionsCLOE.stopCrit.tol_1, 'TolX', optionsCLOE.stopCrit.tol_2);
            end
            [Beta_CLOE,fval,exitflag,output] = fminsearch(@(beta) compute_CLOE_Cost(robot, benchmarkSettings, beta, Q_decim, Qp_decim, Qpp_decim, optionsCLOE), Beta_0, options);
        case 'lsqnonlin'
            if optionsCLOE.debug == true
                options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsCLOE.stopCrit.Max_it, 'FunctionTolerance', optionsCLOE.stopCrit.tol_1, 'StepTolerance', optionsCLOE.stopCrit.tol_2,'UseParallel', true, 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
            else
                options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsCLOE.stopCrit.Max_it, 'FunctionTolerance', optionsCLOE.stopCrit.tol_1, 'StepTolerance', optionsCLOE.stopCrit.tol_2,'UseParallel', true);
            end
            [Beta_CLOE,resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(beta) compute_CLOE_Cost(robot, benchmarkSettings, beta, Q_decim, Qp_decim, Qpp_decim, optionsCLOE), Beta_0,[],[], options);
        case 'pso'
            initialPopulationMatrix = repmat(Beta_0',population,1);
            initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
            if optionsCLOE.debug == true
                options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'PlotFcn', @pswplotbestf, 'UseParallel', true);
            else
                options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'UseParallel', true);
            end
            [Beta_CLOE,fval,exitflag,output] = particleswarm(@(beta) compute_CLOE_Cost(robot, benchmarkSettings, beta', Q_decim, Qp_decim, Qpp_decim, optionsCLOE), robot.paramVectorSize, benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, options);
            Beta_CLOE = Beta_CLOE.';
        case 'ga'
            initialPopulationMatrix = repmat(Beta_0',population,1);
            initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
            if optionsCLOE.debug == true
                options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', optionsCLOE.stopCrit.tol_1, 'Display', 'iter', 'MaxGenerations', optionsCLOE.stopCrit.Max_it, 'PlotFcn', {@gaplotbestindiv,@gaplotdistance,@gaplotrange}, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
            else
                options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', optionsCLOE.stopCrit.tol_1, 'Display', 'iter', 'MaxGenerations', optionsCLOE.stopCrit.Max_it, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
            end
            [Beta_CLOE,fval,exitflag,output] = ga(@(beta) compute_CLOE_Cost(robot, benchmarkSettings, beta', Q_decim, Qp_decim, Qpp_decim, optionsCLOE), robot.paramVectorSize, [], [], [], [], benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, [], options);
            Beta_CLOE = Beta_CLOE.';
    end
end

%% Debug plot

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_CLOE, optionsCLOE, expNb, 'CLOE');

end


function [costVariable] = compute_CLOE_Cost(robot, benchmarkSettings, beta, Q_decim, Qp_decim, Qpp_decim, optionsCLOE, coeff_beta, val_beta)

if nargin >= 10
    Beta = beta;
    Beta(coeff_beta) = val_beta;
else
    Beta = beta;
end

t_ctrl = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_ctrl, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
Q_it_decim = zeros(size(Q_decim));
Qp_it_decim = zeros(size(Qp_decim));
Qpp_it_decim = zeros(size(Qpp_decim));

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
    error("CLOE: unknown implementation option");
end

Qp_it = State_it(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
Q_it = State_it(robot.nbDOF+1:end,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
Qpp_it = Qpp_itt(:,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);

for i = 1:robot.nbDOF
    Q_it_decim(i,:) = decimate(Q_it(i,:),benchmarkSettings.decimRate);
    Qp_it_decim(i,:) = decimate(Qp_it(i,:),benchmarkSettings.decimRate);
    Qpp_it_decim(i,:) = decimate(Qpp_it(i,:),benchmarkSettings.decimRate);
end

switch optionsCLOE.errorFunction
    case 'Q'
        err = Q_it_decim - Q_decim;
    case 'Qp'
        err = Qp_it_decim - Qp_decim;
    case 'Qpp'
        err = Qpp_it_decim - Qpp_decim;
    otherwise
        error("CLOE: unknown errorFunction option");
end

if strcmp(optionsCLOE.globalOptimizer, 'lsqnonlin')
    costVariable = err(:);
else
    costVariable = err(:)'*err(:);
end

end

