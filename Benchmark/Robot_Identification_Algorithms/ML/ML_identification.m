function [Beta_ML] = ML_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsML)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the Maximum Likelihood (ML) method.

%% Data Decimation and Filtering:

[~, ~, ~, ~, ~, ~, Tau_decim, Qpp_decim, Qp_decim, Q_decim, optionsML] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsML, expNb, 'ML');

switch optionsML.filter
    case 'no'
        Sigma2 = diag([robot.numericalParameters.sd_q; 20*robot.numericalParameters.sd_q; 90*robot.numericalParameters.sd_q; robot.numericalParameters.sd_tau].^2);
    case 'lowpass'
        Sigma2 = diag([5e-1*robot.numericalParameters.sd_q; robot.numericalParameters.sd_q; 2*robot.numericalParameters.sd_q; robot.numericalParameters.sd_tau].^2);
    case 'butterworth'
        Sigma2 = diag([5e-1*robot.numericalParameters.sd_q; robot.numericalParameters.sd_q; 2*robot.numericalParameters.sd_q; robot.numericalParameters.sd_tau].^2);
    otherwise
        error('%s: Unknown filter type !', algName);
end

% Sigma2 = diag([optionsML.sigma_Q; optionsML.sigma_Qp; optionsML.sigma_Qpp; optionsML.sigma_Tau].^2);

%% ML Identification:

% Initial population for PSO and GA:
numcores = feature('numcores');
population = 20*numcores;

lambda.upper = 0;
lambda.lower = 0;
jacobian = 0;


switch optionsML.optimizer
    case 'simplex'
        if optionsML.debug == true
            options = optimset('Display', 'iter', 'MaxIter', optionsML.stopCrit.Max_it, 'TolFun', optionsML.stopCrit.tol_1, 'TolX', optionsML.stopCrit.tol_2, 'PlotFcns', {@optimplotx, @optimplotfunccount, @optimplotfval});
        else
            options = optimset('Display', 'iter', 'MaxIter', optionsML.stopCrit.Max_it, 'TolFun', optionsML.stopCrit.tol_1, 'TolX', optionsML.stopCrit.tol_2);
        end
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [Beta_ML,fval,exitflag,output] = fminsearch(@(theta) compute_ML_Cost_mex(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), Beta_0, options);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [Beta_ML,fval,exitflag,output] = fminsearch(@(theta) compute_ML_Cost(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), Beta_0, options);
        else
            error("ML: unknown option");
        end
    case 'lsqnonlin'
        if optionsML.debug == true
            options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsML.stopCrit.Max_it, 'FunctionTolerance', optionsML.stopCrit.tol_1, 'StepTolerance', optionsML.stopCrit.tol_2,'UseParallel', true, 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotfval, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
        else
            options = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt', 'MaxIterations', optionsML.stopCrit.Max_it, 'FunctionTolerance', optionsML.stopCrit.tol_1, 'StepTolerance', optionsML.stopCrit.tol_2,'UseParallel', true);
        end
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [Beta_ML,resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(theta) compute_ML_Cost_mex(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, false), Beta_0,[],[], options);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [Beta_ML,resnorm,fval,exitflag,output,lambda,jacobian] = lsqnonlin(@(theta) compute_ML_Cost(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, false), Beta_0,[],[], options);
        else
            error("ML: unknown option");
        end
    case 'fmin'
        if optionsML.debug == true
            options = optimoptions('fminunc', 'Algorithm','quasi-newton', 'MaxIterations', optionsML.stopCrit.Max_it, 'FunctionTolerance', optionsML.stopCrit.tol_1, 'StepTolerance', optionsML.stopCrit.tol_2,'UseParallel', true, 'PlotFcn',{@optimplotx, @optimplotfunccount, @optimplotfval, @optimplotresnorm, @optimplotstepsize, @optimplotfirstorderopt});
        else
            options = optimoptions('fminunc', 'Algorithm','quasi-newton', 'MaxIterations', optionsML.stopCrit.Max_it, 'FunctionTolerance', optionsML.stopCrit.tol_1, 'StepTolerance', optionsML.stopCrit.tol_2,'UseParallel', true);
        end
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [Beta_ML,resnorm,fval,exitflag,output] = fminunc(@(theta) compute_ML_Cost_mex(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), Beta_0, options);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [Beta_ML,resnorm,fval,exitflag,output] = fminunc(@(theta) compute_ML_Cost(robot.name, theta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), Beta_0, options);
        else
            error("ML: unknown option");
        end
    case 'pso'
        initialPopulationMatrix = repmat(Beta_0',population,1);
        initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
        if optionsML.debug == true
            options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'PlotFcn', @pswplotbestf, 'UseParallel', true);
        else
            options = optimoptions('particleswarm', 'SwarmSize', population, 'InitialSwarmMatrix', initialPopulationMatrix, 'Display', 'iter', 'UseParallel', true);
        end
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [Beta,fval,exitflag,output] = particleswarm(@(theta) compute_ML_Cost_mex(robot.name, theta', Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), robot.paramVectorSize, benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, options);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [Beta,fval,exitflag,output] = particleswarm(@(theta) compute_ML_Cost(robot.name, theta', Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), robot.paramVectorSize, benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, options);
        else
            error("ML: unknown option");
        end
        Beta_ML = Beta';
    case 'ga'
        initialPopulationMatrix = repmat(Beta_0',population,1);
        initialPopulationMatrix = initialPopulationMatrix + 0.01*randn(size(initialPopulationMatrix)).*initialPopulationMatrix;
        if optionsML.debug == true
            options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', 1e-10, 'Display', 'iter', 'MaxGenerations', optionsML.stopCrit.Max_it, 'PlotFcn', {@gaplotbestindiv,@gaplotdistance,@gaplotrange}, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
        else
            options = optimoptions('ga', 'MaxStallGenerations', 20, 'MutationFcn', @mutationadaptfeasible, 'FunctionTolerance', 1e-10, 'Display', 'iter', 'MaxGenerations', optionsML.stopCrit.Max_it, 'PopulationSize', population, 'InitialPopulationMatrix', initialPopulationMatrix, 'UseParallel', true, 'EliteCount', floor(population/2), 'CrossoverFraction', 0.9);
        end
        if strcmp(benchmarkSettings.codeImplementation,'optim')
            [Beta,fval,exitflag,output] = ga(@(theta) compute_ML_Cost_mex(robot.name, theta', Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), robot.paramVectorSize, [], [], [], [], benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, [], options);
        elseif strcmp(benchmarkSettings.codeImplementation,'classic')
            [Beta,fval,exitflag,output] = ga(@(theta) compute_ML_Cost(robot.name, theta', Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, true), robot.paramVectorSize, [], [], [], [], benchmarkSettings.Beta_L, benchmarkSettings.Beta_U, [], options);
        else
            error("ML: unknown option");
        end
        Beta_ML = Beta';
    otherwise
        error("ML: unknown optimization algorithm");
end

%% Debug plot:

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_ML, optionsML, expNb, 'ML');

end


