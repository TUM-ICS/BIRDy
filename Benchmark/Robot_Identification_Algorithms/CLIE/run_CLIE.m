function results_CLIE = run_CLIE(robot, benchmarkSettings, experimentDataStruct, optionsCLIE, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('CLIE: First Iteration...'));
end

results_CLIE.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_CLIE.options = optionsCLIE; % Options that are specific to the identification method.
results_CLIE.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_CLIE.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_CLIE.flag = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Termination flag of the identification code
results_CLIE.iterations = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Number of iterations  
results_CLIE.lambdas_upper = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize);
results_CLIE.lambdas_lower = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize);
% results_CLIE.jacobians = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, (benchmarkSettings.nbSamples-2*benchmarkSettings.samplingBorder)*robot.nbDOF/benchmarkSettings.decimRate, robot.paramVectorSize);
results_CLIE.jacobian_sigma_upper = zeros(benchmarkSettings.numberOfInitialEstimates); % Upper singular value of the Jacobian matrix.
results_CLIE.jacobian_sigma_lower = zeros(benchmarkSettings.numberOfInitialEstimates); % Lower singular value of the Jacobian matrix.

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_CLIE, fval, output, exitflag, lambda, jacobian] = CLIE_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsCLIE);
        results_CLIE.times(initEst, expNb)=toc;
        
        if optionsCLIE.verbose == true
            fprintf('CLIE status: initial estimate %d, experiment %d \n', initEst, expNb);
            fprintf('CLIE status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('CLIE status: estimated parameter error = %d\n', norm(Beta_CLIE-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')
        end
                      
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('CLIE: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        results_CLIE.flag(initEst,expNb)=exitflag;
        results_CLIE.Betas(initEst,expNb,:)=Beta_CLIE;
        results_CLIE.eps(initEst, expNb)=norm(Beta_CLIE-benchmarkSettings.Beta_obj);
        results_CLIE.iteration(initEst,expNb)=output;
        results_CLIE.lambdas_upper(initEst, expNb, :) = lambda.upper;
        results_CLIE.lambdas_lower(initEst, expNb, :) = lambda.lower;
%         results_CLIE.jacobians(initEst, expNb, :, :) = jacobian;  % Requires too much memory
        
        s = svd(jacobian);
        results_CLIE.jacobian_sigma_upper(initEst, expNb) = max(s);
        results_CLIE.jacobian_sigma_lower(initEst, expNb) = min(s);
        
    end
end
end

