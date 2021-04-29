function results_CLOE = run_CLOE(robot, benchmarkSettings, experimentDataStruct, optionsCLOE, progressBar)

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('CLOE: First Iteration...'));
end

results_CLOE.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_CLOE.options = optionsCLOE; % Options that are specific to the identification method.
results_CLOE.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_CLOE.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_CLOE.flag = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Termination flag of the identification code
results_CLOE.iterations = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Number of iterations  
results_CLOE.lambdas_upper = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize);
results_CLOE.lambdas_lower = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize);
% results_CLOE.jacobians = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, (benchmarkSettings.nbSamples-2*benchmarkSettings.samplingBorder)*robot.nbDOF/benchmarkSettings.decimRate, robot.paramVectorSize);
results_CLOE.jacobian_sigma_upper = zeros(benchmarkSettings.numberOfInitialEstimates); % Upper singular value of the Jacobian matrix.
results_CLOE.jacobian_sigma_lower = zeros(benchmarkSettings.numberOfInitialEstimates); % Lower singular value of the Jacobian matrix.

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_CLOE, fval, output, exitflag, lambda, jacobian] = CLOE_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsCLOE);
        results_CLOE.times(initEst, expNb)=toc;
        
        if optionsCLOE.verbose == true
            fprintf('CLOE status: initial estimate %d, experiment %d \n', initEst, expNb);
            fprintf('CLOE status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('CLOE status: estimated parameter error = %d\n', norm(Beta_CLOE-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')
        end
                      
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('CLOE: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        results_CLOE.flag(initEst,expNb)=exitflag;
        results_CLOE.Betas(initEst,expNb,:)=Beta_CLOE;
        results_CLOE.eps(initEst, expNb)=norm(Beta_CLOE-benchmarkSettings.Beta_obj);
        results_CLOE.iteration(initEst,expNb)=output;
        results_CLOE.lambdas_upper(initEst, expNb, :) = lambda.upper;
        results_CLOE.lambdas_lower(initEst, expNb, :) = lambda.lower;
%         results_CLOE.jacobians(initEst, expNb, :, :) = jacobian; % Requires too much memory
        
        s = svd(jacobian);
        results_CLOE.jacobian_sigma_upper(initEst, expNb) = max(s);
        results_CLOE.jacobian_sigma_lower(initEst, expNb) = min(s);
        
    end
end
end

