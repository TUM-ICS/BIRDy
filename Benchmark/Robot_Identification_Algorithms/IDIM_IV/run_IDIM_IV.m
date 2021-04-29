function results_IV = run_IDIM_IV(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_IV, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('IDIM-IV: First Iteration...'));
end

results_IV.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_IV.options = optionsIDIM_IV; % Options that are specific to the identification method.
results_IV.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_IV.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_IV.Betas_iteration = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj), optionsIDIM_IV.stopCrit.Max_it+1); % Identified parameters for each step of the iterative process
results_IV.flag = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Termination flag of the identification code
results_IV.iterations = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Number of iterations       

%% Run the identification code:

if strcmp(optionsIDIM_IV.alg, 'PC-OLS') || strcmp(optionsIDIM_IV.alg, 'PC-WLS') || strcmp(optionsIDIM_IV.alg, 'PC-IRLS') || strcmp(optionsIDIM_IV.solver, 'cvx') || strcmp(optionsIDIM_IV.solver, 'mosek')
    cvx_setup;
    if strcmp(optionsIDIM_IV.solver, 'mosek')
        cvx_solver mosek
    end
    if optionsIDIM_IV.debug == false
        cvx_quiet true
    end
end

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Xhi_0 = benchmarkSettings.Initial_Xhi(:,initEst);
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_IV, it_IV, Betas_IV, flag_IV] = IDIM_IV_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Xhi_0, Beta_0, optionsIDIM_IV);
        results_IV.times(initEst, expNb) = toc;

        if optionsIDIM_IV.verbose == true
            fprintf('IDIM_IV status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('IDIM_IV status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('IDIM_IV status: estimated parameter error = %d\n', norm(Beta_IV-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')
        end
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('IDIM-IV: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        results_IV.Betas(initEst,expNb,:)=Beta_IV;
        results_IV.flag(initEst,expNb)=flag_IV;
        results_IV.iterations(initEst,expNb)=it_IV;
        results_IV.Betas_iteration(initEst, expNb, :, :) = Betas_IV;
    end
end
end

