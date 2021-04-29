function results_DIDIM = run_DIDIM(robot, benchmarkSettings, experimentDataStruct, optionsDIDIM, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('DIDIM: First Iteration...'));
end

results_DIDIM.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_DIDIM.options = optionsDIDIM; % Options that are specific to the identification method.
results_DIDIM.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_DIDIM.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_DIDIM.Betas_iteration = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj), optionsDIDIM.stopCrit.Max_it+1); % Identified parameters for each step of the iterative process
results_DIDIM.flag = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Termination flag of the identification code
results_DIDIM.iterations = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Number of iterations   

%% Run the identification code:

if strcmp(optionsDIDIM.alg, 'PC-OLS') || strcmp(optionsDIDIM.alg, 'PC-WLS') || strcmp(optionsDIDIM.alg, 'PC-IRLS') || strcmp(optionsDIDIM.solver, 'cvx') || strcmp(optionsDIDIM.solver, 'mosek')
    cvx_setup;
    if strcmp(optionsDIDIM.solver, 'mosek')
        cvx_solver mosek
    end
    if optionsDIDIM.debug == false
        cvx_quiet true
    end
end

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Xhi_0 = benchmarkSettings.Initial_Xhi(:,initEst);
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_DIDIM, it_DIDIM, Betas_DIDIM, flag_DIDIM] = DIDIM_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Xhi_0, Beta_0, optionsDIDIM);
        results_DIDIM.times(initEst, expNb)=toc;
        
        if optionsDIDIM.verbose == true
            fprintf('DIDIM status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('DIDIM status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('DIDIM status: estimated parameter error = %d\n', norm(Beta_DIDIM-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')
        end
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('DIDIM: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        results_DIDIM.Betas(initEst,expNb,:)=Beta_DIDIM;
        results_DIDIM.flag(initEst,expNb)=flag_DIDIM;
        results_DIDIM.iterations(initEst,expNb)=it_DIDIM;
        results_DIDIM.Betas_iteration(initEst, expNb, :, :) = Betas_DIDIM;
    end
end
end

