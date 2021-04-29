function results_LS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_LS, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('IDIM-LS: First Iteration...'));
end

results_LS.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_LS.options = optionsIDIM_LS; % Options that are specific to the identification method.
results_LS.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_LS.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_LS.condObs = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Conditioning of the observation matrix.
results_LS.sigmaMin = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Minimum singular value of the observation matrix.

%% Run the identification code:

if strcmp(optionsIDIM_LS.alg, 'PC-OLS') || strcmp(optionsIDIM_LS.alg, 'PC-WLS') || strcmp(optionsIDIM_LS.alg, 'PC-IRLS') || strcmp(optionsIDIM_LS.solver, 'cvx') || strcmp(optionsIDIM_LS.solver, 'mosek')
    cvx_setup;
    if strcmp(optionsIDIM_LS.solver, 'mosek')
        cvx_solver mosek
    end
    if optionsIDIM_LS.debug == false
        cvx_quiet true
    end
end

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Xhi_0 = benchmarkSettings.Initial_Xhi(:,initEst);
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_LS, condObs_LS, sigmaMin_LS] =  IDIM_LS_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Xhi_0, Beta_0, optionsIDIM_LS);
        results_LS.times(initEst, expNb) = toc;
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('IDIM-LS: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        if optionsIDIM_LS.verbose == true
            fprintf('IDIM_LS status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('IDIM_LS status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('IDIM_LS status: estimated parameter error = %d\n', norm(Beta_LS-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')  
        end
        
        results_LS.Betas(initEst,expNb,:)=Beta_LS;
        results_LS.condObs = condObs_LS;
        results_LS.sigmaMin = sigmaMin_LS;
    end
end
end

