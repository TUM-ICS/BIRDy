function results_ML = run_ML(robot, benchmarkSettings, experimentDataStruct, optionsML, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('IDIM-LS: First Iteration...'));
end

results_ML.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_ML.options = optionsML; % Options that are specific to the identification method.
results_ML.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_ML.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_ML] =  ML_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsML);
        results_ML.times(initEst, expNb) = toc;
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('IDIM-LS: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        if optionsML.verbose == true
            fprintf('ML status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('ML status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('ML status: estimated parameter error = %d\n', norm(Beta_ML-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')  
        end
        
        results_ML.Betas(initEst,expNb,:)=Beta_ML;
    end
end
end

