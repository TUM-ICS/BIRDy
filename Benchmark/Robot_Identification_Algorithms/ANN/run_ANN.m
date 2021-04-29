function results_ANN = run_ANN(robot, benchmarkSettings, experimentDataStruct, optionsANN, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('ANN: First Iteration...'));
end

results_ANN.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_ANN.options = optionsANN; % Options that are specific to the identification method.
results_ANN.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_ANN.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_ANN] =  ANN_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsANN);
        results_ANN.times(initEst, expNb)=toc;
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('Adaline NN: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        if optionsANN.verbose == true
            fprintf('ANN status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('ANN status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('ANN status: estimated parameter error = %d\n', norm(Beta_ANN-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')  
        end
        
        results_ANN.Betas(initEst,expNb,:) = Beta_ANN;
    end
end

end

