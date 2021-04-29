function results_HTRNN = run_HTRNN(robot, benchmarkSettings, experimentDataStruct, optionsHTRNN, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('HTRNN: First Iteration...'));
end

results_HTRNN.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_HTRNN.options = optionsHTRNN; % Options that are specific to the identification method.
results_HTRNN.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_HTRNN.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_HTRNN.iterations = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Number of iterations   

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_HTRNN, it_HTRNN] =  HTRNN_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsHTRNN);
        results_HTRNN.times(initEst, expNb)=toc;
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('Hopfield NN: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        if optionsHTRNN.verbose == true
            fprintf('HTRNN status: initial estimate %d, experiment %d \n', initEst, expNb );
            fprintf('HTRNN status: initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('HTRNN status: estimated parameter error = %d\n', norm(Beta_HTRNN-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')  
        end
                
        results_HTRNN.Betas(initEst,expNb,:)=Beta_HTRNN;
        results_HTRNN.iteration(initEst,expNb)=it_HTRNN;
    end
end

end

