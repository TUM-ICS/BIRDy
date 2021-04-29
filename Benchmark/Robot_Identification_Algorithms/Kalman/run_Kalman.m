function results_KF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsKF, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('Kalman Filter (%s): First Iteration...', optionsKF.type));
end

results_KF.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_KF.options = optionsKF; % Options that are specific to the identification method.
results_KF.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize); % Identified parameters
results_KF.flag = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Termination flag of the identification code
results_KF.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times

if optionsKF.getIterationData == true
    results_KF.Betas_iteration = zeros(robot.paramVectorSize, ceil(benchmarkSettings.nbSamples/optionsKF.samplingFactor), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint); % Identified parameters for each step of the iterative process
    results_KF.Covariance_iteration = zeros(robot.paramVectorSize+2*robot.nbDOF, robot.paramVectorSize+2*robot.nbDOF, ceil(benchmarkSettings.nbSamples/optionsKF.samplingFactor), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint); % Identified parameters covatriance for each step of the iterative process
    results_KF.noiseAnneal = zeros(benchmarkSettings.nbSamples, ceil(benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint/optionsKF.samplingFactor)); % Process noise annealing parameter
end 

%% Run the identification code:

for initEst = 1:benchmarkSettings.numberOfInitialEstimates % For each set of initial parameters
    
    Beta_0 = benchmarkSettings.Initial_Beta(:,initEst);
    
    for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % For each trajectory noise
        tic
        [Beta_KF, State_KF, Covariance_KF, noiseAnneal_KF, flag_KF] =  KF_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsKF);
        results_KF.times(initEst, expNb)=toc;
   
        if optionsKF.verbose == true
            fprintf('%s status: initial estimate %d, experiment %d \n', optionsKF.type, initEst, expNb );
            fprintf('%s status: initial parameter error = %d\n', optionsKF.type, norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('%s status: estimated parameter error = %d\n', optionsKF.type, norm(Beta_KF-benchmarkSettings.Beta_obj));
            disp('---------------------------------------------')
        end
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('Kalman Filter: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        results_KF.Betas(initEst,expNb,:)=Beta_KF;
        results_KF.flag(initEst,expNb)=flag_KF;
        if optionsKF.getIterationData == true
            results_KF.Betas_iteration(:,:,benchmarkSettings.numberOfExperimentsPerInitialPoint*(initEst-1)+expNb)=State_KF(:,2*robot.nbDOF+1:end)'; % Store only one column out of 10
            results_KF.Covariance_iteration(:,:,:,benchmarkSettings.numberOfExperimentsPerInitialPoint*(initEst-1)+expNb)=Covariance_KF; % Store only one column out of 10
            results_KF.noiseAnneal(:, benchmarkSettings.numberOfExperimentsPerInitialPoint*(initEst-1)+expNb)=noiseAnneal_KF;
        end
    end
end

end
