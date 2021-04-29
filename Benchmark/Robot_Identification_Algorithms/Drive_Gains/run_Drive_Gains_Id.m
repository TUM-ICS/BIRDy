function results_DriveGains = run_Drive_Gains_Id(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, progressBar)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the DriveGainsId method.

%% Define result data structure:

if benchmarkSettings.displayProgression == true
    waitbar(0, progressBar, sprintf('DriveGainsId: First Iteration...'));
end

results_DriveGains.benchmarkSettings = benchmarkSettings; % Data structure containing the benchmark settings
results_DriveGains.options = optionsDriveGainsId; % Options that are specific to the identification method.
results_DriveGains.Betas = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, numel(benchmarkSettings.Beta_obj)); % Identified parameters
results_DriveGains.times = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Computation times
results_DriveGains.condObs = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Conditioning of the observation matrix.
results_DriveGains.sigmaMin = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint); % Minimum singular value of the observation matrix.

%% Run the identification code:

        [Beta_DriveGains, condObs_DriveGains, sigmaMin_DriveGains] =  Drive_Gains_identification(robot, experimentDataStruct, benchmarkSettings, optionsDriveGainsId);
        
        if benchmarkSettings.displayProgression == true
            waitbar(((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint), progressBar, sprintf('DriveGainsId: %d%% done...', floor(100*((initEst-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+expNb)/((benchmarkSettings.numberOfInitialEstimates-1)*benchmarkSettings.numberOfExperimentsPerInitialPoint+benchmarkSettings.numberOfExperimentsPerInitialPoint))));
        end
        
        
        results_DriveGains.Beta=Beta_DriveGains;
        results_DriveGains.condObs = condObs_DriveGains;
        results_DriveGains.sigmaMin = sigmaMin_DriveGains;

end

