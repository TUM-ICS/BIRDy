%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%        BIRDy: Benchmark for Identification of Robot Dynamics        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Benchmark initialization:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Reset workspace
clear
close all

% Automatically add the Benchmark folder and its subfolders to the path:
addpath('Utils');
addpath(genpath('Utils'));
addpath('Benchmark');
addpath(genpath('Benchmark'));

% Provide the name of the robot to be identified:
robotName = 'TX40_uncoupled'; % [TX40, TX40_uncoupled, RV2SQ]

% Identify the robot using a subset of the experiment trajectory [t_i, t_f]:
t_i = 0;                                    % Start time.
t_f = 10;                                   % End time.
numberOfInitialEstimates = 25;              % Number of different initial estimates (25).
sdOfInitialEstimates = 0.15;                % Standard deviation of the initial estimates.
freq_decim = 500;                           % Decimation frequency used to compute the decimation rate (see the Matlab 'decimate' documentation).
samplingBorder = 50;                        % Number of samples to be removed from the beginning and the end of the data sequence, in order to mitigate filtering artefacts... (50)
gammaReg = 1e0;                            % Regularization factor (for methods where regularization is activated)

% Real or Simulated robot ?
experimentOnTrueRobot = false;              % If true, indicates that the benchmark is executed on experiment data collected on a true robot.
identifyDriveGains = false;                 % If true, indicates that the drive gain identification process should be executed.

% Important parameters:
functionTolerance = 2.5e-2;                 % Function tolerance 2.5%
stepTolerance = 2.5e-2;                     % Step tolerance 2.5%
iterationMax = 10;                          % Maximumm number of iterations

 % Set the noise level for which the identification has to be done: 'oldNoise', 'lowPositionNoise', 'standardNoise', 'highPositionNoise', 'highTorqueNoise' and 'highPositionTorqueNoise'
noiseLevel = 'lowPositionNoise';
% noiseLevel = 'standardNoise';               
% noiseLevel = 'highPositionNoise';
% noiseLevel = 'highTorqueNoise';
% noiseLevel = 'highPositionTorqueNoise';
% noiseLevel = 'oldNoise';

initialParamType = 'RefSd';                 % Initial parameter estimate for algorithms that require it: 'RefSd', 'LS', 'LS_f'
regenerateModel = false;                    % If true, recomputes the robot symbolic model.
regenerateTrajectory = false;               % If true, launch the generation routine for new excitation trajectory.
regenerateData = false;                     % If true, generate new robot simulation data.
generateROSTrajectory = false;              % If true, generate a .yaml file containing the joint trajectory
runPostProcessing = true;                   % If true, runs the post processing function to graph the results.
integrationAlgorithm = 'rk1';               % Defines the integration algorithm used during Identification : 'rk1', 'rk2', 'rk4' or 'ode45'.
interpolator.Algorithm = 'makima';          % Defines the interpolation algorithm used for trajctory data: 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'.
interpolator.expAlgorithm = 'nearest';      % Defines the interpolation algorithm used for experiment data: 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'.
interpolator.visualizeInterpolator = false; % If true, triggers visualization of interpolated trajectories with the different available methods.

% MEX files compilation:
recompileMexFiles = false;                  % Recompile mex files of the project if true.
buildList = [1 1 1 1 1 1 1 1 1 1 1 1 1];    % List of mex files to be regenerated [torqueVector_mex observationMatrix_mex integrateClosedLoopDynamics_mex ekf_opt_mex srekf_opt_mex ukf_opt_mex srukf_opt_mex cdkf_opt_mex srcdkf_opt_mex pf_opt_mex compute_ML_Cost_mex]
% buildList = [0 0 0 0 0 0 0 0 0 0 0 0 1];
% Filter parameters:
% Low pass filter:
filter.lowpass.d = fdesign.lowpass('Fp,Fst,Ap,Ast', 50, 110, 0.01, 60, 1000); % Design of the filter for the LS with filtered joint acceleration.
filter.lowpass.Hd = design(filter.lowpass.d, 'equiripple');
% Butterworth filter:
filter.butterworth.freq_fil = 50;
filter.butterworth.nfilt = 4;

% Cosmetic parameters:
verboseFlag = true;                         % If true, the solver communicates the identification error at each step.
debugFlag = false;                          % If true, the solver plots the robot trajectory using the computed parameters.
displayProgression =  false;                % If true, display progression GUI for each identification algorithm.
filter.visualizeFilter = false;             % Filter tuning
displayTrajectory = false;                  % If true, display the robot joint trajectory.
displayControlPerformance = false;          % If true, display the robot tracking performance on the generated trajectory.
getKinematicSymbolicExpressions = false;    % If true, displays the robot homogeneous transforms and jacobians in symbolic form. Note that identificationMethods.isActivated should be zero for this to be working properly.

% Select the identification methods :
identificationMethods.algName = {
    'OLS', 'OLS_f',...
    'WLS', 'WLS_f', ...
    'TLS', 'TLS_f', ...
    'IRLS', 'IRLS_f', ...
    'ML', 'ML_f','IV',...
    'DIDIM', 'CLIE', 'CLOE', ...
    'EKF', 'SREKF', ...
    'UKF', 'SRUKF', ...
    'CDKF', 'SRCDKF', 'PF', ...
    'ANN', 'ANN_f', ...
    'HTRNN', 'HTRNN_f',...
    'PC_OLS', 'PC_OLS_f',...
    'PC_WLS', 'PC_WLS_f',...
    'PC_IRLS', 'PC_IRLS_f',...
    'PC_OLS_Euclidean', 'PC_OLS_Euclidean_f',...
    'PC_OLS_Entropic', 'PC_OLS_Entropic_f',...
    'PC_OLS_ConstPullback', 'PC_OLS_ConstPullback_f',...
    'PC_IV', 'PC_DIDIM'};


identificationMethods.isActivated = ...
            [1,...% OLS
            1,... % OLS_f
            1,... % WLS
            1,... % WLS_f
            1,... % TLS
            1,... % TLS_f
            1,... % IRLS
            1,... % IRLS_f
            1,... % ML
            1,... % ML_f
            1,... % IV
            1,... % DIDIM
            1,... % CLIE
            1,... % CLOE
            1,... % EKF
            1,... % SREKF
            1,... % UKF
            1,... % SRUKF
            1,... % CDKF
            1,... % SRCDKF
            1,... % PF
            1,... % ANN
            1,... % ANN_f
            1,... % HTRNN
            1,... % HTRNN_f
            1,... % PC_OLS
            1,... % PC_OLS_f
            1,... % PC_WLS
            1,... % PC_WLS_f
            1,... % PC_IRLS
            1,... % PC_IRLS_f
            1,... % PC_OLS_Euclidian
            1,... % PC_OLS_Euclidian_f
            1,... % PC_OLS_Entropic
            1,... % PC_OLS_Entropic_f
            1,... % PC_OLS_ConstPullback
            1,... % PC_OLS_ConstPullback_f
            1,... % PC_IV
            1];   % PC_DIDIM 


identificationMethods.showResults = ...
            [1,...% OLS
            1,... % OLS_f
            1,... % WLS
            1,... % WLS_f
            1,... % TLS
            1,... % TLS_f
            1,... % IRLS
            1,... % IRLS_f
            1,... % ML
            1,... % ML_f
            1,... % IV
            1,... % DIDIM
            1,... % CLIE
            1,... % CLOE
            1,... % EKF
            1,... % SREKF
            1,... % UKF
            1,... % SRUKF
            1,... % CDKF
            1,... % SRCDKF
            1,... % PF
            1,... % ANN
            1,... % ANN_f
            1,... % HTRNN
            1,... % HTRNN_f
            1,... % PC_OLS
            1,... % PC_OLS_f
            1,... % PC_WLS
            1,... % PC_WLS_f
            1,... % PC_IRLS
            1,... % PC_IRLS_f
            1,... % PC_OLS_Euclidian
            1,... % PC_OLS_Euclidian_f
            1,... % PC_OLS_Entropic
            1,... % PC_OLS_Entropic_f
            1,... % PC_OLS_ConstPullback
            1,... % PC_OLS_ConstPullback_f
            1,... % PC_IV
            1];   % PC_DIDIM

% Run the initialization routine:
[robot, benchmarkSettings, experimentDataStruct, startBenchmark, progressBar] = ...
    initBenchmark(robotName, identificationMethods, getKinematicSymbolicExpressions, recompileMexFiles, displayProgression, regenerateModel, ...
    regenerateTrajectory, regenerateData, displayTrajectory, displayControlPerformance, t_i, t_f, integrationAlgorithm, interpolator, numberOfInitialEstimates, ...
    sdOfInitialEstimates, freq_decim, samplingBorder,  buildList, filter, generateROSTrajectory, experimentOnTrueRobot, identifyDriveGains, initialParamType, noiseLevel, gammaReg);
fprintf('Decimation rate: %d\n',benchmarkSettings.decimRate);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Drive Gains Identification Process (only for real robot data):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if identifyDriveGains == true && experimentOnTrueRobot == true
    
    disp("%%%%%%%%   Starting Drive Gain Identification   %%%%%%%%")
    
    % Set Drive Gains Identification Options:
    optionsDriveGainsId.verbose = verboseFlag;                              % If true, the solver communicates the identification error at each step.
    optionsDriveGainsId.debug = debugFlag;                                  % If true, the solver plots the robot trajectory using the computed parameters.
    optionsDriveGainsId.solver = 'svd';                                     % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function
    optionsDriveGainsId.alg = 'WLS';                                        % [LS]: classic Least Squares, [WLS]: Weighted Least Squares, [TLS]: Total Least Squares, [RR]: Ridge Regression, [WRR]: Weighted Ridge Regression
    optionsDriveGainsId.filter = 'butterworth';                             % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
    
    % Run Drive Gains Identification:
    results_DriveGains = run_Drive_Gains_Id(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, progressBar);
    
    disp("%%%%%%%%   Finished Grive Gain Identification   %%%%%%%%")
    
    benchmarkSettings.identificationMethods.isActivated = 0*identificationMethods.isActivated; % Deactivate other identification methods
    identificationMethods.isActivated = 0*identificationMethods.isActivated;
    benchmarkSettings.identificationMethods.showResults = 0*identificationMethods.showResults;
    runPostProcessing = false;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (startBenchmark==true) || (runPostProcessing == true)
    
    methodIndex=1;
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    disp("%%%%%%%%%%%%      Starting Computations     %%%%%%%%%%%%")
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Ordinary Least Square (IDIM-OLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_OLS
        
        disp("%%%%%%%%%%%%%%%    Starting IDIM-OLS    %%%%%%%%%%%%%%%%")
        
        % Set IDIM_OLS Options:
        optionsIDIM_LS.verbose = verboseFlag;                               % If true, the solver communicates the identification error at each step.
        optionsIDIM_LS.debug = debugFlag;                                   % If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_LS.solver ='backslash';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsIDIM_LS.alg = 'OLS';                                         % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_LS.regularizerType = 'no';                              % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_LS.gammaReg = 0;                                        % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_LS.filter = 'no';                                       % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_OLS Identification:
        results_OLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_LS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_OLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%    Finished IDIM-OLS    %%%%%%%%%%%%%%%%")
        
    end % end IDIM_OLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% IDIM and Ordinary Least Square (IDIM-OLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_OLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%%%     Starting IDIM-OLS_f     %%%%%%%%%%%%%%")
        
        % Set IDIM_OLS_f Options:
        optionsIDIM_LS_f.verbose = verboseFlag;                          	% If true, the solver communicates the identification error at each step.
        optionsIDIM_LS_f.debug = debugFlag;                               	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_LS_f.solver = 'backslash';                              % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsIDIM_LS_f.alg = 'OLS';                                       % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_LS_f.regularizerType = 'no';                            % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_LS_f.gammaReg = 0;                                      % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_LS_f.filter = 'butterworth';                            % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_OLS_f Identification:
        results_OLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_LS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_OLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%   Finished IDIM-OLS_f   %%%%%%%%%%%%%%%%")
        
    end % end IDIM_OLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Weighted Least Square (IDIM-WLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_WLS
        
        disp("%%%%%%%%%%%%%%%%    Starting IDIM-WLS    %%%%%%%%%%%%%%%")
        
        % Set IDIM_WLS Options:
        optionsIDIM_WLS.verbose = verboseFlag;                             	% If true, the solver communicates the identification error at each step.
        optionsIDIM_WLS.debug = debugFlag;                                  % If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_WLS.solver ='backslash';                                % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsIDIM_WLS.alg = 'WLS';                                        % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_WLS.regularizerType = 'no';                             % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_WLS.gammaReg = 0;                                       % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_WLS.filter = 'no';                                      % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_WLS Identification:
        results_WLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_WLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_WLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%    Finished IDIM-WLS    %%%%%%%%%%%%%%%%")
        
    end % end IDIM_WLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% IDIM and Weighted Least Square (IDIM-WLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_WLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%%%%     Starting IDIM-WLS_f     %%%%%%%%%%%%%")
        
        % Set IDIM_WLS_f Options:
        optionsIDIM_WLS_f.verbose = verboseFlag;                          	% If true, the solver communicates the identification error at each step.
        optionsIDIM_WLS_f.debug = debugFlag;                               	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_WLS_f.solver = 'backslash';                             % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsIDIM_WLS_f.alg = 'WLS';                                      % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_WLS_f.regularizerType = 'no';                           % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_WLS_f.gammaReg = 0;                                     % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_WLS_f.filter = 'butterworth';                           % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_WLS_f Identification:
        results_WLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_WLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_WLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%   Finished IDIM-WLS_f   %%%%%%%%%%%%%%%%")
        
    end % end IDIM_WLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Total Least Square (IDIM-TLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_TLS
        
        disp("%%%%%%%%%%%%%%%%    Starting IDIM-TLS    %%%%%%%%%%%%%%%")
        
        % Set IDIM_TLS Options:
        optionsIDIM_TLS.verbose = verboseFlag;                             	% If true, the solver communicates the identification error at each step.
        optionsIDIM_TLS.debug = debugFlag;                                  % If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_TLS.solver = 'svd_ridge';                               % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [svd]: compute TLS with svd, [recursive]: compute TLS using Gauss-Newton recursion.
        optionsIDIM_TLS.alg = 'TLS';                                        % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_TLS.filter = 'no';                                      % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Set IDIM_TLS Stop criteria:
        optionsIDIM_TLS.stopCrit.tol_1 = functionTolerance;
        optionsIDIM_TLS.stopCrit.Max_it = iterationMax;
        
        % Run IDIM_TLS Identification:
        results_TLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_TLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_TLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%    Finished IDIM-TLS    %%%%%%%%%%%%%%%")
        
    end % end IDIM_TLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Total Least Square (IDIM-TLS_f) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_TLS_f
        
        disp("%%%%%%%%%%%%%%%%   Starting IDIM-TLS_f   %%%%%%%%%%%%%%%")
        
        % Set IDIM_TLS Options:
        optionsIDIM_TLS_f.verbose = verboseFlag;                          	% If true, the solver communicates the identification error at each step.
        optionsIDIM_TLS_f.debug = debugFlag;                              	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_TLS_f.solver = 'svd_ridge';                             % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [svd]: compute TLS with svd, [recursive]: compute TLS using Gauss-Newton recursion.
        optionsIDIM_TLS_f.alg = 'TLS';                                    	% [LS]: classic Least Squares, [WLS]: Weighted Least Squares, [TLS]: Total Least Squares, [RR]: Ridge Regression, [WRR]: Weighted Ridge Regression
        optionsIDIM_TLS_f.filter = 'butterworth';                        	% [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Set IDIM_TLS Stop criteria:
        optionsIDIM_TLS_f.stopCrit.tol_1 = functionTolerance;
        optionsIDIM_TLS_f.stopCrit.Max_it = iterationMax;
        
        % Run IDIM_TLS Identification:
        results_TLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_TLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_TLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%   Finished IDIM-TLS_f   %%%%%%%%%%%%%%%")
        
    end % end IDIM_TLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Iteratively Reweighted Least Square (IDIM-IRLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_IRLS
        
        disp("%%%%%%%%%%%%%%%%    Starting IDIM-IRLS   %%%%%%%%%%%%%%%")
        
        % Set IDIM_WLS Options:
        optionsIDIM_IRLS.verbose = verboseFlag;                          	% If true, the solver communicates the identification error at each step.
        optionsIDIM_IRLS.debug = debugFlag;                                 % If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_IRLS.solver ='backslash';                               % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function
        optionsIDIM_IRLS.alg = 'IRLS';                                      % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_IRLS.regularizerType = 'no';                            % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_IRLS.gammaReg = 0;                                      % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_IRLS.filter = 'no';                                     % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_WLS Identification:
        results_IRLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_IRLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_IRLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%    Finished IDIM-IRLS   %%%%%%%%%%%%%%%%")
        
    end % end IDIM_WLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% IDIM and Iteratively Reweighted Least Square (IDIM-IRLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM_IRLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%%%%     Starting IDIM-IRLS_f    %%%%%%%%%%%%%")
        
        % Set IDIM_WLS_f Options:
        optionsIDIM_IRLS_f.verbose = verboseFlag;                          	% If true, the solver communicates the identification error at each step.
        optionsIDIM_IRLS_f.debug = debugFlag;                             	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_IRLS_f.solver = 'backslash';                            % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function
        optionsIDIM_IRLS_f.alg = 'IRLS';                                    % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_IRLS_f.regularizerType = 'no';                          % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_IRLS_f.gammaReg = 0;                                    % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_IRLS_f.filter = 'butterworth';                          % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_WLS_f Identification:
        results_IRLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_IRLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_IRLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%   Finished IDIM-IRLS_f  %%%%%%%%%%%%%%%%")
        
    end % end IDIM_IRLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Maximum Likelihood estimator:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % ML estimator
        
        disp("%%%%%%%%%%%%%%%%%%     Starting ML     %%%%%%%%%%%%%%%%%")
        
        % Set ML Options:
        optionsML.verbose = verboseFlag;                                    % If true, the solver communicates the identification error at each step.
        optionsML.debug = debugFlag;                                        % If true, the solver plots the robot trajectory using the computed parameters.
        optionsML.filter = 'no';                                            % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        optionsML.optimizer = 'lsqnonlin';                                  % [simplex], [lsqnonlin], [fmin], [pso] or [ga]
        
        % Set ML Stop criteria:
        optionsML.stopCrit.tol_1 = functionTolerance/1000;
        optionsML.stopCrit.tol_2 = stepTolerance/1000;
        optionsML.stopCrit.Max_it = iterationMax;
        
        % Run ML Identification:
        results_ML = run_ML(robot, benchmarkSettings, experimentDataStruct, optionsML, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_ML', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%%%   Finished ML   %%%%%%%%%%%%%%%%%%%%")
        
    end % end ML
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Maximum Likelihood estimator with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % ML_f estimator
        
        disp("%%%%%%%%%%%%%%%%%%    Starting ML_f    %%%%%%%%%%%%%%%%%")
        
        % Set ML Options:
        optionsML_f.verbose = verboseFlag;                                  % If true, the solver communicates the identification error at each step.
        optionsML_f.debug = debugFlag;                                      % If true, the solver plots the robot trajectory using the computed parameters.
        optionsML_f.filter = 'butterworth';                                 % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        optionsML_f.optimizer = 'lsqnonlin';                                % [simplex], [lsqnonlin], [fmin], [pso] or [ga]
        
        % Set ML Stop criteria:
        optionsML_f.stopCrit.tol_1 = functionTolerance/1000;
        optionsML_f.stopCrit.tol_2 = stepTolerance/1000;
        optionsML_f.stopCrit.Max_it = iterationMax;
        
        % Run ML Identification:
        results_ML_f = run_ML(robot, benchmarkSettings, experimentDataStruct, optionsML_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_ML_f', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%%%  Finished ML_f  %%%%%%%%%%%%%%%%%%%%")
        
    end % end ML_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Instrumental Variables (IDIM-IV):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % IDIM-IV
        
        disp("%%%%%%%%%%%%%%      Starting IDIM-IV      %%%%%%%%%%%%%%")
        
        % Set IDIM_IV Options:
        optionsIDIM_IV.verbose = verboseFlag;                            	% If true, the solver communicates the identification error at each step.
        optionsIDIM_IV.debug = debugFlag;                                 	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsIDIM_IV.solver = 'backslash';                                % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsIDIM_IV.alg = 'WLS';                                      	% [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsIDIM_IV.regularizerType = 'no';                              % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsIDIM_IV.gammaReg = 0;                                        % Regularization factor, to be multiplied to the regularizer.
        optionsIDIM_IV.filter = 'no';                                       % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Set IDIM_IV Stop criteria:
        optionsIDIM_IV.stopCrit.tol_1 = functionTolerance;
        optionsIDIM_IV.stopCrit.tol_2 = stepTolerance;
        optionsIDIM_IV.stopCrit.Max_it = iterationMax;
        
        % Run IDIM_IV Identification:
        results_IV = run_IDIM_IV(robot, benchmarkSettings, experimentDataStruct, optionsIDIM_IV, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_IV', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%    Finished IDIM-IV    %%%%%%%%%%%%%%%%")
        
    end % end IDIM_IV
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Direct and Inverse Dynamic Identification Model (DIDIM):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % DIDIM
        
        disp("%%%%%%%%%%%%%%%%%    Starting DIDIM    %%%%%%%%%%%%%%%%%")
        
        % Set DIDIM Options:
        optionsDIDIM.verbose = verboseFlag;                             	% If true, the solver communicates the identification error at each step.
        optionsDIDIM.debug = debugFlag;                                    	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsDIDIM.solver = 'backslash';                                  % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsDIDIM.alg = 'WLS';                                           % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsDIDIM.regularizerType = 'no';                                % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsDIDIM.gammaReg = 0;                                          % Regularization factor, to be multiplied to the regularizer.
        optionsDIDIM.filter = 'no';                                         % [no]: no filter, [butterworth]: zero-shift butterworth filter
        
        % Set DIDIM Stop criteria:
        optionsDIDIM.stopCrit.tol_1 = functionTolerance;
        optionsDIDIM.stopCrit.tol_2 = stepTolerance;
        optionsDIDIM.stopCrit.Max_it = iterationMax;
        
        % Run DIDIM Identification:
        results_DIDIM = run_DIDIM(robot, benchmarkSettings, experimentDataStruct, optionsDIDIM, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_DIDIM', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%    Finished DIDIM    %%%%%%%%%%%%%%%%%")
        
    end % end DIDIM
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Closed-Loop Input-Error (CLIE) with joint torque input:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % CLIE
        
        disp("%%%%%%%%%%%%%%%%     Starting CLIE     %%%%%%%%%%%%%%%%%")
        
        % Set CLIE Options:
        optionsCLIE.verbose = verboseFlag;                                  % If true, the solver communicates the identification error at each step.
        optionsCLIE.debug = debugFlag;                                  	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsCLIE.isParamWise = false;                                    % If true, run the parameterwise identification, else identify all the parameters at the same time
        optionsCLIE.paramwiseOptimizer = 'lsqnonlin';                       % [simplex] or [lsqnonlin]
        optionsCLIE.globalOptimizer = 'lsqnonlin';                          % [simplex], [lsqnonlin], [pso] or [ga]
        optionsCLIE.filter = 'no';                                          % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Set CLIE Stop criteria:
        optionsCLIE.stopCrit.tol_1 = functionTolerance/1000;
        optionsCLIE.stopCrit.tol_2 = stepTolerance/1000;
        optionsCLIE.stopCrit.Max_it = iterationMax;
        
        % Run CLIE Identification:
        results_CLIE = run_CLIE(robot, benchmarkSettings, experimentDataStruct, optionsCLIE, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_CLIE', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%     Finished CLIE     %%%%%%%%%%%%%%%%")
        
    end % end CLIE
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Closed-Loop Output-Error (CLOE) with joint position input:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % CLOE
        
        disp("%%%%%%%%%%%%%%%%     Starting CLOE     %%%%%%%%%%%%%%%%%")
        
        % Set CLOE Options:
        optionsCLOE.verbose = verboseFlag;                                  % If true, the solver communicates the identification error at each step.
        optionsCLOE.debug = debugFlag;                                      % If true, the solver plots the robot trajectory using the computed parameters.
        optionsCLOE.isParamWise = false;                                    % If true, run the parameterwise identification, else identify all the parameters at the same time
        optionsCLOE.paramwiseOptimizer = 'lsqnonlin';                       % [simplex] or [lsqnonlin]
        optionsCLOE.globalOptimizer = 'lsqnonlin';                          % [simplex], [lsqnonlin], [pso] or [ga]
        optionsCLOE.filter = 'no';                                          % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        optionsCLOE.errorFunction = 'Q';                                    % [Q] for joint position error function, [Qp] for joint velocity error function and [Qpp] for joint acceleration error function
        
        % Set CLOE Stop criteria:
        optionsCLOE.stopCrit.tol_1 = functionTolerance/1000;
        optionsCLOE.stopCrit.tol_2 = stepTolerance/1000;
        optionsCLOE.stopCrit.Max_it = iterationMax;
        
        % Run CLOE Identification:
        results_CLOE = run_CLOE(robot, benchmarkSettings, experimentDataStruct, optionsCLOE, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_CLOE', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%     Finished CLOE     %%%%%%%%%%%%%%%%")
        
    end % end CLOE
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Extended Kalman Filter (EKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % EKF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting EKF   %%%%%%%%%%%%%%%%%%%")
        
        % Set EKF Options:
        optionsEKF.useComputedTorque = false;                               % If true, run the kalman filter in CLOE mode
        optionsEKF.verbose = verboseFlag;                                  	% If true, the solver communicates the identification error at each step.
        optionsEKF.debug = debugFlag;                                   	% If true, the solver plots the parameter evolution.
        optionsEKF.getIterationData = false;                                % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsEKF.anneal = true;                                           % If true, anneals the process noise covariance of the parameters with time.
        optionsEKF.samplingFactor = 100;
        optionsEKF.type = 'ekf';
        
        % Run EKF Identification:
        results_EKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsEKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_EKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished EKF    %%%%%%%%%%%%%%%%%%")
        
    end % end EKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Square Root Extended Kalman Filter (SREKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SREKF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting SREKF   %%%%%%%%%%%%%%%%%%%")
        
        % Set SREKF Options:
        optionsSREKF.useComputedTorque = false;                             % If true, run the kalman filter in CLOE mode
        optionsSREKF.verbose = verboseFlag;                                 % If true, the solver communicates the identification error at each step.
        optionsSREKF.debug = debugFlag;                                     % If true, the solver plots the parameter evolution.
        optionsSREKF.getIterationData = false;                          	% If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSREKF.anneal = true;                                       	% If true, anneals the process noise covariance of the parameters with time.
        optionsSREKF.samplingFactor = 100;
        optionsSREKF.type = 'srekf';
        
        % Run SREKF Identification:
        results_SREKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSREKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SREKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished SREKF    %%%%%%%%%%%%%%%%%%")
        
    end % end SREKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Unscented Kalman Filter (UKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % UKF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting UKF    %%%%%%%%%%%%%%%%%%")
        
        % Set UKF Options:
        optionsUKF.useComputedTorque = false;                               % If true, run the kalman filter in CLOE mode
        optionsUKF.verbose = verboseFlag;                                   % If true, the solver communicates the identification error at each step.
        optionsUKF.debug = debugFlag;                                       % If true, the solver plots the parameter evolution.
        optionsUKF.getIterationData = false;                                % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsUKF.anneal = true;                                           % If true, anneals the process noise covariance of the parameters with time.
        optionsUKF.samplingFactor = 100;
        optionsUKF.type = 'ukf';
        optionsUKF.sigmaCompute = 'svd';                                    % [svd]: compute matrix square root using svd, [chol]: compute matrix square root using Cholesky factorization.
        
        % Run UKF Identification:
        results_UKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsUKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_UKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished UKF    %%%%%%%%%%%%%%%%%%")
        
    end % end UKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Square Root Unscented Kalman Filter (SRUKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SRUKF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting SRUKF  %%%%%%%%%%%%%%%%%%")
        
        % Set SRUKF Options:
        optionsSRUKF.useComputedTorque = false;                             % If true, run the kalman filter in CLOE mode
        optionsSRUKF.verbose = verboseFlag;                                	% If true, the solver communicates the identification error at each step.
        optionsSRUKF.debug = debugFlag;                                   	% If true, the solver plots the parameter evolution.
        optionsSRUKF.getIterationData = false;                              % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSRUKF.anneal = true;                                         % If true, anneals the process noise covariance of the parameters with time.
        optionsSRUKF.samplingFactor = 100;
        optionsSRUKF.type = 'srukf';
        
        % Run SRUKF Identification:
        results_SRUKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSRUKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SRUKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished SRUKF  %%%%%%%%%%%%%%%%%%")
        
    end % end SRUKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Central Difference Kalman Filter (CDKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % CDKF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting CDKF   %%%%%%%%%%%%%%%%%%")
        
        % Set CDKF Options:
        optionsCDKF.useComputedTorque = false;                              % If true, run the kalman filter in CLOE mode
        optionsCDKF.verbose = verboseFlag;                                  % If true, the solver communicates the identification error at each step.
        optionsCDKF.debug = debugFlag;                                      % If true, the solver plots the parameter evolution.
        optionsCDKF.getIterationData = false;                               % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsCDKF.anneal = true;                                          % If true, anneals the process noise covariance of the parameters with time.
        optionsCDKF.samplingFactor = 100;
        optionsCDKF.type = 'cdkf';
        optionsCDKF.sigmaCompute = 'svd';                                	% [svd]: compute matrix square root using svd, [chol]: compute matrix square root using Cholesky factorization.
        
        % Run CDKF Identification:
        results_CDKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsCDKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_CDKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished CDKF   %%%%%%%%%%%%%%%%%%")
        
    end % end CDKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Square Root Central Difference Kalman Filter (SRCDKF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SRCDKF
        disp("%%%%%%%%%%%%%%%%%%    Starting SRCDKF %%%%%%%%%%%%%%%%%%")
        
        % Set SRCDKF Options:
        optionsSRCDKF.useComputedTorque = false;                            % If true, run the kalman filter in CLOE mode
        optionsSRCDKF.verbose = verboseFlag;                                % If true, the solver communicates the identification error at each step.
        optionsSRCDKF.debug = debugFlag;                                  	% If true, the solver plots the parameter evolution.
        optionsSRCDKF.getIterationData = false;                             % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSRCDKF.anneal = true;                                        % If true, anneals the process noise covariance of the parameters with time.
        optionsSRCDKF.samplingFactor = 100;
        optionsSRCDKF.type = 'srcdkf';
        
        % Run SRCDKF Identification:
        results_SRCDKF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSRCDKF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SRCDKF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished SRCDKF %%%%%%%%%%%%%%%%%%")
        
    end % end SRCDKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Particle Filter (PF):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PF
        
        disp("%%%%%%%%%%%%%%%%%%    Starting PF     %%%%%%%%%%%%%%%%%%")
        
        % Set PF Options:
        optionsPF.useComputedTorque = false;                                % If true, run the paticle filter in CLOE mode
        optionsPF.verbose = verboseFlag;                                    % If true, the solver communicates the identification error at each step.
        optionsPF.debug = debugFlag;                                        % If true, the solver plots the parameter evolution.
        optionsPF.getIterationData = false;                                 % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsPF.samplingFactor = 100;                                     % How often is the debug information displayed
        optionsPF.anneal = true;                                            % If true, anneals the process noise covariance of the parameters with time.
        optionsPF.nbParticules = 25*robot.paramVectorSize;
        optionsPF.initialBias = 5e-2;
        optionsPF.type = 'pf';
        optionsPF.resampleThreshold = 1e-2;                                  % Resampling occurs each resampleThreshold iterations
        
        % Run PF Identification:
        results_PF = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsPF, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PF', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%    Finished PF     %%%%%%%%%%%%%%%%%%")
        
    end % end PF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Extended Kalman Filter with computed torque (EKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % EKF_ct
        
        disp("%%%%%%%%%%%%%%%%%%  Starting EKF_ct  %%%%%%%%%%%%%%%%%%%")
        
        % Set EKF_ct Options:
        optionsEKF_ct.useComputedTorque = true;                             % If true, run the kalman filter in CLOE mode
        optionsEKF_ct.verbose = verboseFlag;                                % If true, the solver communicates the identification error at each step.
        optionsEKF_ct.debug = debugFlag;                                   	% If true, the solver plots the parameter evolution.
        optionsEKF_ct.getIterationData = false;                             % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsEKF_ct.anneal = true;                                        % If true, anneals the process noise covariance of the parameters with time.
        optionsEKF_ct.samplingFactor = 100;
        optionsEKF_ct.type = 'ekf';
        
        % Run EKF_ct Identification:
        results_EKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsEKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_EKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%   Finished EKF_ct  %%%%%%%%%%%%%%%%%%")
        
    end % end EKF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Square Root Extended Kalman Filter with computed torque (SREKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SREKF_ct
        
        disp("%%%%%%%%%%%%%%%%%%  Starting SREKF_ct  %%%%%%%%%%%%%%%%%%%")
        
        % Set SREKF_ct Options:
        optionsSREKF_ct.useComputedTorque = true;                           % If true, run the kalman filter in CLOE mode
        optionsSREKF_ct.verbose = verboseFlag;                              % If true, the solver communicates the identification error at each step.
        optionsSREKF_ct.debug = debugFlag;                                  % If true, the solver plots the parameter evolution.
        optionsSREKF_ct.getIterationData = false;                          	% If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSREKF_ct.anneal = true;                                      % If true, anneals the process noise covariance of the parameters with time.
        optionsSREKF_ct.samplingFactor = 100;
        optionsSREKF_ct.type = 'srekf';
        
        % Run SREKF_ct Identification:
        results_SREKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSREKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SREKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%   Finished SREKF_ct  %%%%%%%%%%%%%%%%%%")
        
    end % end SREKF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Unscented Kalman Filter with computed torque (UKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % UKF_ct
        
        disp("%%%%%%%%%%%%%%%%%%   Starting UKF_ct  %%%%%%%%%%%%%%%%%%")
        
        % Set UKF_ct Options:
        optionsUKF_ct.useComputedTorque = true;                             % If true, run the kalman filter in CLOE mode
        optionsUKF_ct.verbose = verboseFlag;                                % If true, the solver communicates the identification error at each step.
        optionsUKF_ct.debug = debugFlag;                                    % If true, the solver plots the parameter evolution.
        optionsUKF_ct.getIterationData = false;                             % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsUKF_ct.anneal = true;                                        % If true, anneals the process noise covariance of the parameters with time.
        optionsUKF_ct.samplingFactor = 100;
        optionsUKF_ct.type = 'ukf';
        optionsUKF_ct.sigmaCompute = 'svd';                                 % [svd]: compute matrix square root using svd, [chol]: compute matrix square root using Cholesky factorization.
        
        % Run UKF_ct Identification:
        results_UKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsUKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_UKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%   Finished UKF_ct  %%%%%%%%%%%%%%%%%%")
        
    end % end UKF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Square Root Unscented Kalman Filter with computed torque (SRUKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SRUKF_ct
        
        disp("%%%%%%%%%%%%%%%%%%  Starting SRUKF_ct %%%%%%%%%%%%%%%%%%")
        
        % Set SRUKF_ct Options:
        optionsSRUKF_ct.useComputedTorque = true;                           % If true, run the kalman filter in CLOE mode
        optionsSRUKF_ct.verbose = verboseFlag;                            	% If true, the solver communicates the identification error at each step.
        optionsSRUKF_ct.debug = debugFlag;                                  % If true, the solver plots the parameter evolution.
        optionsSRUKF_ct.getIterationData = false;                           % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSRUKF_ct.anneal = true;                                      % If true, anneals the process noise covariance of the parameters with time.
        optionsSRUKF_ct.samplingFactor = 100;
        optionsSRUKF_ct.type = 'srukf';
        
        % Run SRUKF_ct Identification:
        results_SRUKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSRUKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SRUKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%  Finished SRUKF_ct %%%%%%%%%%%%%%%%%%")
        
    end % end SRUKF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Central Difference Kalman Filter with computed torque (CDKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % CDKF_ct
        
        disp("%%%%%%%%%%%%%%%%%%  Starting CDKF_ct  %%%%%%%%%%%%%%%%%%")
        
        % Set CDKF_ct Options:
        optionsCDKF_ct.useComputedTorque = true;                            % If true, run the kalman filter in CLOE mode
        optionsCDKF_ct.verbose = verboseFlag;                               % If true, the solver communicates the identification error at each step.
        optionsCDKF_ct.debug = debugFlag;                                   % If true, the solver plots the parameter evolution.
        optionsCDKF_ct.getIterationData = false;                            % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsCDKF_ct.anneal = true;                                       % If true, anneals the process noise covariance of the parameters with time.
        optionsCDKF_ct.samplingFactor = 100;
        optionsCDKF_ct.type = 'cdkf';
        optionsCDKF_ct.sigmaCompute = 'svd';                                % [svd]: compute matrix square root using svd, [chol]: compute matrix square root using Cholesky factorization.
        
        % Run CDKF_ct Identification:
        results_CDKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsCDKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_CDKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%  Finished CDKF_ct  %%%%%%%%%%%%%%%%%%")
        
    end % end CDKF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% SR Central Difference Kalman Filter with computed torque (SRCDKF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % SRCDKF_ct
        disp("%%%%%%%%%%%%%%%%%% Starting SRCDKF_ct %%%%%%%%%%%%%%%%%%")
        
        % Set SRCDKF_ct Options:
        optionsSRCDKF_ct.useComputedTorque = true;                          % If true, run the kalman filter in CLOE mode
        optionsSRCDKF_ct.verbose = verboseFlag;                             % If true, the solver communicates the identification error at each step.
        optionsSRCDKF_ct.debug = debugFlag;                                 % If true, the solver plots the parameter evolution.
        optionsSRCDKF_ct.getIterationData = false;                          % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsSRCDKF_ct.anneal = true;                                     % If true, anneals the process noise covariance of the parameters with time.
        optionsSRCDKF_ct.samplingFactor = 100;
        optionsSRCDKF_ct.type = 'srcdkf';
        
        % Run SRCDKF_ct Identification:
        results_SRCDKF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsSRCDKF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_SRCDKF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%% Finished SRCDKF_ct %%%%%%%%%%%%%%%%%%")
        
    end % end SRCDKF
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Particle Filter with computed torque (PF_ct):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PF_ct
        
        disp("%%%%%%%%%%%%%%%%%%   Starting PF_ct   %%%%%%%%%%%%%%%%%%")
        
        % Set PF_ct Options:
        optionsPF_ct.useComputedTorque = true;                              % If true, run the paticle filter in CLOE mode
        optionsPF_ct.verbose = verboseFlag;                                 % If true, the solver communicates the identification error at each step.
        optionsPF_ct.debug = debugFlag;                                     % If true, the solver plots the parameter evolution.
        optionsPF_ct.getIterationData = false;                              % If true, saves the augmented state estimate and its covariance matrix each samplingFactor iteration. Produces MASSIVE data files !
        optionsPF_ct.samplingFactor = 100;                                  % How often is the debug information displayed
        optionsPF_ct.anneal = true;                                         % If true, anneals the process noise covariance of the parameters with time.
        optionsPF_ct.nbParticules = 25*robot.paramVectorSize;
        optionsPF_ct.initialBias = 5e-2;
        optionsPF_ct.type = 'pf';
        optionsPF_ct.resampleThreshold = 1e-2;                              % Resampling occurs each resampleThreshold iterations
        
        % Run PF_ct Identification:
        results_PF_ct = run_Kalman(robot, benchmarkSettings, experimentDataStruct, optionsPF_ct, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PF_ct', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%%%%   Finished PF_ct   %%%%%%%%%%%%%%%%%%")
        
    end % end PF_ct
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Adaline Neural Network:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % ANN
        
        disp("%%%%%%%%%    Starting Adaline Neural Network   %%%%%%%%%")
        
        % Set ANN Options:
        optionsANN.verbose = verboseFlag;                                   % If true, the solver communicates the identification error at each step.
        optionsANN.debug = debugFlag;                                       % If true, the solver plots the robot trajectory using the computed parameters.
        optionsANN.filter = 'no';                                           % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        optionsANN.neuronet = 'stochastic_grad';                            % [toolbox]: Use matlab neural network toolbox, [grad]: Use custom gradient descent, [stochastic_grad]: Use custom stochastic gradient descent.
        optionsANN.learningRate = 1;
        
        % Set ANN Stop criteria:
        optionsANN.stopCrit.tol = 1e-5;
        optionsANN.stopCrit.Max_training_epochs = 1e4;
        
        % Run ANN Identification:
        results_ANN = run_ANN(robot, benchmarkSettings, experimentDataStruct, optionsANN, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_ANN', '-v7.3')
        
        disp("%%%%%%%%%    Finished Adaline Neural Network   %%%%%%%%%")
        
    end % end ANN
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Filtered Adaline Neural Network:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % ANN_f
        
        disp("%%%%%%%%%    Starting Adaline Neural Network   %%%%%%%%%")
        
        % Set ANN Options:
        optionsANN_f.verbose = verboseFlag;                                 % If true, the solver communicates the identification error at each step.
        optionsANN_f.debug = debugFlag;                                     % If true, the solver plots the robot trajectory using the computed parameters.
        optionsANN_f.filter = 'butterworth';                                % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        optionsANN_f.neuronet = 'stochastic_grad';                          % [toolbox]: Use matlab neural network toolbox, [grad]: Use custom gradient descent, [stochastic_grad]: Use custom stochastic gradient descent.
        optionsANN_f.learningRate = 1;
        
        % Set ANN Stop criteria:
        optionsANN_f.stopCrit.tol = 1e-5;
        optionsANN_f.stopCrit.Max_training_epochs = 1e4;
        
        % Run ANN Identification:
        results_ANN_f = run_ANN(robot, benchmarkSettings, experimentDataStruct, optionsANN_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_ANN_f', '-v7.3')
        
        disp("%%%%%%%%%    Finished Adaline Neural Network   %%%%%%%%%")
        
    end % end ANN_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Hopfield Neural Network:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % HTRNN
        
        disp("%%%%%%%%    Starting Hopfield Neural Network    %%%%%%%%")
        
        % Set HTRNN Options:
        optionsHTRNN.verbose = verboseFlag;                                 % If true, the solver communicates the identification error at each step.
        optionsHTRNN.debug = debugFlag;                                     % If true, the solver plots the robot trajectory using the computed parameters.
        optionsHTRNN.filter = 'no';                                         % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        if strcmp(robotName,'RV2SQ')
            optionsHTRNN.learningRate = 1e-6; % real TX40: 3e-8; % 1e-7 for TX40 1e-6 for RV2SQ;
        elseif strcmp(robotName,'TX40')
            optionsHTRNN.learningRate = 3e-8;
        else
            optionsHTRNN.learningRate = 1e-7;
        end
        
        % Set HTRNN Stop criteria:
        optionsHTRNN.stopCrit.tol = 1e-5;
        optionsHTRNN.stopCrit.Max_training_epochs = 1e4;
        
        % Run HTRNN Identification:
        results_HTRNN = run_HTRNN(robot, benchmarkSettings, experimentDataStruct, optionsHTRNN, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_HTRNN', '-v7.3')
        
        disp("%%%%%%%%    Finished Hopfield Neural Network    %%%%%%%%")
        
    end % end HTRNN
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Filtered Hopfield Neural Network:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % HTRNN_f
        
        disp("%%%%%%%%    Starting Hopfield Neural Network    %%%%%%%%")
        
        % Set HTRNN Options:
        optionsHTRNN_f.verbose = verboseFlag;                               % If true, the solver communicates the identification error at each step.
        optionsHTRNN_f.debug = debugFlag;                                   % If true, the solver plots the robot trajectory using the computed parameters.
        optionsHTRNN_f.filter = 'butterworth';                              % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        if strcmp(robotName,'RV2SQ')
            optionsHTRNN_f.learningRate = 1e-6; % real TX40: 3e-8; % 1e-7 for TX40 1e-6 for RV2SQ;
        elseif strcmp(robotName,'TX40')
            optionsHTRNN_f.learningRate = 3e-8;
        else
            optionsHTRNN_f.learningRate = 1e-7;
        end
        
        % Set HTRNN Stop criteria:
        optionsHTRNN_f.stopCrit.tol = 1e-5;
        optionsHTRNN_f.stopCrit.Max_training_epochs = 1e4;
        
        % Run HTRNN Identification:
        results_HTRNN_f = run_HTRNN(robot, benchmarkSettings, experimentDataStruct, optionsHTRNN_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_HTRNN_f', '-v7.3')
        
        disp("%%%%%%%%    Finished Hopfield Neural Network    %%%%%%%%")
        
    end % end HTRNN
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS (PC-IDIM-OLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS
        
        disp("%%%%%%%%%%%%%    Starting PC-IDIM-OLS     %%%%%%%%%%%%%%")
        
        % Set IDIM_OLS Options:
        optionsPC_IDIM_OLS.verbose = verboseFlag;                           % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS.debug = debugFlag;                               % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS.solver ='mosek';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS.alg = 'PC-OLS';                                  % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS.regularizerType = 'no';                          % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS.gammaReg = 0;                                    % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS.filter = 'no';                                   % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_OLS Identification:
        results_PC_OLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%    Finished PC-IDIM-OLS     %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_OLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS (PC-IDIM-OLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%     Starting PC-IDIM-OLS-f      %%%%%%%%%%%%")
        
        % Set PC_IDIM_OLS_f Options:
        optionsPC_IDIM_OLS_f.verbose = verboseFlag;                         % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_f.debug = debugFlag;                             % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_f.solver ='mosek';                               % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_f.alg = 'PC-OLS';                                % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_f.regularizerType = 'no';                        % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_f.gammaReg = 0;                                  % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_f.filter = 'butterworth';                        % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter

        % Run PC_ IDIM_OLS_f Identification:
        results_PC_OLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%    Finished PC-IDIM-OLS-f    %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_OLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-WLS (PC-IDIM-WLS):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_WLS
        
        disp("%%%%%%%%%%%%%    Starting PC-IDIM-WLS     %%%%%%%%%%%%%%")
        
        % Set IDIM_WLS Options:
        optionsPC_IDIM_WLS.verbose = verboseFlag;                           % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_WLS.debug = debugFlag;                               % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_WLS.solver ='mosek';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_WLS.alg = 'PC-WLS';                                  % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_WLS.regularizerType = 'no';                          % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_WLS.gammaReg = 0;                                    % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_WLS.filter = 'no';                                   % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run IDIM_WLS Identification:
        results_PC_WLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_WLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_WLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%    Finished PC-IDIM-WLS     %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_WLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-WLS (PC-IDIM-WLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_WLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%     Starting PC-IDIM-WLS-f      %%%%%%%%%%%%")
        
        % Set PC_IDIM_WLS_f Options:
        optionsPC_IDIM_WLS_f.verbose = verboseFlag;                         % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_WLS_f.debug = debugFlag;                             % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_WLS_f.solver ='mosek';                               % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_WLS_f.alg = 'PC-WLS';                                % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_WLS_f.regularizerType = 'no';                        % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_WLS_f.gammaReg = 0;                                  % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_WLS_f.filter = 'butterworth';                        % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter

        % Run PC_IDIM_WLS_f Identification:
        results_PC_WLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_WLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_WLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%    Finished PC-IDIM-WLS-f    %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_WLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-IRLS (PC-IDIM-IRLS)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_IRLS
        
        disp("%%%%%%%%%%%%%    Starting PC-IDIM-IRLS    %%%%%%%%%%%%%%")
        
        % Set IDIM_IRLS Options:
        optionsPC_IDIM_IRLS.verbose = verboseFlag;                        	% If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_IRLS.debug = debugFlag;                          	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_IRLS.solver ='mosek';                                % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_IRLS.alg = 'PC-IRLS';                                % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_IRLS.regularizerType = 'no';                         % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_IRLS.gammaReg = 0;                                   % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_IRLS.filter = 'no';                                	% [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_IRLS Identification:
        results_PC_IRLS = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_IRLS, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_IRLS', '-v7.3')
        
        disp("%%%%%%%%%%%%%    Finished PC-IDIM-IRLS    %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_IRLS
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-IRLS (PC-IDIM-IRLS) with Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_IRLS with 2-pass-Butterworth filter
        
        disp("%%%%%%%%%%%     Starting PC-IDIM-IRLS-f     %%%%%%%%%%%%")
        
        % Set PC_IDIM_IRLS_f Options:
        optionsPC_IDIM_IRLS_f.verbose = verboseFlag;                       	% If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_IRLS_f.debug = debugFlag;                          	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_IRLS_f.solver ='mosek';                            	% [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_IRLS_f.alg = 'PC-IRLS';                           	% [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_IRLS_f.regularizerType = 'no';                    	% [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_IRLS_f.gammaReg = 0;                             	% Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_IRLS_f.filter = 'butterworth';                     	% [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_IRLS_f Identification:
        results_PC_IRLS_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_IRLS_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_IRLS_f', '-v7.3')
        
        disp("%%%%%%%%%%%%   Finished PC-IDIM-IRLS-f    %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_IRLS_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with Euclidean Regularization:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS_Euclidean
        
        disp("%%%%%%%    Starting PC-IDIM-OLS-Euclidean   %%%%%%%")
        
        % Set PC_IDIM_OLS_Euclidean Options:
        optionsPC_IDIM_OLS_Euclidean.verbose = verboseFlag;                 % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_Euclidean.debug = debugFlag;                     % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_Euclidean.solver ='mosek';                       % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_Euclidean.alg = 'PC-OLS';                        % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_Euclidean.regularizerType = 'Euclidean';         % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_Euclidean.gammaReg = gammaReg;                   % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_Euclidean.filter = 'no';                         % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_OLS_Euclidean Identification:
        results_PC_OLS_Euclidean = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_Euclidean, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_Euclidean', '-v7.3')
        
        disp("%%%%%%%    Finished PC-IDIM-OLS-Euclidean   %%%%%%%")
        
    end % end PC_IDIM_OLS_Euclidean
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with Euclidean Regularization and Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 %PC_IDIM_OLS_Euclidean with 2-pass-Butterworth filter
        
        disp("%%%%%     Starting PC-IDIM-OLS-Euclidean-f    %%%%%")
        
        % Set PC_IDIM_OLS_Euclidean_f Options:
        optionsPC_IDIM_OLS_Euclidean_f.verbose = verboseFlag;               % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_Euclidean_f.debug = debugFlag;                   % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_Euclidean_f.solver ='mosek';                     % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_Euclidean_f.alg = 'PC-OLS';                      % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_Euclidean_f.regularizerType = 'Euclidean';       % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_Euclidean_f.gammaReg = gammaReg;                 % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_Euclidean_f.filter = 'butterworth';              % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_OLS_Euclidean_f Identification:
        results_PC_OLS_Euclidean_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_Euclidean_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_Euclidean_f', '-v7.3')
        
        disp("%%%%%%   Finished PC-IDIM-OLS-Euclidean-f   %%%%%%%")
        
    end % end PC_IDIM_OLS_Euclidean_f
    
        methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with Entropic Regularization:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS_Entropic
        
        disp("%%%%%%%%%    Starting PC-IDIM-OLS-Entropic    %%%%%%%%%")
        
        % Set PC_IDIM_OLS_Entropic Options:
        optionsPC_IDIM_OLS_Entropic.verbose = verboseFlag;                  % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_Entropic.debug = debugFlag;                      % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_Entropic.solver ='mosek';                        % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_Entropic.alg = 'PC-OLS';                         % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_Entropic.regularizerType = 'Entropic';           % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_Entropic.gammaReg = gammaReg;                    % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_Entropic.filter = 'no';                          % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_OLS_Entropic Identification:
        results_PC_OLS_Entropic = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_Entropic, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_Entropic', '-v7.3')
        
        disp("%%%%%%%%%    Finished PC-IDIM-OLS-Entropic    %%%%%%%%%")
        
    end % end PC_IDIM_OLS_Entropic
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with Entropic Regularization and Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS_Entropic with 2-pass-Butterworth filter
        
        disp("%%%%%%%     Starting PC-IDIM-OLS-Entropic-f     %%%%%%%")
        
        % Set PC_IDIM_OLS_Entropic_f Options:
        optionsPC_IDIM_OLS_Entropic_f.verbose = verboseFlag;                % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_Entropic_f.debug = debugFlag;                    % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_Entropic_f.solver ='mosek';                      % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_Entropic_f.alg = 'PC-OLS';                       % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_Entropic_f.regularizerType = 'Entropic';         % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_Entropic_f.gammaReg = gammaReg;                  % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_Entropic_f.filter = 'butterworth';               % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter

        % Run PC_IDIM_OLS_Entropic_f Identification:
        results_PC_OLS_Entropic_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_Entropic_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_Entropic_f', '-v7.3')
        
        disp("%%%%%%%%    Finished IDIM-PC-OLS-Entropic-f   %%%%%%%%%")
        
    end % end PC_IDIM_OLS_Entropic_f
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with ConstPullback Regularization:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS_ConstPullback
        
        disp("%%%%%%%    Starting PC-IDIM-OLS-ConstPullback   %%%%%%%")
        
        % Set PC_IDIM_OLS_ConstPullback Options:
        optionsPC_IDIM_OLS_ConstPullback.verbose = verboseFlag;             % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_ConstPullback.debug = debugFlag;                 % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_ConstPullback.solver ='mosek';                   % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_ConstPullback.alg = 'PC-OLS';                    % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_ConstPullback.regularizerType = 'ConstPullback'; % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_ConstPullback.gammaReg = gammaReg;               % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_ConstPullback.filter = 'no';                     % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_OLS_ConstPullback Identification:
        results_PC_OLS_ConstPullback = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_ConstPullback, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_ConstPullback', '-v7.3')
        
        disp("%%%%%%%    Finished PC-IDIM-OLS-ConstPullback   %%%%%%%")
        
    end % end PC_IDIM_OLS_ConstPullback
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Physically Consistent IDIM-OLS with ConstPullback Regularization and Zero-shift Butterworth filter:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM_OLS_ConstPullback with 2-pass-Butterworth filter
        
        disp("%%%%%     Starting PC-IDIM-OLS-ConstPullback-f    %%%%%")
        
        % Set PC_IDIM_OLS_ConstPullback_f Options:
        optionsPC_IDIM_OLS_ConstPullback_f.verbose = verboseFlag;           % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_OLS_ConstPullback_f.debug = debugFlag;               % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_OLS_ConstPullback_f.solver ='mosek';                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_OLS_ConstPullback_f.alg = 'PC-OLS';                  % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_OLS_ConstPullback_f.regularizerType = 'ConstPullback'; % [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_OLS_ConstPullback_f.gammaReg = gammaReg;             % Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_OLS_ConstPullback_f.filter = 'butterworth';          % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Run PC_IDIM_OLS_ConstPullback_f Identification:
        results_PC_OLS_ConstPullback_f = run_IDIM_LS(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_OLS_ConstPullback_f, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_OLS_ConstPullback_f', '-v7.3')
        
        disp("%%%%%%   Finished PC-IDIM-OLS-ConstPullback-f   %%%%%%%")
        
    end % end PC_IDIM_OLS_ConstPullback_f
    
        methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Inverse Dynamic Identification Model and Instrumental Variables (IDIM-IV) + LMI:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_IDIM-IV
        
        disp("%%%%%%%%%%%%%%    Starting PC-IDIM-IV    %%%%%%%%%%%%%%")
        
        % Set PC_IDIM_IV Options:
        optionsPC_IDIM_IV.verbose = verboseFlag;                            % If true, the solver communicates the identification error at each step.
        optionsPC_IDIM_IV.debug = debugFlag;                                % If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_IDIM_IV.solver = 'mosek';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_IDIM_IV.alg = 'PC-WLS';                                   % [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_IDIM_IV.regularizerType = 'no';                          	% [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_IDIM_IV.gammaReg = 0;                                   	% Regularization factor, to be multiplied to the regularizer.
        optionsPC_IDIM_IV.filter = 'no';                                  	% [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        
        % Set PC_IDIM_IV Stop criteria:
        optionsPC_IDIM_IV.stopCrit.tol_1 = functionTolerance;
        optionsPC_IDIM_IV.stopCrit.tol_2 = stepTolerance;
        optionsPC_IDIM_IV.stopCrit.Max_it = iterationMax;
        
        % Run PC_IDIM_IV Identification:
        results_PC_IV = run_IDIM_IV(robot, benchmarkSettings, experimentDataStruct, optionsPC_IDIM_IV, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_IV', '-v7.3')
        
        disp("%%%%%%%%%%%%%%    Finished PC-IDIM-IV    %%%%%%%%%%%%%%")
        
    end % end PC_IDIM_IV
    
    methodIndex=methodIndex+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Direct and Inverse Dynamic Identification Model (DIDIM) + LMI:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if identificationMethods.isActivated(methodIndex)==1 % PC_DIDIM 
        
        disp("%%%%%%%%%%%%%%%    Starting PC-DIDIM    %%%%%%%%%%%%%%%")
        
        % Set PC_DIDIM Options:
        optionsPC_DIDIM.verbose = verboseFlag;                             	% If true, the solver communicates the identification error at each step.
        optionsPC_DIDIM.debug = debugFlag;                              	% If true, the solver plots the robot trajectory using the computed parameters.
        optionsPC_DIDIM.solver = 'mosek';                                   % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function, [cvx]: use the cvx solver, [mosek]: use the cvx+mosek solver
        optionsPC_DIDIM.alg = 'PC-WLS';                                    	% [OLS]: Ordinary LS, [WLS]: Weighted LS, [TLS]: Total LS, [IRLS]: Iteratively Reweighted LS, [PC-OLS]: Physically Consistent OLS, [PC-WLS]: Physically Consistent WLS, [PC-IRLS]: Physically Consistent IRLS
        optionsPC_DIDIM.regularizerType = 'no';                         	% [no]: no regularizer, [Euclidean]: Euclidian regularization, [Entropic]: Entropic divergence regularization, [ConstPullback]: Constant Pullback distance regularization
        optionsPC_DIDIM.gammaReg = 0;                                     	% Regularization factor, to be multiplied to the regularizer.
        optionsPC_DIDIM.filter = 'no';                                     	% [no]: no filter, [butterworth]: zero-shift butterworth filter
        
        % Set PC_DIDIM Stop criteria:
        optionsPC_DIDIM.stopCrit.tol_1 = functionTolerance;
        optionsPC_DIDIM.stopCrit.tol_2 = stepTolerance;
        optionsPC_DIDIM.stopCrit.Max_it = iterationMax;
        
        % Run PC_DIDIM Identification:
        results_PC_DIDIM = run_DIDIM(robot, benchmarkSettings, experimentDataStruct, optionsPC_DIDIM, progressBar);
        
        % Save the Results:
        if ~exist(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate), 'dir')
            mkdir(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d', robot.name,noiseLevel,benchmarkSettings.decimRate));
        end
        save(sprintf('Benchmark/Robot_Identification_Results/%s_%s/decim_%d/results_%s_%s.mat',robot.name,noiseLevel,benchmarkSettings.decimRate,identificationMethods.algName{methodIndex},robot.name), 'results_PC_DIDIM', '-v7.3')
        
        disp("%%%%%%%%%%%%%%%    Finished PC-DIDIM    %%%%%%%%%%%%%%%")
        
    end % end PC_DIDIM
    
    methodIndex=methodIndex+1;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    if benchmarkSettings.displayProgression == true
        close(progressBar); % Close progress bar object
    end
    
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    disp("%%%%%%%%%%%%    Starting Post Processing    %%%%%%%%%%%%")
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    
    if runPostProcessing == true
        postProcessing(robot, benchmarkSettings, experimentDataStruct);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
else
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    disp("%%%%%%    No identification method selected...    %%%%%%")
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
end

disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
disp("%%%%%%%%%%%%%    Computations Completed    %%%%%%%%%%%%%")
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
