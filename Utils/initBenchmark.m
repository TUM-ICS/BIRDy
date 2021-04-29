function [robot, benchmarkSettings, experimentDataStruct, startBenchmark, progressBar] = initBenchmark(robotName, identificationMethods, getKinematicExpressions, recompileMexFiles, displayProgression, regenerateModel, ...
    regenerateTrajectory, regenerateData, displayTrajectory, displayControlPerformance, t_i, t_f, integrationAlgorithm, interpolator, numberOfInitialEstimates, sdOfInitialEstimates, freq_decim, samplingBorder, buildList, ...
    filter, generateROSTrajectory, experimentOnTrueRobot, identifyDriveGains, initialParamType, noiseLevel, gammaReg)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function automatically setup the benchmark parameters and data structures.

%% Recompile the mex files if desired or if required:

% Check mex files extension:
extensionMex = mexext;

% Check the operating system
if isunix
    disp('Linux detected.')
    OS = 'linux';
    startBenchmark=true;
elseif ismac
    disp('Mac detected.')
    OS = 'mac';
    startBenchmark=true;
elseif ispc
    disp('Windows detected.')
    OS = 'windows';
    startBenchmark=true;
else
    disp('Platform not recognized. So far only Linux and Windows are supported.')
    OS = 'unknown';
    startBenchmark=false;
    decisionCompilation = 'N';
end

if ~any(identificationMethods.isActivated) % If no identification method is selected, do not start the benchmark...
    startBenchmark=false;
end

disp('Starting Benchmark...');
benchmarkSettings.codeImplementation = 'none';
benchmarkSettings.outputPath = 'figures/Results';% '../LaTeX/Journal/figures/Results';

if sum(identificationMethods.isActivated)~=0
    
    if license('test','distrib_computing_toolbox')==0 % If the parallel computing toolbox is not detected on the machine
        disp('The distributed computing toolbox was not detected. Expect slower computations for the CLOE, and Kalman filtering identification routines...')
    end
    
    if license('test','matlab_coder')==0 % If the matlab coder is not detected
        disp('The matlab coder toolbox was not detected. Switching to pure matlab computations (no mex).')
        benchmarkSettings.codeImplementation = 'classic';
    elseif (recompileMexFiles == true) && ~(license('test','matlab_coder')==0)
        disp('Automatic recompileMexFiles option was selected: proceeding...')
        compileTime = recompileMex(buildList,benchmarkSettings);
        fprintf('The total compile time was: %d s\n', compileTime);
        benchmarkSettings.codeImplementation = 'optim';
    else
        disp('The matlab coder toolbox was detected.')
        disp('Using mex files will increase the execution speed.')
        prompt1 = 'Do you want to use mex files ? Y/N [Y]';
        flag1=false;
        flag2=false;
        flag3=false;
        while flag1==false
            decisionMex = input(prompt1,'s');
            if isempty(decisionMex)
                decisionMex = 'Y';
            end
            if strcmpi(decisionMex, 'Y')
                flag1=true;
                versionMatlab = ver('matlab');
                
                if ~(exist('Utils/compiledVersionMEX.txt', 'file') == 2) % If the compilation status file does not exist
                    disp('The mex files were never compiled on this machine. If you want to use mex files, it will be necessary to recompile them.')
                    recompile = true;
                else
                    
                    fileID = fopen('Utils/compiledVersionMEX.txt','r');
                    
                    if fileID ~= -1
                        versionMatlabMexFile = fscanf(fileID,'%s'); % Version of matlab used to compile the mex files.
                    else
                        versionMatlabMexFile = -1; % Version of matlab used to compile the mex files.
                    end
                    
                    if (fileID ~= -1) && strcmpi(sprintf('%s,%s',versionMatlab.Release,OS),versionMatlabMexFile) % If same matlab version than the one used to compile the mex files
                        fprintf("You are using Matlab %s on a %s platform... \n", versionMatlab.Release, OS);
                        disp('Your matlab version and operating system are compatible with the provided mex files. Launching parameter identification without recompiling...')
                        benchmarkSettings.codeImplementation = 'optim';
                        recompile = false;
                        fclose(fileID);
                    elseif fileID == -1 % No track of previous mex build: rebuild
                        fprintf("You are using Matlab %s on a %s platform... \n", versionMatlab.Release, OS);
                        disp('Unable to detect the matlab version used for building the provided MEX files.  If you want to use mex files, it will be necessary to recompile them.')
                        recompile = true;
                    elseif (fileID ~= -1) && ~strcmpi(sprintf('%s,%s',versionMatlab.Release,OS),versionMatlabMexFile)% If NOT same matlab version than the one used to compile the mex files
                        fprintf("You are using Matlab %s on a %s platform... \n", versionMatlab.Release, OS);
                        versionMatlabMex = extractBefore(versionMatlabMexFile,",");
                        versionOSMex = extractAfter(versionMatlabMexFile,",");
                        if ~strcmp(versionOSMex, OS) % Critical mismatch in OS version
                            fprintf('This benchmark was originally compiled for %s. If you want to use mex files, it will be necessary to recompile them for your operating system.\n',  versionOSMex);
                            recompile = true;
                        end
                        if ~strcmp(versionMatlabMex, versionMatlab.Release) % Non-critical mismatch in matlab version
                            fprintf('This benchmark was originally compiled for matlab %s. You can decide to recompile the mex files but this should not be necessary in most cases.\n',  versionMatlabMex);
                            while flag3==false
                                decisionRecomp = input('Do you want to recompile mex files ? Y/N [N]','s');
                                if isempty(decisionRecomp)
                                    decisionRecomp = 'N';
                                end
                                if strcmpi(decisionRecomp, 'Y')
                                    recompile = true;
                                    flag3=true;
                                elseif strcmpi(decisionRecomp, 'N')
                                    recompile = false;
                                    benchmarkSettings.codeImplementation = 'optim';
                                    flag3=true;
                                else
                                    disp('Incorrect input !')
                                    flag3=false;
                                end
                            end
                        end
                        
                        fclose(fileID);
                    else
                        recompile = true;
                    end
                end
                
                if recompile == true
                    
                    prompt2 = 'Do you want to recompile mex files ? Y/N [Y]';
                    
                    while flag2==false
                        decisionCompilation = input(prompt2,'s');
                        if isempty(decisionCompilation)
                            decisionCompilation = 'Y';
                        end
                        if strcmpi(decisionCompilation, 'Y')
                            flag2=true;
                            compileTime = recompileMex(buildList, benchmarkSettings);
                            fprintf("The total compile time was: = %d minutes, %d seconds\n", floor(compileTime/60), ceil(rem(compileTime,60)));
                            
                            fileID = fopen('Utils/compiledVersionMEX.txt','w+'); % Overwrite file
                            fprintf(fileID, '%s,%s\n', versionMatlab.Release, OS);
                            fclose(fileID);
                            
                            benchmarkSettings.codeImplementation = 'optim';
                        elseif strcmpi(decisionCompilation, 'N')
                            flag2=true;
                            disp('Switching to pure matlab computations (no mex).')
                            benchmarkSettings.codeImplementation = 'classic';
                        else
                            disp('Incorrect input !')
                            flag2=false;
                        end
                    end
                end
                
            elseif strcmpi(decisionMex, 'N')
                flag1=true;
                disp('Switching to pure matlab computations (no mex).')
                benchmarkSettings.codeImplementation = 'classic';
            else
                disp('Incorrect input !')
                flag1=false; % looping
            end
        end
    end
elseif (recompileMexFiles == true) && license('test','matlab_coder')
    disp('Automatic recompileMexFiles option was selected: proceeding...')
    compileTime = recompileMex(buildList, benchmarkSettings);
    fprintf('The total compile time was: %d s\n', compileTime);
    benchmarkSettings.codeImplementation = 'optim';
end

% Is the benchmark executed on a true robot ?
benchmarkSettings.experimentOnTrueRobot = experimentOnTrueRobot;
benchmarkSettings.identifyDriveGains = identifyDriveGains;
if benchmarkSettings.experimentOnTrueRobot
    noiseLevel = 'oldNoise';                                         % Use the noise parameters found for the true robot.
end

benchmarkSettings.noiseLevel = noiseLevel;
benchmarkSettings.regFactor = gammaReg;

%% Symbolic model computation:

% Check if the robot symbolic dynamics has already been computed:
robotHasSymbolicModel = exist(sprintf('Benchmark/Robot_Generated_Data/%s', robotName), 'dir');

if (robotHasSymbolicModel ==  0) || (regenerateModel == true) % If the robot folder does not exist, so does its dynamic model...
    % Start by creating the folders which will contain the symbolic expressions:
    disp('The robot symbolic model was not detected or you decided to rebuild it: starting new symbolic computation of the dynamic model...');
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s', robotName))
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s/Homogeneous_Transforms', robotName))
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s/Jacobian_Matrices', robotName))
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model', robotName))
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s/Experiment_Data', robotName))
    mkdir(sprintf('Benchmark/Robot_Generated_Data/%s/Trajectory_Data', robotName))
    addpath(genpath('Benchmark'));  % Add the newly created directories to the matlab path
    
    % Set the symbolic computation options:
    optionsSymbolicComputation.computeKinematics = true;            % If true, computes the robot symbolic kinematics (but does not save it as a file)
    optionsSymbolicComputation.computeDynamics = true;              % If true, computes the robot symbolic dynamics (but does not save it as a file)
    optionsSymbolicComputation.optimizeKinematics = true;           % If true, generate a set of optimized files containing robot kinematics.
    optionsSymbolicComputation.optimizeDynamics = true;             % If true, generate a set of optimized files containing robot dynamics.
    
    % Load Robot Symbolic and Numerical Parameters:
    disp('Loading robot numerical model parameters...');
    options.loadSymbolic = true;
    options.checkPhysicality = false;
    options.noiseLevel = noiseLevel;
    [robot] = loadRobotModelParameters(robotName, options);
    
    if ~computeSymbolicDynamics(robot, optionsSymbolicComputation)  % Generate robot symbolic model
        error('An error occured during symbolic model computation.');
    else
        recompileMexFiles = true;                                   % Recompile mex files as soon as dynamics model has been modified
    end
elseif sum(identificationMethods.isActivated)==0 && getKinematicExpressions == true
    % Load Robot Numerical and Symbolic Parameters for vizualization purpose only:
    disp('Symbolic model detected, no need to recompute it.');
    disp('Loading robot symbolic and numerical model parameters...');
    options.loadSymbolic = true;
    options.checkPhysicality = false;
    options.noiseLevel = noiseLevel;
    [robot] = loadRobotModelParameters(robotName, options);
    displaySymbolicTransforms(robot);
else
    % Just load Robot Numerical Parameters:
    disp('Symbolic model detected, no need to recompute it.');
    disp('Loading robot numerical model parameters...');
    options.loadSymbolic = false;
    options.checkPhysicality = true;
    options.noiseLevel = noiseLevel;
    [robot] = loadRobotModelParameters(robotName, options);
    % Looking for inconsistency between the dynamic model and the identification model:
    if checkDynamicsConsistancy(robotName)== true
        disp('Everything seems to be fine ! Moving forward...');
    else
        error('Inconsistancy detected between the robot dynamic model and the robot identification model !');
    end
end

%% Initialization:

% Total number of identification methods:
benchmarkSettings.nbAlg = numel(identificationMethods.isActivated);
benchmarkSettings.identificationMethods = identificationMethods;

% Important parameters:
benchmarkSettings.displayProgression = displayProgression;                  % Display progression GUI.
benchmarkSettings.regenerateTrajectory = regenerateTrajectory;              % Generate a new trajectory if set to true.
benchmarkSettings.regenerateData = regenerateData;                          % Generate new robot simulation data if set to true.
benchmarkSettings.displayTrajectory = displayTrajectory;                    % Display the robot joint trajectory.
benchmarkSettings.displayControlPerformance = displayControlPerformance;    % Display the robot tracking performance on the generated trajectory.
Beta = feval(sprintf('Regressor_Beta_%s', robot.name),robot.numericalParameters.Xhi);
robot.paramVectorSize = numel(Beta);

if benchmarkSettings.displayProgression == true
    progressBar = waitbar(0,'Benchmark initialization', 'name', 'Progress of the parametric identification sub-process');
else
    progressBar = 0;
end

% Identify the robot using a subset of the experiment trajectory [t_i, t_f]:
benchmarkSettings.robotName = robotName;
benchmarkSettings.t_i = t_i;                                            % Start time
benchmarkSettings.t_f = t_f;                                          	% End time
if benchmarkSettings.t_i>benchmarkSettings.t_f
    error('Incorrect simulation time horizon ! The simulation stop time t_f cannot be smaller than the simulation init time t_i!');
elseif benchmarkSettings.t_i<0
    error('Incorrect simulation time horizon ! The simulation init time t_i should be greater than 0!');
end
benchmarkSettings.nbSamples = (t_f-t_i)*robot.controlParameters.samplingFrequency+1; % Number of experiment samples
benchmarkSettings.nbCtrlSamples = (t_f-t_i)*robot.controlParameters.controlFrequency+1; % Number of control samples
benchmarkSettings.dt = 1/robot.controlParameters.samplingFrequency;     % Sampling time
benchmarkSettings.f = robot.controlParameters.samplingFrequency;        % Sampling frequency
benchmarkSettings.f_ctrl = robot.controlParameters.controlFrequency;    % Control frequency
benchmarkSettings.dt_ctrl = 1/robot.controlParameters.controlFrequency; % Control time

benchmarkSettings.numberOfInitialEstimates = numberOfInitialEstimates;  % Number of different initial estimates
benchmarkSettings.sdOfInitialEstimates = sdOfInitialEstimates;          % Standard deviation of the different initial estimates

benchmarkSettings.integrationAlgorithm = integrationAlgorithm;          % Defines the integration algorithm used during Identification : 'rk1', 'rk2', 'rk4' or 'ode45'
benchmarkSettings.interpolationAlgorithm = interpolator.Algorithm;      % Defines the interpolation algorithm used for trajectory data: 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'.
benchmarkSettings.interpolationAlgorithmExp = interpolator.expAlgorithm;% Defines the interpolation algorithm used for experiment data: 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'.

benchmarkSettings.Xhi_obj = robot.numericalParameters.Xhi;
benchmarkSettings.Beta_obj = feval(sprintf('Regressor_Beta_%s', robot.name),benchmarkSettings.Xhi_obj);

if interpolator.visualizeInterpolator == true
    testInterpolationTrajectory();
end

% Decimate parameters
benchmarkSettings.fdecim = freq_decim;
benchmarkSettings.fnyq = benchmarkSettings.f/2;                         % Nyquist frequency
benchmarkSettings.decimRate = round(0.8*benchmarkSettings.fnyq/(benchmarkSettings.fdecim)); % Decimation rate (see the Matlab 'decimate' documentation)
benchmarkSettings.samplingBorder = samplingBorder;                      % Sampling border (number of samples to be removed from the beginning and the end of the data sequence, in order to mitigate filtering artefacts...)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if benchmarkSettings.experimentOnTrueRobot
    benchmarkSettings.numberOfExperimentsPerInitialPoint = 1;
else
    benchmarkSettings.numberOfExperimentsPerInitialPoint = robot.paramVectorSize;          % Number of experiments for each method and each initial point
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if sum(identificationMethods.isActivated)~=0
    %% Generate trajectory:
    
    if benchmarkSettings.experimentOnTrueRobot == true
        trajectoryDataFile = sprintf('Benchmark/Robot_Generated_Data/%s/Trajectory_Data/trajectoryData_%s_real.mat', benchmarkSettings.robotName, benchmarkSettings.robotName);
        
        if ~exist(trajectoryDataFile, 'file')
            % Trigger real experiment data preprocessing:
            experimentData = feval(sprintf('preprocessExperimentData_%s', robot.name),robot, benchmarkSettings, options);
        else
            % Load existing robot trajectory data:
            disp('Trajectory Data found: Loading Trajectory Data...');
            load(trajectoryDataFile);
        end
        
    else
        trajectoryParameterFile = sprintf('Benchmark/Robot_Generated_Data/%s/Trajectory_Data/trajectoryParameters_%s.mat', benchmarkSettings.robotName, benchmarkSettings.robotName);
        trajectoryDataFile = sprintf('Benchmark/Robot_Generated_Data/%s/Trajectory_Data/trajectoryData_%s.mat', benchmarkSettings.robotName, benchmarkSettings.robotName);
        
        if ~exist(trajectoryDataFile, 'file') || benchmarkSettings.regenerateTrajectory % If the robot trajectory data file is not detected or if the flag 'regenerateTrajectory' is set.
            
            if ~exist(trajectoryParameterFile, 'file') || benchmarkSettings.regenerateTrajectory
                % Optimize robot trajectory:
                benchmarkSettings.max_traj_it = 1;
                disp('Optimizing robot trajectory...');
                [trajectoryParameters, trajectoryData] = generateExcitationTrajectory(robot, benchmarkSettings, true);
                save(trajectoryParameterFile, 'trajectoryParameters', '-v7.3');
            else
                load(trajectoryParameterFile);
                benchmarkSettings.trajectoryParameters = trajectoryParameters;
                [~, trajectoryData] = generateExcitationTrajectory(robot, benchmarkSettings, false);
            end
            % Save trajectory data:
            disp('Saving Trajectory Data...');
            save(trajectoryDataFile, 'trajectoryData', '-v7.3');
        else
            % Load existing robot trajectory data:
            disp('Trajectory Data found: Loading Trajectory Data...');
            load(trajectoryDataFile);
        end
        
    end
    
    % Store trajectory data in a dedicated object:
    benchmarkSettings.trajectoryData = Trajectory;
    benchmarkSettings.trajectoryData.setTrajectoryData(trajectoryData.t,trajectoryData.Q,trajectoryData.Qp,trajectoryData.Qpp);
    
    if benchmarkSettings.displayTrajectory == true
        plotTrajectory(robot, benchmarkSettings);
    end
    
    %% Generate experiment data:
    
    if benchmarkSettings.experimentOnTrueRobot == true
        dataFolder = sprintf('Benchmark/Robot_Generated_Data/%s/Experiment_Data/data_%s_real.mat', benchmarkSettings.robotName, benchmarkSettings.robotName);
    else
        dataFolder = sprintf('Benchmark/Robot_Generated_Data/%s/Experiment_Data/data_%s_sim_%s.mat', benchmarkSettings.robotName, benchmarkSettings.robotName, noiseLevel);
    end
    %         [Gains_TCG, fval, output, exitflag, lambda, jacobian] = TuneControlGains(robot, benchmarkSettings, options);
    %         Gains_TCG
    %         pause
    
    if ~exist(dataFolder, 'file') || benchmarkSettings.regenerateData  % If the robot simulation data is not detected or if the flag 'regenerateData' is set
        
        if benchmarkSettings.experimentOnTrueRobot == true
            % Trigger real experiment data preprocessing:
            experimentData = feval(sprintf('preprocessExperimentData_%s', robot.name),robot, benchmarkSettings, options);
        else
            % Generate simulated experiment data:
            disp('Generating Experiment Data...');
            options.jointFrictionData = false;
            options.integrationAlgorithm = 'rk4';
            experimentData = generateExperimentData(robot, benchmarkSettings, options);
        end
        % Save experiment data:
        disp('Saving Experiment Data...');
        save(dataFolder, 'experimentData', '-v7.3');
    else
        % Load existing robot experiment data:
        disp('Experiment Data found: Loading Experiment Data...');
        load(dataFolder);
    end
    
    % Store the experiment data in a dedicated container equiped with zero-order time interpolation routines so that it can be accessed at any epoch:
    if benchmarkSettings.experimentOnTrueRobot == true
        experimentDataStruct = RealExperiment;
    else
        experimentDataStruct = SimulatedExperiment;
    end
    experimentDataStruct.setExperimentData(experimentData);
    
    if benchmarkSettings.displayControlPerformance == true && benchmarkSettings.experimentOnTrueRobot == false
        plotControlPerformance(robot, benchmarkSettings, experimentDataStruct, 1)
    end
    
    if generateROSTrajectory == true
        options.generateRealRobot = true;
        options.useROS = false; % publish trajectories as a ROS topic for visualization purpose
        Experiment_ROS_Trajectory(robot, benchmarkSettings, options); % Generate .yaml trajectory file to be used on a real robot
    end
    
    disp('Experiment Data Ready !');
    
    %% Filtering
    
    % Filter parameters
    
    benchmarkSettings.filter = filter;
    
    % Filter performance visualization
    if filter.visualizeFilter == true
        filterTuning(benchmarkSettings, experimentDataStruct);
    end
    
% else
%     experimentDataStruct = 0;
% end

%% Generate starting points:

% Generate a set of 'numberOfInitialEstimates' initial parameter estimates and store it in 'Initial_pts':
benchmarkSettings.initialParamType = initialParamType;
[benchmarkSettings.Xhi_U, benchmarkSettings.Xhi_L, ~, benchmarkSettings.Beta_U, benchmarkSettings.Beta_L, benchmarkSettings.Initial_Beta, benchmarkSettings.Initial_Xhi] = generateInitParamEst(robot, benchmarkSettings, experimentDataStruct);

end
