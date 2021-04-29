function [trajectoryParameters, trajectoryData] = generateExcitationTrajectory(robot, benchmarkSettings, generateNewTrajectoryParameters)


if generateNewTrajectoryParameters == true
    %% Trajectory optimization
    
    % Authors: Julien Roux, Quentin Leboutet, Alexandre Janot, Gordon Cheng
    
    %% -> Choice of a criterion
    % The conditionning of the W matrix is a classic
    % C = k1*cond(W) + k2/sig_min
    %   - cond(W) needs to be as close as possible to 1 (it is superior to 1)
    %   - sig_min needs to be as big as possible
    % Thus C has to be as close as possible to 1 (it is superior to 1)
    % k1 and k2 have to be heuristically chosen
    
    %% -> Parametrization of the trajectory
    
    nbTrajSamples = 100;
    caractFreq = 1/(benchmarkSettings.t_f-benchmarkSettings.t_i);
    caractPuls = 2*pi*caractFreq;
    n_f = 20;
    
    limQ_U = robot.physicalConstraints.limQ_U;
    limQp_U = robot.physicalConstraints.limQp_U;
    limQpp_U = robot.physicalConstraints.limQpp_U;
    limQ_L = robot.physicalConstraints.limQ_L;
    limQp_L = robot.physicalConstraints.limQp_L;
    limQpp_L = robot.physicalConstraints.limQpp_L;
    
    augmentedState_max = [limQpp_U; limQp_U; limQ_U];
    augmentedState_min = [limQpp_L; limQp_L; limQ_L];
    
    nbVars = 2*n_f*robot.nbDOF;
    
    
    %% Setting constraints on initial position and velocity:
    Aeq = zeros(3*robot.nbDOF, nbVars);
    beq = zeros(3*robot.nbDOF,1);
    
    for i=1:robot.nbDOF
        for j=1:n_f
            Aeq(i, 2*n_f*(i-1)+n_f+j)=1/(caractPuls*j); % -> set the initial position to the desired position qi0
            beq(i)=0;
            Aeq(robot.nbDOF+i, 2*n_f*(i-1)+j)=1; % -> set the initial velocities to 0
            Aeq(2*robot.nbDOF+i, 2*n_f*(i-1)+n_f+j)=caractPuls*j; % -> set the initial acceleration to 0
        end
    end
    
    %% Trajectory optimization:
    
    trajectoryParameters = zeros(nbVars,2);
    trajectoryParameters(1,2) = nbTrajSamples;
    trajectoryParameters(2,2) = benchmarkSettings.max_traj_it;
    trajectoryParameters(3,2) = caractFreq;
    trajectoryParameters(4,2) = caractPuls;
    trajectoryParameters(5,2) = n_f;
    trajectoryParameters(6,2) = benchmarkSettings.t_i;
    trajectoryParameters(7,2) = benchmarkSettings.t_f;
    trajectoryParameters(8,2) = robot.nbDOF;
    fval_final = trajectoryCriterion(robot, trajectoryParameters);
    
    iteration = 0;
    alg = 'fmin'; % 'fmin' or 'ga'
    
    fun = @(trajectoryParameters_tmp)trajectoryCriterion(robot, [trajectoryParameters_tmp trajectoryParameters(:,2)]);
    nonlcon = @(trajectoryParameters_tmp) trajectoryConstraints(robot, [trajectoryParameters_tmp trajectoryParameters(:,2)], augmentedState_max, augmentedState_min);
    
    [C0,Ceq0]=trajectoryConstraints(robot, [0.001*(-2 + 4*rand(nbVars, 1))  trajectoryParameters(:,2)], augmentedState_max, augmentedState_min);
    
    if ~all(C0 <= 0) % Trajectory generation constraints are violated by the initial point...
        error('Trajectory generation constraints are violated by the initial point...');
    end
    
    while iteration<benchmarkSettings.max_traj_it && fval_final > 5
        if strcmp(alg,'fmin') % 'interior-point' (default), 'trust-region-reflective', sqp', 'sqp-legacy', 'active-set'. 
            %options = optimoptions('fmincon','Algorithm' ,'interior-point', 'Display','iter', 'MaxFunctionEvaluations', 10000*robot.nbDOF, 'UseParallel', true);
            options = optimoptions('fmincon','Algorithm' ,'interior-point', 'Display','iter', 'MaxFunctionEvaluations', 10, 'UseParallel', true);
            [trajectoryParameters_optim,fval,exitFlag] = fmincon(fun, 0.1*(-2 + 4*rand(nbVars, 1)), [], [], Aeq, beq, [], [], nonlcon, options);
        elseif strcmp(alg,'ga')
            population = 240;
            options = optimoptions('ga','MaxStallGenerations',20,'MutationFcn',@mutationadaptfeasible, 'FunctionTolerance',1e-10,'Display','iter','MaxGenerations',50*robot.nbDOF,'PlotFcn',{@gaplotbestindiv,@gaplotdistance,@gaplotrange}, 'PopulationSize', population,'UseParallel', true, 'EliteCount', floor(population/10), 'CrossoverFraction', 0.9);
            [trajectoryParameters_optim,fval] = ga(fun, nbVars, [], [], Aeq, beq, [], [], nonlcon, options);
        else
            error("Trajectory generation: unknown algorithm");
        end
        
        if fval < fval_final
            trajectoryParameters(:,1) = trajectoryParameters_optim;
            fval_final = fval;
        end
        
        iteration = iteration + 1;
    end
    
    fprintf('Optimization Complete!');
    fprintf('Lowest value of the cost functon: %d\n', fval_final);
    
    % Trajectory data generation:
    
    fprintf('Generating Trajectory Data...\n');
    trajectoryData = generateTrajectoryData(benchmarkSettings, robot, trajectoryParameters);
    fprintf('Trajectory Data Generation Completed!\n');
    
else
    
    % Trajectory data generation only:
    
    trajectoryParameters = benchmarkSettings.trajectoryParameters;
    fprintf('Generating Trajectory Data...\n');
    trajectoryData = generateTrajectoryData(benchmarkSettings, robot, trajectoryParameters);
    fprintf('Trajectory Data Generation Completed!\n');
end

end



function [trajectoryData] = generateTrajectoryData(benchmarkSettings, robot, trajectoryParameters)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot, Gordon Cheng
%
% Generation of trajectory data

t = linspace(benchmarkSettings.t_i,benchmarkSettings.t_f,benchmarkSettings.nbCtrlSamples);
trajectoryData.t = t;
trajectoryData.Q=zeros(robot.nbDOF,benchmarkSettings.nbCtrlSamples);
trajectoryData.Qp=zeros(robot.nbDOF,benchmarkSettings.nbCtrlSamples);
trajectoryData.Qpp=zeros(robot.nbDOF,benchmarkSettings.nbCtrlSamples);

for i = 1:benchmarkSettings.nbCtrlSamples
    augmentedState=trajectoryGeneratorFourier(t(i), trajectoryParameters, robot.numericalParameters.Q0);
    trajectoryData.Qpp(:,i) = augmentedState(1:robot.nbDOF);
    trajectoryData.Qp(:,i) = augmentedState(robot.nbDOF+1:2*robot.nbDOF);
    trajectoryData.Q(:,i) = augmentedState(2*robot.nbDOF+1:end);
end

end



