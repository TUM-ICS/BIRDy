function [Beta_HTRNN, it] = HTRNN_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsHTRNN)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the Hopfield Neural Network method.

%% Data Decimation and Filtering:

[~, ~, Qpp, Qp, Q, ~, Tau_decim, ~, ~, ~, optionsHTRNN] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsHTRNN, expNb, 'HTRNN');

%% Populating the Observation Matrix W and Torque Vector:

if strcmp(benchmarkSettings.codeImplementation,'optim')
    % Compute the sampled vector:
    Y_tau = torqueVector_mex(Tau_decim);
    % Compute the observation matrix:
    W_ndec = observationMatrix_mex(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);
elseif strcmp(benchmarkSettings.codeImplementation,'classic')
    % Compute the sampled vector:
    Y_tau = torqueVector(Tau_decim);
    % Compute the observation matrix:
    W_ndec = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);
else
    error("HTRNN: unknown option");
end

% Decimation filter
W=zeros(robot.nbDOF*optionsHTRNN.nbSampleDecim, robot.paramVectorSize); % Observation matrix
for col=1:robot.paramVectorSize
    for i = 1:robot.nbDOF
        W(i:robot.nbDOF:end,col) = decimate(W_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
    end
end

%% Defining the Hopfield Neural Network:

% Initialization:

Beta = Beta_0;

% Calculate weight and bias matrices:
alpha = max(abs(1.5*Beta_0));
beta = 5;
Betas = zeros(robot.paramVectorSize, optionsHTRNN.nbSampleDecim);
U = Beta_0;
Beta_dot = ones(robot.paramVectorSize,1);
it=1;
Weight = -W'*W;
Bias = - W'*Y_tau;

% Simulation of Hopfield-Tank neural dyanmics:
while norm(Beta_dot)>optionsHTRNN.stopCrit.tol && it < optionsHTRNN.stopCrit.Max_training_epochs
    Beta_old = Beta;
    [U, Beta] = HopfieldNeuronDynamics(U, Weight, Bias, alpha, beta, optionsHTRNN.learningRate);
    Beta_dot = (Beta_old-Beta)/optionsHTRNN.learningRate;
    Err = norm(Beta-benchmarkSettings.Beta_obj);
    Betas(:,it)=Beta;
    it =  it +1;
end

Beta_HTRNN = Beta;

%% Debug plot:

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_HTRNN, optionsHTRNN, expNb, 'HTRNN', Betas, it);

end

function [U, Beta] = HopfieldNeuronDynamics(U, Weight, Bias, alpha, beta, dt)

Beta = alpha*tanh(U/beta); % Compute the output of the neurons
Udot = Weight*Beta - Bias; % Compute the change of neuron input at next iteration
U = U + Udot*dt;

end
