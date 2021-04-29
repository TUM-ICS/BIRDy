function [Beta_ANN] = ANN_identification(robot, experimentDataStruct, expNb, benchmarkSettings, Beta_0, optionsANN)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Performs parameter identification using stochastic gradient descent (Adaline Neural Network)

%% Data Decimation and Filtering:

[~, ~, Qpp, Qp, Q, ~, Tau_decim, ~, ~, ~, optionsANN] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsANN, expNb, 'ANN');

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
    error("ANN: unknown option");
end

% Decimation filter
W=zeros(robot.nbDOF*optionsANN.nbSampleDecim, robot.paramVectorSize); % Observation matrix
for col=1:robot.paramVectorSize
    for i = 1:robot.nbDOF
        W(i:robot.nbDOF:end,col) = decimate(W_ndec(i:robot.nbDOF:end,col),benchmarkSettings.decimRate);
    end
end

%% Defining the Adaline Neural Network:

switch (optionsANN.neuronet)
    case 'toolbox' % Use matlab neural network toolbox
        
        net = linearlayer;
        net = configure(net,zeros(robot.paramVectorSize,1),0);
        net.trainFcn = 'traingd';
        net.trainParam.epochs = optionsANN.stopCrit.Max_training_epochs;
        net.trainParam.min_grad = optionsANN.stopCrit.tol;
        
        % Given an input sequence with N steps the network is updated as follows.
        % Each step in the sequence of inputs is presented to the network one at
        % a time. The network's weight and bias values are updated after each step,
        % before the next step in the sequence is presented. Thus the network is
        % updated N times. The output signal and the error signal are returned,
        % along with new network.
        
        Inputs = con2seq(W');
        Targets = con2seq(Y_tau');
        net = train(net, Inputs, Targets, 'showResources', 'yes', 'useParallel','no');
        
        % view network structure
        % view(net)
        
        % check final network parameters
        % disp('Weights and bias of the ADALINE after adaptation')
        Beta = transpose(net.IW{1,1});
        
    case 'grad' % Use custom gradient descent
        
        % Initialization:
        
        Beta = Beta_0;
        Beta_dot = ones(robot.paramVectorSize,1);
        Betas = zeros(robot.paramVectorSize, optionsANN.nbSampleDecim);
        A = W'*W;
        it = 1;
        
        while norm(Beta_dot)>optionsANN.stopCrit.tol && it < optionsANN.stopCrit.Max_training_epochs
            r = W'*Y_tau-A*Beta;
            alpha = (r'*A*r)\r'*r;
            Beta = Beta  + optionsANN.learningRate*alpha*r;
            Err = norm(Beta-benchmarkSettings.Beta_obj);
            Betas(:,it)=Beta;
            it =  it +1;
        end
        
    case 'stochastic_grad'  % Use custom stochastic gradient descent
        
        % Initialization:
        Beta = Beta_0;
        Beta_dot = ones(robot.paramVectorSize,1);
        Betas = zeros(robot.paramVectorSize, optionsANN.nbSampleDecim);
        it = 1;
        j=1;
        index = randperm(optionsANN.nbSampleDecim,optionsANN.nbSampleDecim); % randomized sample indexes
        
        while norm(Beta_dot)>0.01*optionsANN.stopCrit.tol && it <= optionsANN.stopCrit.Max_training_epochs
            i=index(j);
            r = W((i-1)*robot.nbDOF+1:i*robot.nbDOF,:)'*Y_tau((i-1)*robot.nbDOF+1:i*robot.nbDOF)-W((i-1)*robot.nbDOF+1:i*robot.nbDOF,:)'*W((i-1)*robot.nbDOF+1:i*robot.nbDOF,:)*Beta;
            alpha = (r'*W((i-1)*robot.nbDOF+1:i*robot.nbDOF,:)'*W((i-1)*robot.nbDOF+1:i*robot.nbDOF,:)*r)\(r'*r);
            Beta = Beta  + optionsANN.learningRate*alpha*r;
            Err = norm(Beta-benchmarkSettings.Beta_obj);
            Betas(:,it)=Beta;
            it =  it + 1;
            
            j = j + 1;
            if i >= optionsANN.nbSampleDecim
                index = randperm(optionsANN.nbSampleDecim,optionsANN.nbSampleDecim); % randomized sample indexes
                j=1;
            end
        end
    otherwise
        error('ANN: unknown training algorithm !');
end

Beta_ANN = Beta;

%% Debug plot:

debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, Beta_ANN, optionsANN, expNb, 'ANN', Betas, it);

end

