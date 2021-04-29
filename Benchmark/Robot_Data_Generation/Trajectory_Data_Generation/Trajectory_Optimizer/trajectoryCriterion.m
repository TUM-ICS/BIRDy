function J = trajectoryCriterion(robot, trajectoryParameters)

% Authors: Julien Roux, Quentin Leboutet, Alexandre Janot, Gordon Cheng
%
% The cost-function to be minimized during the trajectory generation process.

nbTrajSamples = trajectoryParameters(1,2);
t_i = trajectoryParameters(6,2);
t_f = trajectoryParameters(7,2);
W = zeros(robot.nbDOF*nbTrajSamples, robot.paramVectorSize);
timeSamples = linspace(t_i, t_f, nbTrajSamples);

for i=1:nbTrajSamples
    augmentedState = trajectoryGeneratorFourier(timeSamples(i), trajectoryParameters, robot.numericalParameters.Q0); % augmentedState = [Qpp; Qp; Q];
    Qpp = augmentedState(1:robot.nbDOF);
    Qp = augmentedState(robot.nbDOF+1:2*robot.nbDOF);
    Q = augmentedState(2*robot.nbDOF+1:end);
    W(robot.nbDOF*(i-1)+1:robot.nbDOF*i,:) = feval(sprintf('Regressor_Y_%s', robot.name),Q, Qp, Qpp, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity);
end


%% Compute the optimization cost:

k1 = 1;
k2 = 100;
S = svd(W);
sig_min = min(S);
C = cond(W'*W);

J = k1*C+k2*1/sig_min;

end
