function [C, Ceq] = trajectoryConstraints(robot, trajectoryParameters, augmentedState_max, augmentedState_min)

nbTrajSamples = trajectoryParameters(1,2);
t_i = trajectoryParameters(6,2); 
t_f = trajectoryParameters(7,2);
nbDOF = trajectoryParameters(8,2);

if (nbDOF ~= robot.nbDOF)
    error('The trajectory constraint tool detected inconsistancy in nbDOF !')
end

augmentedJointState = zeros(3*robot.nbDOF*nbTrajSamples,1);
cartesianState1 = zeros(3*nbTrajSamples,1);
cartesianState2 = zeros(3*nbTrajSamples,1);
timeSamples = linspace(t_i, t_f, nbTrajSamples);

for i=1:nbTrajSamples 
    augmentedState = trajectoryGeneratorFourier(timeSamples(i), trajectoryParameters, robot.numericalParameters.Q0);  % augmentedState = [Qpp; Qp; Q];
    augmentedJointState(3*robot.nbDOF*(i-1)+1:3*robot.nbDOF*i,1) = augmentedState;
    Q = augmentedState(2*robot.nbDOF+1:end);
    HT1 = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name),Q, robot.numericalParameters.Geometry);
    %HT2 = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF-1,robot.name),Q, robot.numericalParameters.Geometry);
    %     cartesianState(3*(i-1)+1:3*i,1) = 0;%HT(1:3,4); % Constraint x, y and z axes in the positive space.
    cartesianState1(i,1) = -HT1(3,4)+0.3; % Constraint z axis in the positive space.
    %cartesianState2(i,1) = HT2(2,4)+0.2; % Constraint y axis in the positive space.
end

C = [augmentedJointState-repmat(augmentedState_max-0.1,nbTrajSamples,1);repmat(augmentedState_min+0.1,nbTrajSamples,1)-augmentedJointState; cartesianState1]; % C <= 0
% C = [augmentedJointState-repmat(augmentedState_max-0.1,nbTrajSamples,1);repmat(augmentedState_min+0.1,nbTrajSamples,1)-augmentedJointState]; % C <= 0
Ceq = [];

end
