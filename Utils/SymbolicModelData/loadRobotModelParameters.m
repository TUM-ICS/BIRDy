function [robot] = loadRobotModelParameters(robotName, options)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the robot data structure.

robot = feval(robotName, options);

if options.checkPhysicality == true
    load(sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_Perm_%s.mat',robot.name,robot.name))
    load(sprintf('Benchmark/Robot_Generated_Data/%s/Dynamic_Model/Regressor_K_d_%s.mat',robot.name,robot.name))
    
    robot.K_d = K_d;
    robot.Perm = Perm;
    if checkPhysicalConsistency(robot) == false
        error('The parameters of the simulated robot are not physically consistent !');
    else
        disp('The robot parameters seems to be physically consistent. Moving forward...');
    end
end
end
















