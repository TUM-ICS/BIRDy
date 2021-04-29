function [status] = checkPhysicalConsistency(robot)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function check whether the numerical values of the robot parameters
% are physically consistent or not...

status = true;

disp('Checking physical consistency of the robot parameters...');

for i = 1: robot.nbDOF
    
    % Check positivity of the friction terms:
    if robot.numericalParameters.friction.Fc(i) < 0 
        status = false;
        error('Fc(%d) < 0 !\n',i);
    end
    if robot.numericalParameters.friction.Fv(i) < 0
        status = false;
        error('Fv(%d) < 0 !\n',i);
    end
    
    % Check positivity of the transmission chain inertia term Ia:
    if robot.numericalParameters.Ia(i) < 0
        status = false;
        error('Ia(%d,%d) < 0 !\n',i,i);
    end
    
    % Check positive-definiteness of the inertia matrix I:
    I = robot.numericalParameters.InertiaCOM(:,:,i);
    
    [~,FLAG] = chol(I);
    if FLAG ~= 0
        status = false;
        I
        error('Inertia matrix of link %d wrt COM is NOT positive-definite !\n',i);
    end
    
    % Check positive-definiteness of the pseudo-inertia matrix P:
    P = pseudoInertiaMatrix(robot.numericalParameters.InertiaDH(:,:,i), robot.numericalParameters.Mass(i), robot.numericalParameters.Mass(i)*robot.numericalParameters.GeometryCOM(:,i), robot.numericalParameters.Ia(i), robot.numericalParameters.friction.Fv(i), robot.numericalParameters.friction.Fc(i));
    
    [~,FLAG] = chol(P);
    if FLAG ~= 0
        status = false;
        P
        error('Pseudo inertia matrix of link %d is NOT positive-definite !\n',i);
    end
    
end

end
