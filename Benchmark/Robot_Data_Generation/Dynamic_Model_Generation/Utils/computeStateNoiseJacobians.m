function [stateJacobian, noiseJacobian] = computeStateNoiseJacobians(robot, M, C, G, Tau_friction, nbParam)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

disp('Augmented State vector:')
x = [robot.symbolicParameters.Qp;robot.symbolicParameters.Q;robot.symbolicParameters.Beta(1:nbParam)];
% [dimx,~] = size(x);

disp('State function:')

f = x+[M\(robot.symbolicParameters.Tau - C*robot.symbolicParameters.Qp - G - Tau_friction); robot.symbolicParameters.Qp; sym(zeros(nbParam,1))]*robot.symbolicParameters.dt + robot.symbolicParameters.Noise(1:nbParam);

% [dimf,~] = size(f);

disp('Computing state Jacobian:')
% for i = 1:dimx
%     for j = 1:dimf
%         testA(i,j) = diff(f(j),x(i));
%     end
% end

stateJacobian=jacobian(f,x);
% ErrorA = simplify(stateJacobian - testA)

dis('Computing noise Jacobian:')
% for i = 1:dimf
%     for j = 1:dimx
%         testB(i,j) = diff(f(i),noise(j));
%     end
% end

noiseJacobian = eye(2*robot.nbDOF + nbParam);%jacobian(f,robot.symbolicParameters.Noise);
% ErrorB = simplify(B - testB)
end

