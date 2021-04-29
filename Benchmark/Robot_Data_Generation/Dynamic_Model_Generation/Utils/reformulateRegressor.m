function [Y_M, Tau_h, Beta_symb] = reformulateRegressor(robot, Y_b, Beta)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function ptovides symbolic expressions for the inertia matrix and
% the Corriolis+gravity+friction vector as a function of the BASE inertial
% parameter vector Beta (by contrast to the standard inertial parameter 
% vector Xhi)

Qp = robot.symbolicParameters.Qp;
Qpp = robot.symbolicParameters.Qpp;
Gravity = robot.symbolicParameters.Gravity;
Qp0 = sym(zeros(robot.nbDOF,1));
Qpp0 = sym(zeros(robot.nbDOF,1));
Gravity0 = sym(zeros(3,1));

Beta_symb = sym('Beta', size(Beta));

fprintf('Rewriting robot dynamics as a function of Beta...');
Y_b_handle =  subs(Y_b, Qpp, Qpp0);
Tau_h = Y_b_handle*Beta_symb; % Setting Qpp to 0 results in Tau_h =  C(Q,Qp)*Qp + g(Q) + Friction + Tau_off = f(Beta).

for i = 1:robot.nbDOF
    fprintf('.');
    Qpp0(i,1) = sym(1); 
    % In the expression M(Q)*Qpp, it we only set Qpp(i) to 1 and the rest
    % of Qpp elements to 0, then the torque M(Q)*Qpp will be given by the i^th
    % column of M(Q):
    Y_b_handle =  subs(Y_b, [Qp;Qpp;Gravity] , [Qp0;Qpp0;Gravity0]);
    Y_M(:,i) = Y_b_handle*Beta_symb ; % Gives M as a function of Beta.
    Qpp0(i,1) = sym(0);
end

% To handle the offset parameters:
for i = 1:robot.nbDOF
    fprintf('.');
    Y_b_handle =  subs(Y_b, [Qp;Qpp;Gravity] , [Qp0;Qpp0;Gravity0]);
    Y_M(:,i) = Y_M(:,i) - Y_b_handle*Beta_symb; % Gives M as a function of Beta.
    Qpp0(i,1) = sym(0);
end
fprintf('.\n');
end
