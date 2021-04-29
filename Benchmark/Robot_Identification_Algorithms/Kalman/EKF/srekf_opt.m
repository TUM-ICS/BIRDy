function [x, S, K, inov, x_kkm1, S_kkm1, S_xy, S_yy] = srekf_opt(t_km1, t_k, x_km1, u_km1, S_km1, y_k, Sv, Sn, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% SREKF: Square Root Extended Kalman Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x_km1:    state mean at time k-1
% S_km1:    matrix square root of state covariance at time k-1
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Sv:       matrix square root of process noise covariance matrix
% Sn:       matrix square root of observation noise covariance matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% x:        estimated state
% S:        matrix square root of estimated state covariance
% K:        Kalman Gain
% inov:     inovation signal
% x_kkm1:   predicted state mean
% S_kkm1:   matrix square root of predicted state covariance
% S_xy:     predicted state and observation covariance
% S_yy:     matrix square root of inovation covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical Jacobian options:
jacobianOptions.epsVal = 1e-7;

% Problem dimensions:
% Xdim = length(x_km1);           % Number of states
Vdim = size(Sv, 1);             % Number of noise states
Ydim = size(Sn, 1);             % Number of observations

% Expected prediction and measurement:
x_kkm1 = process_Model(t_km1, t_k, x_km1, u_km1, zeros(Vdim,1), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque);   
y_kkm1 = measurement_Model(x_kkm1,zeros(Ydim,1));                

% Compute the Jacobian using the state at k-1:
hF = @(X) process_Model(t_km1, t_k, X, u_km1, zeros(Vdim,1), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
hG = @(V) process_Model(t_km1, t_k, x_km1, u_km1, V, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
hH = @(X) measurement_Model(X,zeros(Ydim,1));
hD = @(Y) measurement_Model(x_kkm1,Y);
F = computeNumJacobian(x_km1, hF, jacobianOptions);
G = computeNumJacobian(zeros(Vdim,1), hG, jacobianOptions);
H = computeNumJacobian(x_kkm1, hH, jacobianOptions); 
D = computeNumJacobian(zeros(Ydim,1), hD, jacobianOptions); 

% Compute covariance of the prediction:
[~,S_kkm1] = qr([S_km1.'*F.'; Sv.'*G.'],0);
S_kkm1 = S_kkm1.';

[~,R] = qr([Sn.'*D.' zeros(Ydim, Vdim); S_kkm1.'*H.' S_kkm1.'],0);

% Compute covariance of predicted observation:
S_yy = R(1:Ydim,1:Ydim).';

% Compute covariance of predicted observation and predicted state:
S_xy = R(1:Ydim,Ydim+1:end).';

% Compute innovation vector:
inov = y_k - y_kkm1;

% Kalman gain:
K = S_xy/S_yy;

% State correction:
x = x_kkm1 + K*inov;

% Covariance correction:
S = R(Ydim+1:end,Ydim+1:end).';

end % srekf
