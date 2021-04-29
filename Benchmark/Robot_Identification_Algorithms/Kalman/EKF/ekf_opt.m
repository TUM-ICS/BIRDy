function [x, P, K, inov, x_kkm1, P_kkm1, P_xy, P_yy] = ekf_opt(t_km1, t_k, x_km1, u_km1, P_km1, y_k, Rv, Rn, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% EKF: Extended Kalman Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x_km1:    state mean at time k-1
% P_km1:    state covariance at time k-1
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Rv:       process noise covariance matrix
% Rn:       observation noise covariance matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% x:        estimated state
% P:        estimated state covariance
% K:        Kalman Gain
% inov:     inovation signal
% x_kkm1:   predicted state mean
% P_kkm1:   predicted state covariance
% P_xy:     predicted state and observation covariance
% P_yy:     inovation covariance  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical Jacobian options:
jacobianOptions.epsVal = 1e-7;

% Problem dimensions:
% Xdim = length(x_km1);           % Number of states
Vdim = size(Rv, 1);             % Number of noise states
Ydim = size(Rn, 1);             % Number of observations

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

% Compute innovation vector:
inov = y_k - y_kkm1;

% Compute covariance of the prediction:
P_kkm1 = F*P_km1*F.' + G*Rv*G.';

% Compute covariance of predicted observation and predicted state:
P_xy = P_kkm1*H.';

% Compute covariance of predicted observation:
P_yy =   H*P_kkm1*H.' + D*Rn*D.';

% Kalman gain:
K = P_xy / P_yy;

% State correction:
x = x_kkm1 + K*inov;

% Covariance correction:
% P = P_kkm1 - K*P_yy*K.';
P = P_kkm1 - K*H*P_kkm1;

end % ekf
