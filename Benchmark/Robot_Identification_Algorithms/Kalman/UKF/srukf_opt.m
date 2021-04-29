function [x, S, K, inov, x_kkm1, S_kkm1, P_xy, S_yy] = srukf_opt(t_km1, t_k, x_km1, u_km1, S, y_k, Sv, Sn, alpha, beta, kappa, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% SRUKF: Square Root Unscented Kalman Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% This code is inspired of the work of Wan, Eric A. and Rudoph van der Merwe

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x_km1:    state mean at time k-1
% S_km1:    matrix square root of state covariance at time k-1
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Sv:       matrix square root of process noise covariance matrix
% Sn:       matrix square root of observation noise covariance matrix

% UKF tunning parameters:
%       0 <  Alpha <= 1
%       0 <= Beta
%       0 <= Kappa <= 3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x:        estimated state
% S:        matrix square root of estimated state covariance
% K:        Kalman Gain
% inov:     inovation signal
% x_kkm1:   predicted state mean
% S_kkm1:   matrix square root of predicted state covariance
% P_xy:     predicted state and observation covariance
% S_yy:     matrix square root of inovation covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Problem dimensions:
Xdim = length(x_km1);           % Number of states
Vdim = size(Sv, 1);             % Number of noise states
Ydim = size(Sn, 1);             % Number of observations
L  = Xdim + Vdim + Ydim;        % Dimension of augmented state
Ns = 2*L+1;                     % Number of sigma points
errorCode = 0;

% Weights:
lambda = alpha ^2 * (L + kappa) - L;
gamma  = sqrt(L + lambda);
W_m_0  = lambda / (L + lambda);
W_c_0  = W_m_0 + 1 - alpha^2 + beta;
sqrtW_c_0 = sqrt(abs(W_c_0));
W_i    = 1/(2*(L + lambda));
sqrtW_c_i = sqrt(W_i);

% Build the augmented system:
S_aug     = blkdiag(S, Sv, Sn);
x_aug_km1 = [x_km1; zeros(Vdim, 1); zeros(Ydim, 1)];

% Create the sigma points:
gamma_S_aug = gamma * S_aug;
X_aug_km1 = [x_aug_km1, repmat(x_aug_km1, 1, L)+gamma_S_aug, repmat(x_aug_km1, 1, L)-gamma_S_aug];

% Predict sigma points and measurements:
Y_kkm1   = zeros(Ydim, Ns);
X_x_kkm1 = zeros(Xdim, Ns);

state = X_aug_km1(1:Xdim, :);
pNoise = X_aug_km1(Xdim+1:Xdim+Vdim, :);
mNoise = X_aug_km1(Xdim+Vdim+1:Xdim+Vdim+Ydim, :);

parfor sp = 1:Ns
    X_x_kkm1(:, sp) = process_Model(t_km1, t_k, state(:,sp), u_km1, pNoise(:,sp), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
    Y_kkm1(:, sp) = measurement_Model(X_x_kkm1(:, sp), mNoise(:, sp));                     
end

% Expected prediction and measurement:
x_kkm1 = W_m_0 * X_x_kkm1(:, 1) + W_i * sum(X_x_kkm1(:, 2:end), 2);
y_kkm1 = W_m_0 * Y_kkm1(:, 1)   + W_i * sum(Y_kkm1(:, 2:end), 2);

% Compute innovation vector:
inov = y_k - y_kkm1;

% Remove expectations from X_x_kkm1 and y_kkm1:
X_x_kkm1 = bsxfun(@minus, X_x_kkm1, x_kkm1);
Y_kkm1   = bsxfun(@minus, Y_kkm1, y_kkm1);

% Compute covariance of the prediction:
[~,S_kkm1] = qr((sqrtW_c_i*X_x_kkm1(:,2:Ns)).',0);      
% QR update of state Cholesky factor. 

if W_c_0>0                                   
    S_kkm1 = cholupdate(S_kkm1,sqrtW_c_0*X_x_kkm1(:,1),'+');
else
    [S_kkm1, errorCode] = cholupdate(S_kkm1,sqrtW_c_0*X_x_kkm1(:,1),'-'); 
end

% Compute covariance of predicted observation:
[~,S_yy] = qr((sqrtW_c_i*Y_kkm1(:,2:Ns)).',0);      % QR update of state Cholesky factor. 

if W_c_0>0                                    
    S_yy = cholupdate(S_yy,sqrtW_c_0*Y_kkm1(:,1),'+');
else
    [S_yy, errorCode] = cholupdate(S_yy,sqrtW_c_0*Y_kkm1(:,1),'-'); 
end

S_yy = S_yy.';

% Compute covariance of predicted observation and predicted state:
P_xy =   (W_c_0 * X_x_kkm1(:, 1)) * Y_kkm1(:, 1).' + W_i * (X_x_kkm1(:, 2:end) * Y_kkm1(:, 2:end).');

% Kalman gain:
K = (P_xy/S_yy.')/S_yy;

% State correction:
x = x_kkm1 + K*inov;
U = K*S_yy;

% Covariance correction:
for j=1:Ydim
    [S_kkm1, errorCode] = cholupdate(S_kkm1,U(:,j),'-');
end
S = S_kkm1.';

if errorCode ~=0
    disp("S_kkm1 is close to non-positive-definiteness!")
end

end % srukf
