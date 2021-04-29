function [x, P, K, inov, x_kkm1, P_kkm1, P_xy, P_yy] = ukf_opt(t_km1, t_k, x_km1, u_km1, P, y_k, Rv, Rn, alpha, beta, kappa, sigmaComputeMethod, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% UKF: Unscented Kalman Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% This code is inspired of the work of Wan, Eric A. and Rudoph van der Merwe and www.anuncommonlab.com

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x_km1:    state mean at time k-1
% P_km1:    state covariance at time k-1
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Rv:       process noise covariance matrix
% Rn:       observation noise covariance matrix

% UKF tunning parameters:
%       0 <  Alpha <= 1
%       0 <= Beta
%       0 <= Kappa <= 3

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

% Problem dimensions:
Xdim = length(x_km1);           % Number of states
Vdim = size(Rv, 1);             % Number of noise states
Ydim = size(Rn, 1);             % Number of observations
L  = Xdim + Vdim + Ydim;        % Dimension of augmented state
Ns = 2*L+1;                     % Number of sigma points

% Weights:
lambda = alpha ^2 * (L + kappa) - L;
gamma  = sqrt(L + lambda);
W_m_0  = lambda / (L + lambda);
W_c_0  = W_m_0 + 1 - alpha^2 + beta;
W_i    = 1/(2*(L + lambda));

% Build the augmented system:
P_aug     = blkdiag(P, Rv, Rn);
x_aug_km1 = [x_km1; zeros(Vdim, 1); zeros(Ydim, 1)];

% Create the sigma points:
switch sigmaComputeMethod
    case false
        % Compute sqrt(P_aug) with Cholesky factorization
        gamma_sqrt_P_aug = gamma * chol(P_aug).';
    case true
        % Compute sqrt(P_aug) with SVD in case there is a 0 eigenvalue
        [U, Sigma, V] = svd(P_aug,0);
        gamma_sqrt_P_aug = gamma * U * sqrt(Sigma) * V.';
    otherwise
    error("UKF: Please select a correct sigma point computation method !");
end

X_aug_km1 = [x_aug_km1, repmat(x_aug_km1, 1, L) + gamma_sqrt_P_aug, repmat(x_aug_km1, 1, L) - gamma_sqrt_P_aug];

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
P_kkm1 =   (W_c_0 * X_x_kkm1(:, 1)) * X_x_kkm1(:, 1).' + W_i * (X_x_kkm1(:, 2:end) * X_x_kkm1(:, 2:end).');

% Compute covariance of predicted observation:
P_yy =   (W_c_0 * Y_kkm1(:, 1)) * Y_kkm1(:, 1).' + W_i * (Y_kkm1(:, 2:end) * Y_kkm1(:, 2:end).');

% Compute covariance of predicted observation and predicted state:
P_xy =   (W_c_0 * X_x_kkm1(:, 1)) * Y_kkm1(:, 1).' + W_i * (X_x_kkm1(:, 2:end) * Y_kkm1(:, 2:end).');

% Compute the Kalman gain:
K = P_xy / P_yy;

% State correction:
x = x_kkm1 + K*inov;

% Covariance correction:
P = P_kkm1 - K * P_yy * K.';

end % ukf
