function [x, S, K, inov, x_kkm1, S_kkm1, P_xy, S_yy] = srcdkf_opt(t_km1, t_k, x_km1, u_km1, S, y_k, Sv, Sn, h, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% CDKF: Square Root Central Difference Kalman Filter
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

% SRCDKF tunning parameters:
% h >= 1    scalar central difference interval size

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
L_v  = Xdim + Vdim;             % Dimension of augmented state
L_n  = Xdim + Ydim;             % Dimension of augmented state
Ns_v = 2*L_v+1;                 % Number of sigma points
Ns_n = 2*L_n+1;                 % Number of sigma points

% Weights:
h2 = h^2;
W_m_0_v = (h2 - L_v)/h2;
W_m_0_n = (h2 - L_n)/h2;
W_m_i = 1/(2*h2);
W_c1_i = 1/(2*h);
W_c2_i = sqrt(h2-1)/(2*h2);

%% Calculate sigma-points for time-update:

% Build the augmented system:
S_aug_v = blkdiag(S, Sv);
h_S_aug_v = h*S_aug_v;
x_aug_v_km1 = [x_km1; zeros(Vdim, 1)];
X_aug_v_km1 = [x_aug_v_km1, repmat(x_aug_v_km1, 1, L_v) + h_S_aug_v, repmat(x_aug_v_km1, 1, L_v) - h_S_aug_v];

%% Time-update equations:

X_x_kkm1 = zeros(Xdim, Ns_v);
state = X_aug_v_km1(1:Xdim, :);
pNoise = X_aug_v_km1(Xdim+1:Xdim+Vdim, :);

parfor sp = 1:Ns_v
    X_x_kkm1(:, sp) = process_Model(t_km1, t_k, state(:,sp), u_km1, pNoise(:,sp), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
end

% Expected prediction:
x_kkm1 = W_m_0_v * X_x_kkm1(:, 1) + W_m_i * sum(X_x_kkm1(:, 2:end), 2);

% Compute covariance of the prediction:
X_x_kkm1mx = W_c1_i * (X_x_kkm1(:,2:L_v+1)-X_x_kkm1(:,L_v+2:end));
X_x_kkm1px = W_c2_i * bsxfun(@minus,X_x_kkm1(:,2:L_v+1)+X_x_kkm1(:,L_v+2:Ns_v), 2*X_x_kkm1(:,1));
[~,S_kkm1] = qr([X_x_kkm1mx, X_x_kkm1px].',0);
S_kkm1 = S_kkm1.';


%% Calculate sigma-points for measurement-update:

% Build the augmented system:
S_aug_n = blkdiag(S_kkm1, Sn);
x_aug_n_km1 = [x_kkm1; zeros(Ydim, 1)];
h_S_aug_n = h*S_aug_n;
X_aug_n_km1 = [x_aug_n_km1, repmat(x_aug_n_km1, 1, L_n) + h_S_aug_n, repmat(x_aug_n_km1, 1, L_n) - h_S_aug_n];

%% Measurement-update equations:

Y_kkm1 = zeros(Ydim, Ns_n);
mState = X_aug_n_km1(1:Xdim, :);
mNoise = X_aug_n_km1(Xdim+1:Xdim+Ydim, :);

parfor sp = 1:Ns_n
    Y_kkm1(:, sp) = measurement_Model(mState(:, sp), mNoise(:, sp));
end

% Expected measurement:
y_kkm1 = W_m_0_n * Y_kkm1(:, 1) + W_m_i * sum(Y_kkm1(:, 2:end), 2);

% Compute covariance of predicted measurement:
Y_kkm1mx = W_c1_i * (Y_kkm1(:,2:L_n+1)-Y_kkm1(:,L_n+2:end));
Y_kkm1px = W_c2_i * bsxfun(@minus,Y_kkm1(:,2:L_n+1)+Y_kkm1(:,L_n+2:Ns_n), 2*Y_kkm1(:,1));
[~,S_yy] = qr([Y_kkm1mx, Y_kkm1px].',0);
S_yy = S_yy.';

% Compute covariance of predicted observation and predicted state:
Syx1 = Y_kkm1mx(:,1:Xdim);
Syw1 = Y_kkm1mx(:,Xdim+1:end);
P_xy = S_kkm1*Syx1.';

% Compute innovation vector:
inov = y_k - y_kkm1;

% Compute the Kalman gain:
K = (P_xy/S_yy.')/S_yy;

% State correction:
x = x_kkm1 + K*inov;

% Covariance correction:
[~,S] = qr([S_kkm1-K*Syx1 K*Syw1 K*Y_kkm1px].',0); % Inspired by the Rebel toolkit implementation of Wan, Eric A. and Rudoph van der Merwe
S = S.';

end % srcdkf
