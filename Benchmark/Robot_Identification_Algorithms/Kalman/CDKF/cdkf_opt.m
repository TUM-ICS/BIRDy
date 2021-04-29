function [x, P, K, inov, x_kkm1, P_kkm1, P_xy, P_yy] = cdkf_opt(t_km1, t_k, x_km1, u_km1, P, y_k, Rv, Rn, h, sigmaComputeMethod, robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% CDKF: Central Difference Kalman Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% This code is inspired of the work of Wan, Eric A. and Rudoph van der Merwe

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x_km1:    state mean at time k-1
% P_km1:    state covariance at time k-1
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Rv:       process noise covariance matrix
% Rn:       observation noise covariance matrix

% CDKF tunning parameters:
% h >= 1    scalar central difference interval size

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
Ns_x = 2*Xdim+1;		% Number of sigma points (first set)
Ns_v = 2*Vdim;                	% Number of sigma points (second set)
Ns_n = 2*Ydim;                	% Number of sigma points (third set)

% Weights:
h2 = h^2;
h4 = h^4;
W_m_0_v = (h2 - Xdim - Vdim)/h2;
W_m_0_n = (h2 - Xdim - Ydim)/h2;
W_m_i = 1/(2*h2);
W_c1_i = 1/(4*h2);
W_c2_i = (h2-1)/(4*h4);

%% Calculate sigma-points for time-update:

% Build the augmented system:

switch sigmaComputeMethod
    case false
        % Compute sqrt(P) with Cholesky factorization
        [S_x, chk] = chol(P,'lower');
        if chk > 0
            error('CDKF: Error in Cholesky factorization at time update.')
        end
        h_S_x = h * S_x;
        % Compute sqrt(Rv) with Cholesky factorization
        [S_v, chk] = chol(Rv,'lower');
        if chk > 0
            error('CDKF: Error in Cholesky factorization at time update.')
        end
        h_S_v = h * S_v;
    case true
        % Compute sqrt(P) with SVD in case there is a 0 eigenvalue
        [U_x, Sigma_x, V_x] = svd(P,0);
        h_S_x = h * U_x * sqrt(Sigma_x) * V_x.';
        % Compute sqrt(Rv) with SVD in case there is a 0 eigenvalue
        [U_v, Sigma_v, V_v] = svd(Rv,0);
        h_S_v = h * U_v * sqrt(Sigma_v) * V_v.';
    otherwise
        error("CDKF: Please select a correct sigma point computation method !");
end

X_km1 = [x_km1, repmat(x_km1, 1, Xdim) + h_S_x, repmat(x_km1, 1, Xdim) - h_S_x];
V_km1 = [h_S_v, -h_S_v];

%% Time-update equations:

X_x_kkm1 = zeros(Xdim, Ns_x);
parfor sp = 1:Ns_x
    X_x_kkm1(:, sp) = process_Model(t_km1, t_k, X_km1(:,sp), u_km1, zeros(Vdim,1), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
end
X_v_kkm1 = zeros(Xdim, Ns_v);
parfor sp = 1:Ns_v
    X_v_kkm1(:, sp) = process_Model(t_km1, t_k, x_km1, u_km1, V_km1(:,sp), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
end

% Expected prediction:
x_kkm1 = W_m_0_v * X_x_kkm1(:, 1) + W_m_i * sum(X_x_kkm1(:, 2:end), 2) + W_m_i * sum(X_v_kkm1, 2);

% Compute covariance of the prediction:
X_x_kkm1mx = W_c1_i * (X_x_kkm1(:,2:Xdim+1)-X_x_kkm1(:,Xdim+2:end))*(X_x_kkm1(:,2:Xdim+1)-X_x_kkm1(:,Xdim+2:end)).' + W_c1_i * (X_v_kkm1(:,1:Vdim)-X_v_kkm1(:,Vdim+1:end))*(X_v_kkm1(:,1:Vdim)-X_v_kkm1(:,Vdim+1:end)).';
X_x_kkm1px =  W_c2_i * (bsxfun(@minus,X_x_kkm1(:,2:Xdim+1)+X_x_kkm1(:,Xdim+2:end), 2*X_x_kkm1(:,1)))*(bsxfun(@minus,X_x_kkm1(:,2:Xdim+1)+X_x_kkm1(:,Xdim+2:end), 2*X_x_kkm1(:,1))).' + W_c2_i * (bsxfun(@minus,X_v_kkm1(:,1:Vdim)+X_v_kkm1(:,Vdim+1:end), 2*X_x_kkm1(:,1)))*(bsxfun(@minus,X_v_kkm1(:,1:Vdim)+X_v_kkm1(:,Vdim+1:end), 2*X_x_kkm1(:,1))).';
P_kkm1 = X_x_kkm1mx + X_x_kkm1px;


%% Calculate sigma-points for measurement-update:

% Build the augmented system:

switch sigmaComputeMethod
    case false
        % Compute sqrt(P_kkm1) with Cholesky factorization
        [S_xkkm1, chk] = chol(P_kkm1,'lower');
        if chk > 0
            error('CDKF: Error in Cholesky factorization at time update.')
        end
        h_S_xkkm1 = h * S_xkkm1;
        % Compute sqrt(Rn) with Cholesky factorization
        [S_n, chk] = chol(Rn,'lower');
        if chk > 0
            error('CDKF: Error in Cholesky factorization at time update.')
        end
        h_S_n = h * S_n;
    case true
        % Compute sqrt(P_kkm1) with SVD in case there is a 0 eigenvalue
        [U_xkkm1, Sigma_xkkm1, V_xkkm1] = svd(P_kkm1,0);
        h_S_xkkm1 = h * U_xkkm1 * sqrt(Sigma_xkkm1) * V_xkkm1.';
        % Compute sqrt(Rn) with SVD in case there is a 0 eigenvalue
        [U_n, Sigma_n, V_n] = svd(Rn,0);
        h_S_n = h * U_n * sqrt(Sigma_n) * V_n.';
    otherwise
        error("CDKF: Please select a correct sigma point computation method !");
end

X_kkm1 = [x_kkm1, repmat(x_kkm1, 1, Xdim) + h_S_xkkm1, repmat(x_kkm1, 1, Xdim) - h_S_xkkm1];
N_km1 = [h_S_n, -h_S_n];

%% Measurement-update equations:

Y_x_kkm1 = zeros(Ydim, Ns_x);
parfor sp = 1:Ns_x
    Y_x_kkm1(:, sp) = measurement_Model(X_kkm1(:,sp), zeros(Ydim,1));
end
Y_n_kkm1 = zeros(Ydim, Ns_n);
parfor sp = 1:Ns_n
    Y_n_kkm1(:, sp) = measurement_Model(x_kkm1, N_km1(:,sp));
end

% Expected measurement:
y_kkm1 = W_m_0_n * Y_x_kkm1(:, 1) + W_m_i * sum(Y_x_kkm1(:, 2:end), 2) + W_m_i * sum(Y_n_kkm1, 2);

% Compute covariance of the prediction:
Y_kkm1mx = W_c1_i * (Y_x_kkm1(:,2:Xdim+1)-Y_x_kkm1(:,Xdim+2:end))*(Y_x_kkm1(:,2:Xdim+1)-Y_x_kkm1(:,Xdim+2:end)).' + W_c1_i * (Y_n_kkm1(:,1:Ydim)-Y_n_kkm1(:,Ydim+1:end))*(Y_n_kkm1(:,1:Ydim)-Y_n_kkm1(:,Ydim+1:end)).';
Y_kkm1px =  W_c2_i * (bsxfun(@minus,Y_x_kkm1(:,2:Xdim+1)+Y_x_kkm1(:,Xdim+2:end), 2*Y_x_kkm1(:,1)))*(bsxfun(@minus,Y_x_kkm1(:,2:Xdim+1)+Y_x_kkm1(:,Xdim+2:end), 2*Y_x_kkm1(:,1))).' + W_c2_i * (bsxfun(@minus,Y_n_kkm1(:,1:Ydim)+Y_n_kkm1(:,Ydim+1:end), 2*Y_x_kkm1(:,1)))*(bsxfun(@minus,Y_n_kkm1(:,1:Ydim)+Y_n_kkm1(:,Ydim+1:end), 2*Y_x_kkm1(:,1))).';
P_yy = Y_kkm1mx + Y_kkm1px;

% Compute covariance of predicted observation and predicted state:
P_xy = W_m_i*h_S_xkkm1*(Y_x_kkm1(:, 2:Xdim+1)-Y_x_kkm1(:, Xdim+2:end)).';

% Compute innovation vector:
inov = y_k - y_kkm1;

% Compute the Kalman gain:
K = P_xy / P_yy;

% State correction:
x = x_kkm1 + K*inov;

% Covariance correction:
P = P_kkm1 - K * P_yy * K.';

end % cdkf
