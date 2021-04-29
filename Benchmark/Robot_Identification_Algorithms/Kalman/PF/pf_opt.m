function [x, X, w, w_kkm1] = pf_opt(t_km1, t_k, X_km1, w_km1, u_km1, y_k, Rv, Rn, robotName, Geometry, Gravity, h_tune, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque)
%#codegen

% PF: Particle Filter
% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
% This code is inspired of the work of Wan, Eric A. and Rudoph van der Merwe and www.anuncommonlab.com

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% X_km1:    Matrix of particles at time k-1|nx-by-np|
% w_km1:    Matrix of weights at time k-1, |1-by-np|
% u_km1:    control input at time k-1
% y_k:      noisy observation at time k
% Rv:       process noise covariance matrix
% Rn:       observation noise covariance matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OUTPUTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x:        Estimated state
% X:        Matrix of particles at time k |nx-by-np|
% w:        Matrix of weights at time k, |1-by-np|
% w_kkm1:   Updated weights, prior to resampling

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Problem dimensions:
[Xdim, Np] = size(X_km1);         % Number of states and number of particles
Ydim = numel(y_k);                % Number of observations

%% Sequential importance sampling step

% Propagate particles:
X_kkm1 = zeros(Xdim, Np);
w_kkm1 = zeros(1, Np);
invRn = inv(Rn);

parfor particle = 1:Np
    % Propagate the particles:
    X_kkm1(:, particle) = process_Model(t_km1, t_k, X_km1(:,particle), u_km1, 1e-2*(w_kkm1(particle))*randn(Xdim,1), robotName, Geometry, Gravity, integrationAlgorithm, dt_control, Xd, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U, useComputedTorque); 
    % Update the weight based on the difference between the true and predicted measurement:
    w_kkm1(particle) = w_km1(particle) * prob_obs_error(invRn, y_k - measurement_Model(X_kkm1(:, particle), zeros(Ydim,1)), X_kkm1(:, particle));
end

% Normalize the weights:
w_kkm1 = w_kkm1 ./ sum(w_kkm1);

% Calculate the updated state:
x = sum(bsxfun(@times, w_kkm1, X_kkm1), 2);

% Resample:
[X_kkm1,w_kkm1] = resampleParticles(X_kkm1, w_kkm1, Np, h_tune, Xdim);

X = X_kkm1;
w = w_kkm1;

end % pf

function [X_k,w] = resampleParticles(X_kkm1, w_kkm1, Np, h_tune, Xdim)
% Randomly pick particles from X_kkm1 according to their weights.
% The weights act as a probability mass function.
% outIndex = zeros(1,Np);

% outIndex = randsample(1:Np, Np, true, w_kkm1);
% X_k = X_kkm1(:, outIndex);
indices = pmfdraw(w_kkm1);
X_k     = X_kkm1(:, indices);

% By virtue of randomly selecting particles, all of the current
% particles now have the same weight of 1/Np.
w = (1/Np)*ones(1,Np);

X_twiddle_k = bsxfun(@minus, X_k, sum(X_k, 2)/Np);
        Sigma       = 1/(Np-1) * (X_twiddle_k * X_twiddle_k.');
        
%         % Alternately, we could have used the weights instead of the
%         % newly selected particles. The expected result is the same; the
%         % actual result may be slightly different.
%         X_twiddle_k = bsxfun(@minus, X_kkm1, x_hat_k);
%         X_twiddle_k = bsxfun(@times, X_twiddle_k, sqrt(w_kkm1));
%         Sigma       = X_twiddle_k * X_twiddle_k.';
        
        % Multiply those perturbations by the tuning parameter and add them
        % to the particles.
%         h_tune = (4/(Np * (Xdim + 2)))^(1/(Xdim + 4));

        [u, s] = svd(Sigma);
        X_k    = X_k + h_tune * u * sqrt(s) * randn(Xdim, Np);
        
end
% Draw n indices randomly from a probability mass function defined by the
% vector p.
function indices = pmfdraw(p, n)

    % Default number of draws is the number of elements in p.
    if nargin < 2, n = length(p); end;

    % Create the discrete cumulative distribution function.
    cdf = cumsum(p);
    
    % If it's not actually a normalized input, normalize it.
    if cdf(end) ~= 1
        cdf = cdf ./ cdf(end);
    end
    
    % Make a bunch of normal, uniform random draws and sort them.
    draws = sort(rand(1, n));
    
    % Find the CDF "bin" containing each draw. This will be the appropriate
    % index of the CDF.
    np      = length(p);
    indices = zeros(1, n);  % Output randomly drawn indices
    ci      = 1;            % Index of PMF bin
    for di = 1:n
        while cdf(ci) < draws(di) && ci < np
            ci = ci + 1;
        end
        indices(di) = ci;
    end
    
end % pmfdraw
function p = prob_obs_error(invRn, dy, varargin)
p = 2*chi2pdf(norm(dy), 2);
% p = exp(-0.5e-6 * dy.'*invRn*dy); % Normal probability density function

end

