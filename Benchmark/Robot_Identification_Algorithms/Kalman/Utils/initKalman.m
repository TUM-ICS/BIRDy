function [x, P, S, Rv, Rn, Sv, Sn, Particules, w, alpha, beta, kappa, h, paramSize, pNoiseAdaptParams, State, Covariance, pNoise] = initKalman(robot, benchmarkSettings, Tau_decim, Qpp_decim, Qp_decim, Q_decim, expNb, Beta_0, optionsKF)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

% Initial augmented state estimate:
x = [Qp_decim(:,1);Q_decim(:,1);Beta_0];

% Sizes:
paramSize = robot.paramVectorSize;
[stateAugSize,~]=size(x);
stateSize = stateAugSize-paramSize;

State =  zeros(ceil(benchmarkSettings.nbSamples/optionsKF.samplingFactor), stateAugSize);
State(1,:) = [Qp_decim(:,1).' Q_decim(:,1).' Beta_0.'];
Covariance =  zeros(stateAugSize, stateAugSize, ceil(benchmarkSettings.nbSamples/optionsKF.samplingFactor));
pNoise =  zeros(ceil(benchmarkSettings.nbSamples/optionsKF.samplingFactor), 1);

% UKF tunning parameters:
%       0 <  Alpha <= 1
%       0 <= Beta
%       0 <= Kappa <= 3

alpha = 1e-1;
beta  = 2;
kappa = 0;

% CDKF tunning parameter:
% h >= 1    scalar central difference interval size

h = sqrt(3);

% P = 0.1*eye(stateAugSize); % Initial state covariance = covariance of the noise
% P(stateSize+1:end,stateSize+1:end) = 0.02*eye(paramSize); % Parameters initial covariance
% Rn = diag(robot.numericalParameters.sd_q)*diag(robot.numericalParameters.sd_q)*eye(robot.nbDOF); % Measurement noise covariance
% Rv = 1e-7*eye(stateAugSize); % Process noise covariance
% Rv(stateSize+1:end,stateSize+1:end) = 1e-5*eye(paramSize); % Process noise covariance

P = 1*eye(stateAugSize); % Initial state covariance = covariance of the noise
epsilon0 = 1e-3*ones(size(Beta_0));
P(stateSize+1:end,stateSize+1:end) = 0.15*diag(abs(Beta_0)+epsilon0); % Parameters initial covariance

Rn = diag(robot.numericalParameters.sd_q.^2); % Measurement noise covariance

Rv = diag([0.05*robot.numericalParameters.sd_tau.^2;0.05*benchmarkSettings.dt*robot.numericalParameters.sd_tau.^2;1e-6*ones(paramSize,1)]);
% stateSize2 = stateSize/2;
% % Process noise covariance
% sigmaProcess = 1e-7;
% Gamma = [1; benchmarkSettings.dt]*[1; benchmarkSettings.dt].';
% Rv = 1e-7*eye(stateAugSize); % Process noise covariance
% Rv(stateSize+1:end,stateSize+1:end) = 1e-4*eye(paramSize); % Process noise covariance
% 
% for i=1:size(Gamma,1)
%     for j=1:size(Gamma,2)
%             Rv(stateSize2*(i-1)+1:stateSize2*i,stateSize2*(j-1)+1:stateSize2*j) = sigmaProcess*Gamma(i,j)*eye(stateSize2);
%     end
% end

% In case a particle filter is being used, initialize the particles:
if strcmp(optionsKF.type, 'pf') || strcmp(optionsKF.type, 'sppf')
    Particules = repmat(x, 1, optionsKF.nbParticules);
    for i=1:optionsKF.nbParticules
        Particules(:,i) = Particules(:,i) + optionsKF.initialBias*randn(size(x)).*x;
    end
    w = repmat(1/optionsKF.nbParticules, 1, optionsKF.nbParticules);
else
    Particules = 0;
    w = 0;
end

pNoiseAdaptParams.annealFactor = 1-benchmarkSettings.dt;  
pNoiseAdaptParams.variance = 1e-8;

[U,Sig,V] = svd(P,0);
S = U*sqrt(Sig)*V.';

[Uv,Sigv,Vv] = svd(Rv,0);
Sv = Uv*sqrt(Sigv)*Vv.';

[Un,Sign,Vn] = svd(Rn,0);
Sn = Un*sqrt(Sign)*Vn.';

% Init Covariance Annealing Parameters
dv = diag(Rv);  % grab diagonal
pNoiseAdaptParams.cov = dv(end-paramSize+1:end); % extract the part of the diagonal that relates to the 'parameter section'

end
