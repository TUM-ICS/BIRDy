function [costVariable] = compute_ML_Cost(robotName, Beta, Q_decim, Qp_decim, Qpp_decim, Tau_decim, Sigma2, Geometry, Gravity, squaredCost) %#codegen

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%

nbDOF = size(Tau_decim,1);
nbSamples = size(Tau_decim,2);
Err = zeros(nbDOF*nbSamples,1);
jacobianOptions.epsVal = 1e-7;

G = @(X_k) errorFunction(robotName, nbDOF, X_k, Beta, Geometry, Gravity);

for k=1:nbSamples
    X_k = [Q_decim(:,k);Qp_decim(:,k);Qpp_decim(:,k);Tau_decim(:,k)];
    B_k = computeNumJacobian(X_k, G, jacobianOptions);
    b_k = -G(X_k);
    [U,S,V] = svd(B_k*Sigma2*B_k',0);
    Err(nbDOF*(k-1)+1:nbDOF*k) = U*sqrt(inv(S))*V.'*b_k;
end

if squaredCost == true
    costVariable = 0.5*Err.'*Err;
else
    costVariable = Err';
end

end

function [ Error ] = errorFunction(robotName, nbDOF, X_k, Beta, Geometry, Gravity)

Q = X_k(1:nbDOF);
Qp = X_k(nbDOF+1:2*nbDOF);
Qpp = X_k(2*nbDOF+1:3*nbDOF);
Tau = X_k(3*nbDOF+1:4*nbDOF);

% Compute the Tau error
switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, RV2SQ, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
    %         case 'toyRobot'
    %             Yr = Regressor_Y_toyRobot(Q, Qp, Qpp, Geometry, Gravity);
    %         case 'SCARA'
    %             Yr = Regressor_Y_SCARA(Q, Qp, Qpp, Geometry, Gravity);
    %         case 'UR3'
    %             Yr = Regressor_Y_UR3(Q, Qp, Qpp, Geometry, Gravity);
    %     case 'UR5'
    %         Yr = Regressor_Y_UR5(Q, Qp, Qpp, Geometry, Gravity);
    %         case 'UR10'
    %             Error = Regressor_Y_UR10(Q, Qp, Qpp, Geometry, Gravity);
    case 'TX40'
        Yr = Regressor_Y_TX40(Q, Qp, Qpp, Geometry, Gravity);
    case 'TX40_uncoupled'
        Yr = Regressor_Y_TX40_uncoupled(Q, Qp, Qpp, Geometry, Gravity);
    case 'RV2SQ'
        Yr = Regressor_Y_RV2SQ(Q, Qp, Qpp, Geometry, Gravity);
        %         case 'RX90'
        %             Yr = Regressor_Y_RX90(Q, Qp, Qpp, Geometry, Gravity);
        %         case 'PUMA560'
        %             Yr = Regressor_Y_PUMA560(Q, Qp, Qpp, Geometry, Gravity);
%     case 'REEMC_right_arm'
%         Yr = Regressor_Y_REEMC_right_arm(Q, Qp, Qpp, Geometry, Gravity);
        %         case 'NAO_right_arm'
        %             Yr = Regressor_Y_NAO_right_arm(Q, Qp, Qpp, Geometry, Gravity);
    otherwise
        Yr = feval(sprintf('Regressor_Y_%s',robotName),Q , Qp, Qpp, Geometry, Gravity); % Much slower when compiled !
end

Error = Yr*Beta-Tau;
end
