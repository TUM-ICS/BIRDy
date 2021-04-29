function [W] = observationMatrix(robotName, paramVectorSize, Geometry, Gravity, Q, Qp, Qpp) %#codegen

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building of the observation matrix W for the current iteration of Beta

[nbDOF,nbSamples]=size(Q);
W = zeros(nbSamples*nbDOF, paramVectorSize);

for i=1:nbSamples
    % Compute the observation matrix:
    switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
%         case 'toyRobot'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_toyRobot(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'SCARA'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_SCARA(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'UR3'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_UR3(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'UR5'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_UR5(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'UR10'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_UR10(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'TX40'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_TX40(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'TX40_uncoupled'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_TX40_uncoupled(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'RX90'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_RX90(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'PUMA560'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_PUMA560(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'REEMC_right_arm'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_REEMC_right_arm(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'RV2SQ'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_RV2SQ(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
%         case 'NAO_right_arm'
%             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_NAO_right_arm(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        otherwise
            W(nbDOF*(i-1)+1:nbDOF*i,:) = feval(sprintf('Regressor_Y_%s',robotName),Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity); % Much slower when compiled !
    end
end
end

