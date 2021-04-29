function [Chi_DG_TLS, cond, sig_min] = Drive_Gains_identification(robot, experimentDataStruct, benchmarkSettings, optionsDriveGainsId)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using the DriveGainsId method.

%% Data Decimation and Filtering + Populating the Observation Matrix W and Torque Vector:
K = ones(6,1);
% 

K = [-0.0692;
   -0.0843;
    0.0413;
   -0.0172;
   -0.0169;
   -0.0353];
% K = [-0.0645;
%    -0.0843;
%     0.0413;
%    -0.0172;
%    -0.0170;
%    -0.0380];
% K = [-0.0347;
%     -0.0845;
%     0.0412;
%     -0.0165;
%     -0.0173;
%     -0.0145]; % filtrage 10

% K = [-0.0467;
%    -0.0885;
%     0.0410;
%    -0.0169;
%    -0.0174;
%    -0.0026]; filtage 20

% Compute the observation matrices of the experiment without end-effector mass:
[~, ~, ~, ~, ~, time_no_load1, I_decim_no_load1, Qpp_decim_no_load1, Qp_decim_no_load1, Q_decim_no_load1, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, 1, 'DriveGainsId');
% [~, ~, ~, ~, ~, time_no_load2, I_decim_no_load2, Qpp_decim_no_load2, Qp_decim_no_load2, Q_decim_no_load2, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, 2, 'DriveGainsId');

% W_no_load = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, [Q_decim_no_load1 Q_decim_no_load2], [Qp_decim_no_load1 Qp_decim_no_load2], [Qpp_decim_no_load1 Qpp_decim_no_load2]); % Observation matrix for the base parameters
% V_no_load = driveCurrentMatrix([I_decim_no_load1 I_decim_no_load2]);
% Y_no_load = torqueVector(K.*[I_decim_no_load1 I_decim_no_load2]);
W_no_load = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim_no_load1, Qp_decim_no_load1, Qpp_decim_no_load1); % Observation matrix for the base parameters
V_no_load = driveCurrentMatrix(I_decim_no_load1);
Y_no_load = torqueVector(K.*I_decim_no_load1);

% Compute the observation matrices of the experiment with end-effector mass:
[~, ~, ~, ~, ~, time_load1, I_decim_load1, Qpp_decim_load1, Qp_decim_load1, Q_decim_load1, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, 2, 'DriveGainsId');
% [~, ~, ~, ~, ~, time_load2, I_decim_load2, Qpp_decim_load2, Qp_decim_load2, Q_decim_load2, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsDriveGainsId, 4, 'DriveGainsId');

% W_load = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, [Q_decim_load1 Q_decim_load2], [Qp_decim_load1 Qp_decim_load2], [Qpp_decim_load1 Qpp_decim_load2]); % Observation matrix for the base parameters
% W_ef_load = observationMatrix_load(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, [Q_decim_load1 Q_decim_load2], [Qp_decim_load1 Qp_decim_load2], [Qpp_decim_load1 Qpp_decim_load2]); % Observation matrix for the end-effector-mass parameters
% V_load = driveCurrentMatrix([I_decim_load1 I_decim_load2]);
% Y_load = torqueVector(K.*[I_decim_load1 I_decim_load2]);
W_load = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim_load1, Qp_decim_load1, Qpp_decim_load1); % Observation matrix for the base parameters
W_ef_load = observationMatrix_load(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim_load1, Qp_decim_load1, Qpp_decim_load1); % Observation matrix for the end-effector-mass parameters
V_load = driveCurrentMatrix(I_decim_load1);
Y_load = torqueVector(K.*I_decim_load1);

% figure()
% plot(Q_decim_no_load1.','b')
% hold on
% plot(Q_decim_no_load2.','r')
% hold on
% plot(Q_decim_load1.','g')
% hold on
% plot(Q_decim_load2.','m')
% hold on
% 
% figure()
% plot(Qp_decim_no_load1.','b')
% hold on
% plot(Qp_decim_no_load2.','r')
% hold on
% plot(Qp_decim_load1.','g')
% hold on
% plot(Qp_decim_load2.','m')
% hold on
% 
% figure()
% plot(Qpp_decim_no_load1.','b')
% hold on
% plot(Qpp_decim_no_load2.','r')
% hold on
% plot(Qpp_decim_load1.','g')
% hold on
% plot(Qpp_decim_load2.','m')
% hold on

% Parameters of the mass attached to the robot end-effector during the
% experiments. These parameters are known with a good level of accuracy.

M = 1.73058;
X = 0;
Y = 0;
Z = 2*0.01989136;
MX = M*X;
MY = M*Y;
MZ = M*Z;
XX = 0.00096429;
XY = 0;
XZ = 0;
YY = 0.001719;
YZ = 0;
ZZ = 0.00167572;

Beta_load = [MX; MZ; MY; XY-2*M*X*Y; XZ-2*M*X*Z; YZ-2*M*Y*Z; XX + 2*M*Y.^2 + 2*M*Z.^2; ZZ + 2*M*X.^2 + 2*M*Y.^2; YY + 2*M*X.^2 + 2*M*Z.^2; M];


% OLS solution:
Beta_LS = W_no_load\Y_no_load;

[Beta_LS benchmarkSettings.Beta_obj]

% WLS solution:
[sig2_error] = getErrorCovariance(W_no_load, Y_no_load, robot.nbDOF);
s = 1./sig2_error;
% Omega = repmat(s,numel(time_no_load1)+0*numel(time_no_load2),1);
Omega = repmat(s,numel(time_no_load1),1);
B = bsxfun(@times,W_no_load,sqrt(Omega));
C = bsxfun(@times,W_no_load,Omega);
Beta_WLS = (B.'*B)\C.'*Y_no_load;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Total observation matrix, as defined in Gautier and Briot LS (2011):
Beta_DG_LS = zeros(robot.nbDOF,1);
for ii = 1:robot.nbDOF
    
    W_tot_LS = [W_no_load(ii:robot.nbDOF:end,:) zeros(size(W_no_load,1)/robot.nbDOF,1);...
        W_load(ii:robot.nbDOF:end,:) W_ef_load(ii:robot.nbDOF:end,:)*Beta_load];
    
    XX = W_tot_LS\[Y_no_load(ii:robot.nbDOF:end); Y_load(ii:robot.nbDOF:end)];
    Beta_DG_LS(ii) = 1/XX(end);
end

% Total observation matrix, as defined in Gautier and Briot TLS (2011):
W_tot_TLS = [-W_no_load  V_no_load   zeros(size(W_ef_load));...
             -W_load     V_load     -W_ef_load];

[U,S,V]=svd(W_tot_TLS,0);
[sig_min, br] = min(diag(S));
cond = max(diag(S))/min(diag(S));

W_tot_hat = W_tot_TLS - S(br,br)*U(:,br)*V(:,br).';
Chi_DG_TLS = (M/V(end,br))*V(:,br);
Beta_TLS = Chi_DG_TLS(1:robot.paramVectorSize);
Beta_DG_TLS = Chi_DG_TLS(robot.paramVectorSize+1:robot.paramVectorSize+robot.nbDOF);

err_LS =  Beta_LS-benchmarkSettings.Beta_obj;
err_LS2 = err_LS'*err_LS

err_TLS =  Beta_TLS-benchmarkSettings.Beta_obj;
err_TLS2 = err_TLS'*err_TLS

Beta_DG_LS
Beta_DG_TLS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Tau_computed = zeros(robot.nbDOF,numel(time_no_load1));
Tau_computed_LS = zeros(robot.nbDOF,numel(time_no_load1));
Tau_computed_WLS = zeros(robot.nbDOF,numel(time_no_load1));
Tau_computed_TLS = zeros(robot.nbDOF,numel(time_no_load1));
for i=1:numel(time_no_load1)
    Tau_computed(:,i) = Regressor_Y_RV2SQ(Q_decim_no_load1(:,i), Qp_decim_no_load1(:,i), Qpp_decim_no_load1(:,i), robot.numericalParameters.Geometry, robot.numericalParameters.Gravity)*benchmarkSettings.Beta_obj;
    Tau_computed_LS(:,i) = Regressor_Y_RV2SQ(Q_decim_no_load1(:,i), Qp_decim_no_load1(:,i), Qpp_decim_no_load1(:,i), robot.numericalParameters.Geometry, robot.numericalParameters.Gravity)*Beta_LS;
    Tau_computed_WLS(:,i) = Regressor_Y_RV2SQ(Q_decim_no_load1(:,i), Qp_decim_no_load1(:,i), Qpp_decim_no_load1(:,i), robot.numericalParameters.Geometry, robot.numericalParameters.Gravity)*Beta_WLS;
    Tau_computed_TLS(:,i) = Regressor_Y_RV2SQ(Q_decim_no_load1(:,i), Qp_decim_no_load1(:,i), Qpp_decim_no_load1(:,i), robot.numericalParameters.Geometry, robot.numericalParameters.Gravity)*Beta_TLS;
end

figure('Name','Joint torque NO LOAD')
ax = gca;
ax.ColorOrderIndex = 1;
graph(1,:) = stairs(time_no_load1,Tau_computed.','Linewidth', 1);
legendInfo{1}='Reference';
hold on
ax.ColorOrderIndex = 1;
graph(2,:) = stairs(time_no_load1, (K.*I_decim_no_load1).','-.','Linewidth', 1);
legendInfo{2}='Measure';
hold off
title('\textbf{Joint torque}','Interpreter', 'LaTeX')
xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
ylabel('\textbf{Joint torque [$N.m$]}','Interpreter', 'LaTeX');
grid on
grid minor
legend(graph(:,1),legendInfo{:},'Interpreter', 'LaTeX')

clear 'graph' 'legendinfo'
for k=1:robot.nbDOF
    figure('Name',sprintf('Joint torque %d NO LOAD',k))
    graph(1,:)=stairs(time_no_load1,Tau_computed(k,:).','Linewidth', 1);
    legendInfo{1}='Reference';
    hold on
    graph(2,:)=stairs(time_no_load1, (K(k).*I_decim_no_load1(k,:)).','Linewidth', 1);
    legendInfo{2}='Measure';
    hold on
    graph(3,:)=stairs(time_no_load1, Tau_computed_LS(k,:).','Linewidth', 1);
    legendInfo{3}='WLS';
    hold on
    graph(4,:)=stairs(time_no_load1, Tau_computed_TLS(k,:).','Linewidth', 1);
    legendInfo{4}='TLS';
    hold off
    title('\textbf{Joint torque}','Interpreter', 'LaTeX')
    xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
    ylabel('\textbf{Joint torque [$N.m$]}','Interpreter', 'LaTeX');
    grid on
    grid minor
    legend(graph,legendInfo{:},'Interpreter', 'LaTeX')
end



% figure()
% plot([Q_decim_no_load1 Q_decim_no_load2].')
%
% figure()
% plot([Qp_decim_no_load1 Qp_decim_no_load2].')
% %
% figure('Name', 'Qpp NO Load')
% plot([Qpp_decim_no_load1 Qpp_decim_no_load2].')
%
% figure()
% plot([Q_decim_load1 Q_decim_load2].')
%
% figure()
% plot([Qp_decim_load1 Qp_decim_load2].')
%
% figure('Name', 'Qpp Load')
% plot([Qpp_decim_load1 Qpp_decim_load2].')
%
% figure('Name', 'I NO Load')
% plot([I_decim_no_load1 I_decim_no_load2].')
%
% figure('Name', 'I Load')
% plot([I_decim_load1 I_decim_load2].')
%
% figure('Name', 'Qp NO Load')
% plot(time_no_load1, Qp_decim_no_load1.')
% grid on
% grid minor
% legend
%
% figure('Name', 'Qp Load')
% plot(time_load1, Qp_decim_load1.')
% grid on
% grid minor
% legend
%
% figure('Name', 'I NO Load')
% plot(time_no_load1, (K.*I_decim_no_load1).')
% grid on
% grid minor
% legend
%
% figure('Name', 'I Load')
% plot(time_load1, (K.*I_decim_load1).')
% grid on
% grid minor
% legend


end


function [W] = observationMatrix_load(robotName, paramVectorSize, Geometry, Gravity, Q, Qp, Qpp) 

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building of the observation matrix W for the current iteration of Beta

[nbDOF,nbSamples]=size(Q);
W = zeros(nbSamples*nbDOF, 10);

for i=1:nbSamples
    % Compute the observation matrix:
    switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
        %         case 'toyRobot'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_toyRobot(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'SCARA'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_SCARA(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'UR3'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_UR3(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'UR5'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_UR5(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'UR10'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_UR10(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'TX40'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_TX40(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'TX40_uncoupled'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_TX40_uncoupled(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'RX90'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_RX90(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'PUMA560'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_PUMA560(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        %         case 'REEMC_right_arm'
        %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_REEMC_right_arm(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'RV2SQ'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_RV2SQ(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
            %         case 'NAO_right_arm'
            %             W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_ef_NAO_right_arm(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        otherwise
            W(nbDOF*(i-1)+1:nbDOF*i,:) = feval(sprintf('Regressor_Y_ef_%s',robotName),Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity); % Much slower when compiled !
    end
end
end

function [V_tau] = driveCurrentMatrix(I) 

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building the sampled torque matrix.

[nbDOF,nbSamples]=size(I);
V_tau = zeros(nbDOF*nbSamples, nbDOF);

for i=1:nbSamples
    for j=1:nbDOF
        V_tau((i-1)*nbDOF+j,j)=I(j,i);
    end
end
end



