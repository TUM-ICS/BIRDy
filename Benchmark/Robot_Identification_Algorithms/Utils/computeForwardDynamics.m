function Xp = computeForwardDynamics(robotName, X, Tau, Beta, Geometry, Gravity) %#codegen

% Return the forward dynamics of a robot with a torque input Tau.
% State vector X = [Qp;Q]. The friction model is included within h. 

%% Dynamics

nbDOF = numel(Tau);
Q = X(nbDOF+1:2*nbDOF);
Qp = X(1:nbDOF);

switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
    case 'TX40'
        M = Inertia_M_Beta_TX40(Q, Geometry, Beta);
        h = CorCen_h_Beta_TX40(Q, Qp, Geometry, Gravity, Beta);
    case 'TX40_uncoupled'
        M = Inertia_M_Beta_TX40_uncoupled(Q, Geometry, Beta);
        h = CorCen_h_Beta_TX40_uncoupled(Q, Qp, Geometry, Gravity, Beta);
    case 'RV2SQ'
        M = Inertia_M_Beta_RV2SQ(Q, Geometry, Beta);
        h = CorCen_h_Beta_RV2SQ(Q, Qp, Geometry, Gravity, Beta);
    otherwise
        M = feval(sprintf('Inertia_M_Beta_%s', robotName),Q, Geometry, Beta); % Much slower when compiled !
        h = feval(sprintf('CorCen_h_Beta_%s', robotName),Q, Qp, Geometry, Gravity, Beta); % Much slower when compiled !
end

%% Computation of the joint acceleration

K = 5e2; % Acceleration is saturated to avoid inconsistant results
Qpp = min(K*ones(nbDOF,1),max(-K*ones(nbDOF,1),M\(Tau - h)));

Xp = [Qpp; Qp];
end
