function Xp = computeDataGenForwardDynamics(robotName, X, Tau, Xhi, Geometry, Gravity, Friction) %#codegen

% Return the forward dynamics of a robot with a torque input Tau.
% State vector X = [Qp;Q]. The friction model is defined externally. 

%% Dynamics

nbDOF = numel(Tau);
Q = X(nbDOF+1:2*nbDOF);
Qp = X(1:nbDOF);

switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, RV2SQ, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
    case 'TX40'
        M = Inertia_M_TX40(Q, Geometry, Xhi);
        C = CorCen_C_TX40(Q, Qp, Geometry, Xhi);
        G = Gravity_G_TX40(Q, Geometry, Xhi, Gravity);
    case 'TX40_uncoupled'
        M = Inertia_M_TX40_uncoupled(Q, Geometry, Xhi);
        C = CorCen_C_TX40_uncoupled(Q, Qp, Geometry, Xhi);
        G = Gravity_G_TX40_uncoupled(Q, Geometry, Xhi, Gravity);
    case 'RV2SQ'
        M = Inertia_M_RV2SQ(Q, Geometry, Xhi);
        C = CorCen_C_RV2SQ(Q, Qp, Geometry, Xhi);
        G = Gravity_G_RV2SQ(Q, Geometry, Xhi, Gravity);
    otherwise
        M = feval(sprintf('Inertia_M_%s', robotName),Q, Geometry, Xhi);                 % Much slower when compiled !
        C = feval(sprintf('CorCen_C_%s', robotName),Q, Qp, Geometry, Xhi);              % Much slower when compiled !
        G = feval(sprintf('Gravity_G_%s', robotName),Q, Geometry, Xhi, Gravity);        % Much slower when compiled !
end

%% Computation of the joint acceleration

K = 5e2; % Acceleration is saturated to avoid inconsistant results
Qpp = min(K*ones(nbDOF,1),max(-K*ones(nbDOF,1),M\(Tau - C*Qp - G - Friction)));

Xp = [Qpp; Qp];
end
