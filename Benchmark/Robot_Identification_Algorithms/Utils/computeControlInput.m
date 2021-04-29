function [controlInput, integralError] = computeControlInput(dt, currentState, augmentedDesiredState, integralError, Kp, Ki, Kd, Ktau, antiWindup, limQ_L, limQ_U, limQp_L, limQp_U, limQpp_L, limQpp_U, limTau_L, limTau_U)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%

nbDOF = numel(limQ_L);

% Use the trajectory generator to get the desired state at control epoch i:

Qd = min(limQ_U,max(limQ_L,augmentedDesiredState(2*nbDOF+1:end)));
Qpd = min(limQp_U,max(limQp_L,augmentedDesiredState(nbDOF+1:2*nbDOF)));

% Compute the error vectors:
DeltaQ = Qd - currentState(nbDOF+1:end);
DeltaQp = Qpd - currentState(1:nbDOF);
integralError = min(antiWindup*ones(nbDOF,1),max(-antiWindup*ones(nbDOF,1),integralError + dt*DeltaQ));

% Compute the control imput:
controlInput = min(limTau_U,max(limTau_L, Ktau*(Kp*DeltaQ + Kd*DeltaQp + Ki*integralError)));

end


