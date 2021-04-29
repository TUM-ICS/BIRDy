function [frictionForce] = computeFrictionModel(robotName, Qp, frictionParameters, Z_m1, dt)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Compute robot friction model in symbolic from. Returns a [nbDOFx(nbDOF+1)]
% matrix, whose columns are the friction torques generated with each model.

frictionForce = sym(zeros(numel(Qp), 8));

% No friction:
frictionForce(:,1) = 0*Qp + frictionParameters.Tau_off;
% Viscous friction:
frictionForce(:,2) = diag(frictionParameters.Fv)*Qp + frictionParameters.Tau_off;
% Coulomb friction:
frictionForce(:,3) = diag(frictionParameters.Fc)*tanh(100*Qp) + frictionParameters.Tau_off;
% Intagrated Viscous and Coulomb friction:
frictionForce(:,4) = diag(frictionParameters.Fv)*Qp + diag(frictionParameters.Fc)*tanh(100*Qp);
if strcmp(robotName, 'TX40') || strcmp(robotName, 'RX90')
    frictionForce(5,4) = frictionForce(5,4) + Qp(6)*frictionParameters.Fvm(6) + tanh(100*Qp(6))*frictionParameters.Fcm(6);
    frictionForce(6,4) = frictionForce(6,4) + Qp(5)*frictionParameters.Fvm(6) + tanh(100*Qp(5))*frictionParameters.Fcm(6);
end
% Intagrated Viscous and Coulomb friction with offset:
frictionForce(:,5) = diag(frictionParameters.Fv)*Qp + diag(frictionParameters.Fc)*tanh(100*Qp) + frictionParameters.Tau_off;
if strcmp(robotName, 'TX40') || strcmp(robotName, 'RX90')
    frictionForce(5,5) = frictionForce(5,4) + Qp(6)*frictionParameters.Fvm(6) + tanh(100*Qp(6))*frictionParameters.Fcm(6);
    frictionForce(6,5) = frictionForce(6,4) + Qp(5)*frictionParameters.Fvm(6) + tanh(100*Qp(5))*frictionParameters.Fcm(6);
end
% Stribeck friction:
frictionForce(:,6) = diag(frictionParameters.Fv)*Qp + diag(frictionParameters.Fc + diag(frictionParameters.Fs - frictionParameters.Fc)*exp(-(abs(Qp./frictionParameters.Vs).^frictionParameters.Es)))*tanh(100*Qp) + frictionParameters.Tau_off;
% LuGre friction:
[F, Z] = LuGre(Z_m1, Qp, frictionParameters.Fc, frictionParameters.Fs, frictionParameters.Vs, frictionParameters.Sigma_0, frictionParameters.Sigma_1, frictionParameters.Sigma_2, frictionParameters.Es, dt);
frictionForce(:,7) = F + frictionParameters.Tau_off;
% LuGre state:
frictionForce(:,8) = Z;
end

function [F, Z] = LuGre(Z, Qp, Fc, Fs, Vs, Sigma_0, Sigma_1, Sigma_2, Es, dt)
phi = (Fc + diag(Fs - Fc)*exp(-(abs(Qp./Vs).^Es))) ./ Sigma_0;
Zp = Qp - abs(Qp).*Z ./ phi;
Z = Z + Zp*dt;
F = diag(Sigma_0)*Z + diag(Sigma_1)*Zp + diag(Sigma_2)*Qp;
end
