% This function allows to visualize the fricition characteristics of the different friction models.

% Authors: Julien Roux, Quentin Leboutet, Alexandre Janot, Gordon Cheng

robot.nbDOF = 4;

frictionParameters.Fv = 0.25*ones(robot.nbDOF,1);                   % Viscous Friction
frictionParameters.Fc = 1.5*ones(robot.nbDOF,1);                    % Coulomb Friction
frictionParameters.Fs = 2*ones(robot.nbDOF,1);                      % Static Friction: only in Stribeck and LuGre models
frictionParameters.Vs = 0.1*ones(robot.nbDOF,1);                    % Stribeck velocity: only in Stribeck and LuGre models
frictionParameters.Es = 2*ones(robot.nbDOF,1);                      % Exponent: only in Stribeck and LuGre models
frictionParameters.Sigma_0 = 500*ones(robot.nbDOF,1);               % Contact stiffness: only in LuGre model
frictionParameters.Sigma_1 = 1*ones(robot.nbDOF,1);                 % Damping coefficient of the bristle: only in LuGre model
frictionParameters.Sigma_2 = 0.25*ones(robot.nbDOF,1);              % Viscous friction coefficient of the bristle: only in LuGre model
frictionParameters.Z = 0*randn(robot.nbDOF,1);                       % Average deflection of the contacting asperities: only in LuGre model
frictionParameters.Tf0 = 0*ones(robot.nbDOF,1);                     % Friction torque parameter for nonlinear friction models (Stribeck and LuGre)
dt = 1e-3;

% qp = sin(-2*pi:0.01:2*pi);
qp = ones(10000,1);
% Qp = repmat(qp,robot.nbDOF,1);
% Qp = Qp + 0.1*randn(size(Qp));
Qp = experimentDataStruct.Qpd(:,:,1);

counter = 0;
frictionForce = zeros(6, robot.nbDOF, 10000);
for i=1:10000


% model{1}='no';
frictionForce(1,:,i) = 0*Qp(:,i);
% model{2}='Viscous';
frictionForce(2,:,i) = diag(frictionParameters.Fv)*Qp(:,i);
% model{3}='Coulomb';
frictionForce(3,:,i) = diag(frictionParameters.Fc)*tanh(100*Qp(:,i));
% model{4}='integratedViscousCoulomb';
frictionForce(4,:,i) = diag(frictionParameters.Fv)*Qp(:,i) + diag(frictionParameters.Fc)*tanh(100*Qp(:,i));
% model{5}='Stribeck'; % Nonlinear friction !
frictionForce(5,:,i) = diag(frictionParameters.Fv)*Qp(:,i) + diag(frictionParameters.Fc + diag(frictionParameters.Fs - frictionParameters.Fc)*exp(-(abs(Qp(:,i)./frictionParameters.Vs).^frictionParameters.Es)))*tanh(100*Qp(:,i));
% model{6}='LuGre'; % Nonlinear friction + NO ANALYTICAL EXPRESSION !


    counter = counter+1;
    [frictionForce(6,:,counter), frictionParameters.Z] = LuGre(frictionParameters.Z, Qp(:,i), frictionParameters.Fc, frictionParameters.Fs, frictionParameters.Vs, frictionParameters.Sigma_0, frictionParameters.Sigma_1, frictionParameters.Sigma_2, frictionParameters.Es, dt);
end


figure(1)
subplot(1,3,1)
for i=1:robot.nbDOF
    plot(Qp(i,:),reshape(frictionForce(4,i,:), 1, numel(qp)), '.','Linewidth', 2);
    hold on
end
hold off
grid on
grid minor
title('\textbf{Viscous + Coulomb Friction Model}', 'Interpreter', 'LaTeX');
xlabel('$\dot{q}$', 'Interpreter', 'LaTeX')
ylabel('$f$', 'Interpreter', 'LaTeX')
subplot(1,3,2)
for i=1:robot.nbDOF
    plot(Qp(i,:),reshape(frictionForce(5,i,:), 1, numel(qp)), '.', 'Linewidth', 2);
    hold on
end
hold off
grid on
grid minor
title('\textbf{Stribeck Friction Model}', 'Interpreter', 'LaTeX');
xlabel('$\dot{q}$', 'Interpreter', 'LaTeX')
ylabel('$f$', 'Interpreter', 'LaTeX')
subplot(1,3,3)
for i=1:robot.nbDOF
    plot(Qp(i,:),reshape(frictionForce(6,i,:), 1, numel(qp)), '.', 'Linewidth', 2);
    hold on
end
hold off
grid on
grid minor
title('\textbf{LuGre Friction Model}', 'Interpreter', 'LaTeX');
xlabel('$\dot{q}$', 'Interpreter', 'LaTeX')
ylabel('$f$', 'Interpreter', 'LaTeX')


function [F, Z] = LuGre(Z, Qp, Fc, Fs, Vs, Sigma_0, Sigma_1, Sigma_2, Es, dt)
phi = (Fc + diag(Fs - Fc)*exp(-(abs(Qp./Vs).^Es))) ./ Sigma_0;
Zp = Qp - abs(Qp).*Z ./ phi;
Z = Z + Zp*dt;
F = diag(Sigma_0)*Z + diag(Sigma_1)*Zp + diag(Sigma_2)*Qp;
end
