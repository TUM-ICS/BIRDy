function [Tau] = RecursiveNewtonEuler(Q,Qp,Qpp,robot,Xhi)

% We assume fixed-base serial robots (to start...)

% Forward recursion:

for i=1:nbDOF
    J = feval(sprintf('J_dh%d_world_%s', i,robot.name),Q,robot.numericalParameters.Geometry);
    Jp = feval(sprintf('Jd_dh%d_world_%s', i,robot.name),Q,Qp,robot.numericalParameters.Geometry);
    Xp(:,i) = J*Qp;
    Xpp(:,i) = Jp*Qp + J*Qpp;
end

% Backward recursion:

for i=nbDOF:2
    Li = inertiaMatrix(Xhi_i(1), Xhi_i(2), Xhi_i(3), Xhi_i(4), Xhi_i(5), Xhi_i(6));
    li = Xhi_i(7:9);
    Mi = Xhi_i(10);
    
    pseudoInertia = [Mi*eye(3), -Skew(li);...
                Skew(li), Li];
    
    H_dhi_dhi_1 = invHT(feval(sprintf('HT_dh%d_dh%d_%s', i,i-1,robot.name),Q, robot.numericalParameters.Geometry)); % Homogeneous transform of frame i-1 wrt frame i
    R = H_dhi_dhi_1(1:3,1:3);
    r = H_dhi_dhi_1(1:3,4);
    T = [R, -R*Skew(r); zeros(3), R];
    W(:,i-1) = pseudoInertia*Xpp(:,i) + [Skew(Xp(4:6,i))*(Skew(Xp(4:6,i))*li); Skew(Xp(4:6,i))*(Li*Xp(4:6,i))];
    Tau(i) = J'*W(:,i-1);
end
