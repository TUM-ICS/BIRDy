function plotTrajectory(robot, benchmarkSettings, varargin)

nbDOF = robot.nbDOF;
t_i = benchmarkSettings.t_i;
t_f = benchmarkSettings.t_f;
nbSamples = benchmarkSettings.nbSamples;

limQ_U = robot.physicalConstraints.limQ_U;
limQp_U = robot.physicalConstraints.limQp_U;
limQpp_U = robot.physicalConstraints.limQpp_U;
limQ_L = robot.physicalConstraints.limQ_L;
limQp_L = robot.physicalConstraints.limQp_L;
limQpp_L = robot.physicalConstraints.limQpp_L;


%% Computation of the trajectory
caractFreq = 0.1;
caractPuls = 2*pi*caractFreq;
n_f = 5;
timeSamples = linspace(t_i, t_f, nbSamples);
augmentedState = zeros(3*nbDOF,nbSamples);
cartesianState = zeros(3,nbSamples);

augmentedState = benchmarkSettings.trajectoryData.getTrajectoryData(timeSamples, 'spline'); % augmentedState = [Qpp; Qp; Q];
for i=1:nbSamples
    Q = augmentedState(2*robot.nbDOF+1:end,i);
    HT = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name), Q, robot.numericalParameters.Geometry);
    cartesianState(:,i) = HT(1:3,4);
end

%% Display of the results

ddot = '$\ddot{q}';
dot = '$\dot{q}';
for i = 1:robot.nbDOF
    jtAcc{i} = sprintf('%s_%d $', ddot, i);
    jtVel{i} = sprintf('%s_%d $', dot,i);
    jtPos{i} = sprintf('$q_%d $',i);
end
set(0, 'DefaultFigureRenderer', 'painters');
figure('Name','Desired joint trajectories')
subplot(3,1,1)
plot(timeSamples, augmentedState(1:nbDOF,:),'LineStyle','-', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQpp_L,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQpp_U,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
grid on
grid minor
title('Optimized Trajectory: Joint Acceleration')
legend(jtAcc, 'interpreter', 'Latex')

subplot(3,1,2)
plot(timeSamples, augmentedState(nbDOF+1:2*nbDOF,:),'LineStyle','-', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQp_L,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQp_U,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
grid on
grid minor
title('Optimized Trajectory: Joint Velocities')
legend(jtVel, 'interpreter', 'Latex')

subplot(3,1,3)
plot(timeSamples, augmentedState(2*nbDOF+1:end,:),'LineStyle','-', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQ_L,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
hold on
plot(timeSamples, repmat(limQ_U,1,numel(timeSamples)),'LineStyle','--', 'Linewidth',2)
grid on
grid minor
title('Optimized Trajectory: Joint Angles')
legend(jtPos, 'interpreter', 'Latex')

figure
plot3(cartesianState(1,:), cartesianState(2,:), cartesianState(3,:))
grid on
grid minor
title('Optimized Trajectory: 3D End-Effector Position')
legend

end



