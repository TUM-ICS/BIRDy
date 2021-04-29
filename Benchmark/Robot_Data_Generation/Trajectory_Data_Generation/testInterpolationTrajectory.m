function testInterpolationTrajectory()

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Display interpolated trajectories with the different available methods.

t_ref = 0:0.01:10;
q_ref = sawtooth(2*t_ref);%sin(2*t_ref);
qp_ref = cos(2*t_ref);
qpp_ref= -sin(2*t_ref);

t = 0:0.2:10;
q_sampled = sawtooth(2*t); %sin(2*t);
qp_sampled = cos(2*t);
qpp_sampled= -sin(2*t);
traj = Trajectory;
traj.setTrajectoryData(t,q_sampled,qp_sampled,qpp_sampled);
t1 = 0:0.05:10;
% interpolation method = 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
augTrajLinear = traj.getTrajectoryData(t1);
augTrajNearest = traj.getTrajectoryData(t1,'nearest');
augTrajNext = traj.getTrajectoryData(t1,'next');
augTrajPrevious = traj.getTrajectoryData(t1,'previous');
augTrajPchip = traj.getTrajectoryData(t1,'pchip');
augTrajCubic = traj.getTrajectoryData(t1,'cubic');
augTrajV5cubic = traj.getTrajectoryData(t1,'v5cubic');
augTrajMakima = traj.getTrajectoryData(t1,'makima');
augTrajSpline = traj.getTrajectoryData(t1,'spline');


figure(1)
subplot(3,1,1)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajLinear(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Linear Interpolation');
subplot(3,1,2)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajNearest(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Nearest Interpolation');
subplot(3,1,3)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajNext(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Next Interpolation');

figure(2)
subplot(3,1,1)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajPrevious(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Previous Interpolation');
subplot(3,1,2)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajPchip(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Pchip Interpolation');
subplot(3,1,3)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajCubic(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Cubic Interpolation');

figure(3)
subplot(3,1,1)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajV5cubic(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with V5ubic Interpolation');
subplot(3,1,2)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajMakima(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Makima Interpolation');
subplot(3,1,3)
plot(t_ref,q_ref,t,q_sampled,'ro',t1,augTrajSpline(3,:),'--', 'linewidth', 2);
grid on 
grid minor
legend('reference', 'sampled','interpolated')
title('Test Trajectory with Spline Interpolation');
