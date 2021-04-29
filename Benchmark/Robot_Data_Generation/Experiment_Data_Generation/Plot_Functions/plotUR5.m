function flag = plotUR5(robot, Q)
[L0, UR5, ~, ~]=loadUR5(robot, Q);
fprintf('Plotting the robot... \n');
% set(0, 'DefaultFigureRenderer', 'painters');
set(0, 'DefaultFigureRenderer', 'opengl');
plotFrame(eye(4), 'World_Frame'); % Plot the world frame
HT_DH_world = feval(sprintf('HT_dh%d_world_%s', 0,robot.name), Q, robot.numericalParameters.Geometry);
plotFrame(HT_DH_world, 'DH-Frame_0'); % Plot the robot Base frame
plotLink(L0);

for indexJoint = 1:robot.nbDOF
    HT_DH_world = feval(sprintf('HT_dh%d_world_%s', indexJoint,robot.name), Q, robot.numericalParameters.Geometry);
    plotLink(UR5(indexJoint));
    hold on
    plotFrame(HT_DH_world, sprintf('DH-Frame_%d',indexJoint)); % Plot the robot DH frame w.r.t world
    hold on
end

flag = true;
end

