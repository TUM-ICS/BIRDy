function flag = plotRobot(robot, Q, options)

if nargin < 3
    options.displayText = true;
end

% set(0, 'DefaultFigureRenderer', 'painters');
set(0, 'DefaultFigureRenderer', 'opengl');
plotFrame(eye(4), 'World_Frame'); % Plot the world frame
HT_DH_world = feval(sprintf('HT_dh%d_world_%s', 0,robot.name), Q, robot.numericalParameters.Geometry);
plotFrame(HT_DH_world, 'DH-Frame_0'); % Plot the robot Base frame

for indexJoint = 1:robot.nbDOF
    HT_DH_world_1 = HT_DH_world;
    HT_DH_world = feval(sprintf('HT_dh%d_world_%s', indexJoint,robot.name), Q, robot.numericalParameters.Geometry);
    plot3([HT_DH_world_1(1,4);HT_DH_world(1,4)],[HT_DH_world_1(2,4);HT_DH_world(2,4)],[HT_DH_world_1(3,4);HT_DH_world(3,4)],'k --', 'LineWidth', 0.5,'HandleVisibility','off');
    hold on
    plotFrame(HT_DH_world, sprintf('DH-Frame_%d',indexJoint), options); % Plot the robot DH frame w.r.t world
    hold on
end

flag = true;
end
