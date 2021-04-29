function plotControlPerformance(robot, benchmarkSettings, experimentDataStruct, expNb)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%

controlPlot = true;
robotPlot = false;
useROS = false;
animatedPlot = false;
t = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples);
data = experimentDataStruct.getExperimentData(t, 1, benchmarkSettings.interpolationAlgorithmExp);


for i = 1:robot.nbDOF
    legendText{i} = sprintf("Joint %d desired",i);
    legendText{i+robot.nbDOF} = sprintf("Joint %d actual",i);
end

if controlPlot == true
    if benchmarkSettings.experimentOnTrueRobot == true
        figure('Name','Joint positions ')
        plot(t, (min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1)
        hold on
        stairs(t,(data.Qm)','--','Linewidth', 1)
        title('\textbf{Joint desired position}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint position')
        subplot(3,1,1)
        plot(t,(min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1)
        title('\textbf{Joint desired position}','Interpreter', 'LaTeX')
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,2)
        stairs(t,(data.Qm)','Linewidth', 1)
        title('\textbf{Joint position}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,3)
        stairs(t,(data.Qm-min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1)
        title('\textbf{Joint position error}','Interpreter', 'LaTeX')
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint torque')
        stairs(t,data.Taum','Linewidth', 1)
        title('\textbf{Joint torque}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint torque [$N.m$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Trajectory visualization')
        desiredTrajectory = plot3(data.Xd(1,:), data.Xd(2,:), data.Xd(3,:), 'LineStyle','--', 'Linewidth',3,'HandleVisibility','off');
        hold on
        noisyMeasurement = plot3(data.Xm(1,:), data.Xm(2,:), data.Xm(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
            grid on
            grid minor
        camproj('perspective')
        title('\textbf{Optimized Trajectory: 3D End-Effector Position}','Interpreter', 'LaTeX')
        legend([desiredTrajectory noisyMeasurement],{'Desired trajectory' 'Noisy measurement'})
        
    else
        figure('Name','Joint positions ')
        plot(t,(min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1.5)
        hold on
        stairs(t,(data.Q)','--','Linewidth', 2)
        title('\textbf{Joint desired position}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint velocities')
        plot(t,(min(robot.physicalConstraints.limQp_U,max(robot.physicalConstraints.limQp_L,data.Qpd)))','Linewidth', 1.5)
        hold on
        stairs(t,(data.Qp)','--','Linewidth', 2)
        title('\textbf{Joint desired velocity}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint velocity [$rad.s^{-1}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint position')
        subplot(3,1,1)
        plot(t,(min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1)
        title('\textbf{Joint desired position}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,2)
        stairs(t,(data.Q)','Linewidth', 1)
        title('\textbf{Joint position}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,3)
        stairs(t,(data.Q-min(robot.physicalConstraints.limQ_U,max(robot.physicalConstraints.limQ_L,data.Qd)))','Linewidth', 1)
        title('\textbf{Joint position error}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint angle [$rad$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint velocity')
        subplot(3,1,1)
        plot(t,(min(robot.physicalConstraints.limQp_U,max(robot.physicalConstraints.limQp_L,data.Qpd)))','Linewidth', 1)
        title('\textbf{Joint desired velocity}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint velocity [$rad.s^{-1}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,2)
        stairs(t,(data.Qp)','Linewidth', 1)
        title('\textbf{Joint velocity}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint velocity [$rad.s^{-1}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,3)
        stairs(t,(data.Qp-min(robot.physicalConstraints.limQp_U,max(robot.physicalConstraints.limQp_L,data.Qpd)))','Linewidth', 1)
        title('\textbf{Joint velocity error}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint velocity [$rad.s^{-1}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint acceleration')
        subplot(3,1,1)
        plot(t,(min(robot.physicalConstraints.limQpp_U,max(robot.physicalConstraints.limQpp_L,data.Qppd)))','Linewidth', 1)
        title('\textbf{Joint desired acceleration}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint acceleration [$rad.s^{-2}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,2)
        stairs(t,(data.Qpp)','Linewidth', 1)
        title('\textbf{Joint acceleration}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint acceleration [$rad.s^{-2}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        subplot(3,1,3)
        stairs(t,(data.Qpp-min(robot.physicalConstraints.limQpp_U,max(robot.physicalConstraints.limQpp_L,data.Qppd)))','Linewidth', 1)
        title('\textbf{Joint acceleration error}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint acceleration [$rad.s^{-2}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Joint torque')
        stairs(t,data.Tau','Linewidth', 1)
        title('\textbf{Joint torque}','Interpreter', 'LaTeX')
        xlabel('\textbf{Time [$s$]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint torque [$N.m$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Friction characteristic')
        plot(data.Qp',data.Tau_friction', '-','Linewidth', 2)
        title('\textbf{Friction characteristic}','Interpreter', 'LaTeX')
        ylabel('\textbf{Friction torque [$N.m$]}','Interpreter', 'LaTeX');
        xlabel('\textbf{Joint velocity [$rad.s^{-1}$]}','Interpreter', 'LaTeX');
        grid on
        grid minor
        legend(legendText,'Interpreter', 'LaTeX')
        
        figure('Name','Trajectory visualization')
        desiredTrajectory = plot3(data.Xd(1,:), data.Xd(2,:), data.Xd(3,:), 'LineStyle','--', 'Linewidth',3,'HandleVisibility','off');
        hold on
        realTrajectory = plot3(data.X(1,:), data.X(2,:), data.X(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        hold on
        noisyMeasurement = plot3(data.Xm(1,:), data.Xm(2,:), data.Xm(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        %     grid on
        %     grid minor
        camproj('perspective')
        title('\textbf{Optimized Trajectory: 3D End-Effector Position}','Interpreter', 'LaTeX')
        legend([desiredTrajectory realTrajectory noisyMeasurement],{'Desired trajectory' 'Real trajectory' 'Noisy measurement'})
    end
    
    
end

if useROS == true
    publishJointTrajectory(robot, data, expNb);
end
if strcmp(robot.name,'UR5')
    if robotPlot == true
        %     set(0, 'DefaultFigureRenderer', 'painters');
        set(0, 'DefaultFigureRenderer', 'opengl');
        figure('Name','Trajectory visualization', 'pos',[10 10 1920 1080])
        
        desiredTrajectory = plot3(data.Xd(1,:), data.Xd(2,:), data.Xd(3,:), 'LineStyle','--', 'Linewidth',3,'HandleVisibility','off');
        hold on
        realTrajectory = plot3(data.X(1,:), data.X(2,:), data.X(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        hold on
        noisyMeasurement = plot3(data.Xm(1,:), data.Xm(2,:), data.Xm(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        %     grid on
        %     grid minor
        hold on
        plotUR5(robot, data.Q(:,1));
        camproj('perspective')
        title('\textbf{Optimized Trajectory: 3D End-Effector Position}','Interpreter', 'LaTeX')
        legend([desiredTrajectory realTrajectory noisyMeasurement],{'Desired trajectory' 'Real trajectory' 'Noisy measurement'})
        view(89,9)
    end
elseif robotPlot == true
    figure('Name','Trajectory visualization')
    desiredTrajectory = plot3(data.Xd(1,:), data.Xd(2,:), data.Xd(3,:), 'LineStyle','--', 'Linewidth',3,'HandleVisibility','off');
    hold on
    realTrajectory = plot3(data.X(1,:), data.X(2,:), data.X(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
    hold on
    noisyMeasurement = plot3(data.Xm(1,:), data.Xm(2,:), data.Xm(3,:), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
    %     grid on
    %     grid minor
    hold on
    for i=1:500:size(data.Q,2)
        if i == 1
            options.displayText = true;
        else
            options.displayText = false;
        end
        plotRobot(robot, data.Q(:,i), options);
        hold on
    end
    camproj('perspective')
    title('\textbf{Optimized Trajectory: 3D End-Effector Position}','Interpreter', 'LaTeX')
    legend([desiredTrajectory realTrajectory noisyMeasurement],{'Desired trajectory' 'Real trajectory' 'Noisy measurement'})
end

if animatedPlot == true
    figure('Name','Animated Plot')
    for i=1:10:size(data.Q,2)
        plot3(data.Xm(1,1:10:end), data.Xm(2,1:10:end), data.Xm(3,1:10:end), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        hold on
        plot3(data.Xd(1,1:10:end), data.Xd(2,1:10:end), data.Xd(3,1:10:end), 'LineStyle','-', 'Linewidth',3,'HandleVisibility','off');
        hold on
        plotRobot(robot, data.Q(:,i));
        axis([-1 1 -1 1 -1 1])
        camproj('perspective')
        title('\textbf{Robot trajectory}','Interpreter', 'LaTeX')
        M(i) = getframe;
        hold off
    end
    movie(M,5)
end

end


function flag = publishJointTrajectory(robot, data, expNb)
% Create the publisher only on the first function call
rosshutdown
rosinit
pause(1)
persistent jointpub trajpub_d trajpub_m
if isempty(jointpub)
    jointpub = rospublisher('/robot_joint_states','sensor_msgs/JointState');
end
if isempty(trajpub_d)
    trajpub_d = rospublisher('/robot_trajectory_d','nav_msgs/Path');
end
if isempty(trajpub_m)
    trajpub_m = rospublisher('/robot_trajectory_m','nav_msgs/Path');
end

% TF publisher
tftree = rostf;


k=1;
trajmsg_d = rosmessage(trajpub_d);
trajmsg_m = rosmessage(trajpub_m);
jointmsg = rosmessage(jointpub);
for i = 1:size(data.Qd,2)/k
    posemsg_d(i) = rosmessage('geometry_msgs/PoseStamped');
    posemsg_m(i) = rosmessage('geometry_msgs/PoseStamped');
end

% showdetails(jointmsg)
% showdetails(trajmsg)

trajmsg_d.Header.FrameId = 'base';

counter = 0;
for i = 1:k:size(data.Qd,2)
    counter = counter+1;
    posemsg_d(counter).Pose.Position.X = data.Xd(1,i);
    posemsg_d(counter).Pose.Position.Y  = data.Xd(2,i);
    posemsg_d(counter).Pose.Position.Z  = data.Xd(3,i);
    posemsg_d(counter).Pose.Orientation.X  = 0;
    posemsg_d(counter).Pose.Orientation.Y  = 0;
    posemsg_d(counter).Pose.Orientation.Z  = 0;
    posemsg_d(counter).Pose.Orientation.W  = 1;
    posemsg_m(counter).Pose.Position.X = data.Xm(1,i);
    posemsg_m(counter).Pose.Position.Y  = data.Xm(2,i);
    posemsg_m(counter).Pose.Position.Z  = data.Xm(3,i);
    posemsg_m(counter).Pose.Orientation.X  = 0;
    posemsg_m(counter).Pose.Orientation.Y  = 0;
    posemsg_m(counter).Pose.Orientation.Z  = 0;
    posemsg_m(counter).Pose.Orientation.W  = 1;
end
trajmsg_d.Poses = posemsg_d;
trajmsg_m.Poses = posemsg_m;

if strcmp(robot.name,'UR5')
    jointmsg.Name = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
    baseID='base';
elseif strcmp(robot.name,'TX40')
    jointmsg.Name = {'$joint_1', '$joint_2', '$joint_3', '$joint_4', '$joint_5', '$joint_6'};
    baseID='base';
elseif strcmp(robot.name,'RV2SQ')
    jointmsg.Name = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6'};
    baseID='base_link';
else
    jointmsg.Name = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
    baseID='base';
end

jointmsg.Header.Stamp=rostime('now');
jointmsg.Header.Seq=0;



%Use the joint limits of the specific robot --see urdf file--
jointmsg.Position = data.Q(:,i);
jointmsg.Velocity = data.Qp(:,i);
jointmsg.Effort = data.Tau(:,i);
send(jointpub,jointmsg);


desiredRate = 10;
rate = rosrate(desiredRate);
for j = 1:5
    counter = 1;
    counter2 = 1;
    
    
    for i = 1:10:size(data.Qd,2)
        t = rostime('now');
        trajmsg_d.Header.Stamp=t;
        trajmsg_d.Header.Seq=counter2;
        trajmsg_d.Header.FrameId = baseID;
        
        trajmsg_m.Header.Stamp=t;
        trajmsg_m.Header.Seq=counter2;
        trajmsg_m.Header.FrameId = baseID;
        
        jointmsg.Header.Stamp=t;
        jointmsg.Header.Seq=counter;
        
        markermsg.Header.Stamp=t;
        markermsg.Pose.Position.X = data.Xm(1,i);
        markermsg.Pose.Position.Y = data.Xm(1,i);
        markermsg.Pose.Position.Z = data.Xm(1,i);
        counter=counter+1;
        
        %Use the joint limits of the specific robot --see urdf file--
        jointmsg.Position = data.Q(:,i);
        jointmsg.Velocity = data.Qp(:,i);
        jointmsg.Effort = data.Tau(:,i);
        send(jointpub,jointmsg);
        send(trajpub_d,trajmsg_d);
        send(trajpub_m,trajmsg_m);
        
        waitfor(rate);
    end
end
clear jointpub trajpub_d trajpub_m
rosshutdown
flag = true;
end


