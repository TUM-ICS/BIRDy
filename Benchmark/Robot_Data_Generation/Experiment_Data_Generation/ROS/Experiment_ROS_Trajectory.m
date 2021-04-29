function [statusFlag] = Experiment_ROS_Trajectory(robot, benchmarkSettings, options, varargin)

if nargin<3
    options.generateRealRobot = false;
    options.useROS = false;
end

switch robot.name
    
    case 'REEMC_right_arm'
        
        nbDOF = robot.nbDOF;
        t_i = benchmarkSettings.t_i;
        t_f = benchmarkSettings.t_f;
        nbSamples = benchmarkSettings.nbSamples;
        
        
        %% Computation of the trajectory
        timeSamples = linspace(t_i, t_f, nbSamples);
        augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(timeSamples, benchmarkSettings.interpolationAlgorithm); % Initial state
        cartesianState = zeros(3,nbSamples);
        
        for i=1:nbSamples
            Qd(:,i) = augmentedDesiredState(2*robot.nbDOF+1:end,i);
            Qpd(:,i) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,i);
            Qppd(:,i) = augmentedDesiredState(1:robot.nbDOF,i);
            HT = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name), Qd(:,i), robot.numericalParameters.Geometry);
            cartesianState(:,i) = HT(1:3,4);
        end
        
        %% Fill trajectory files for the real robot:
        if options.generateRealRobot == true
            QQ = augmentedDesiredState(2*nbDOF+1:end,:);
            Qd=QQ';
            file = fopen('Benchmark/Robot_Generated_Data/REEMC_right_arm/Experiment_Data/arm_excitation.yaml', 'w');
            
            fprintf(file, "action_name: /right_arm_controller/follow_joint_trajectory\n");
            fprintf(file, "\n");
            fprintf(file, "joints:\n");
            fprintf(file, "        - arm_right_1_joint\n");
            fprintf(file, "        - arm_right_2_joint\n");
            fprintf(file, "        - arm_right_3_joint\n");
            fprintf(file, "        - arm_right_4_joint\n");
            fprintf(file, "        - arm_right_5_joint\n");
            fprintf(file, "        - arm_right_6_joint\n");
            fprintf(file, "        - arm_right_7_joint\n");
            fprintf(file, "\n");
            fprintf(file, "trans_time: 0.005\n");
            fprintf(file, "\n");
            fprintf(file, "way_points:\n");
            fprintf(file, "        [\n");
            fprintf(file, "\n");
            
            for i = 1:nbSamples
                fprintf(file, "[");
                for j = 1:nbDOF
                    if j == nbDOF
                        fprintf(file, "%d ", Qd(i,j));
                    else
                        fprintf(file, "%d, ", Qd(i,j));
                    end
                end
                if i == nbSamples
                    fprintf(file, "]\n");
                else
                    fprintf(file, "],\n");
                end
            end
            
            fprintf(file, "        ]\n");
            fprintf(file, "\n");
            fprintf(file, "way_point_velocities:\n");
            fprintf(file, "        [\n");
            fprintf(file, "\n");
            
            QQ = augmentedDesiredState(2*nbDOF+1:end,:);
            Qd=QQ';
            QQp = augmentedDesiredState(nbDOF+1:2*nbDOF,:);
            Qpd=QQp';
            
            for i = 1:nbSamples
                fprintf(file, "[");
                for j = 1:nbDOF
                    if j == nbDOF
                        fprintf(file, "%d ", Qd(i,j));
                    else
                        fprintf(file, "%d, ", Qpd(i,j));
                    end
                end
                if i == nbSamples
                    fprintf(file, "]\n");
                else
                    fprintf(file, "],\n");
                end
                
            end
            fprintf(file, "        ]\n");
            fclose(file);
            
            
            
        end
        
        if options.useROS == true
            % rosshutdown
            % rosinit
            
            jointState = csvread('jointStates_100_NB_1.csv',1,106);
            
            % figure
            %
            % plot(jointState(:,8:14),'LineWidth',2);
            %
            % figure
            %
            % plot(jointState(:,8+102:14+102),'LineWidth',2);
            %
            % figure
            %
            % plot(jointState(:,8+102+102:14+102+102),'LineWidth',2);
            
            QQQ = jointState(:,8:14);
            pause(1)
            QQ = augmentedDesiredState(2*nbDOF+1:end,:);
            desiredRate = 10;
            rate = rosrate(desiredRate);
            
            % TF publisher
            tftree = rostf;
            %=== specify data to publish
            persistent trajpub_d trajpub_a
            
            while true
                %
                
                if isempty(trajpub_d)
                    trajpub_d = rospublisher('/robot_trajectory_d','nav_msgs/Path');
                end
                if isempty(trajpub_a)
                    trajpub_a = rospublisher('/robot_trajectory_a','nav_msgs/Path');
                end
                
                k=10;
                trajmsg_d = rosmessage(trajpub_d);
                trajmsg_a = rosmessage(trajpub_a);
                
                for i = 1:size(QQ,2)/k
                    posemsg_d(i) = rosmessage('geometry_msgs/PoseStamped');
                end
                for i = 1:2500/k
                    posemsg_a(i) = rosmessage('geometry_msgs/PoseStamped');
                end
                
                
                % showdetails(jointmsg)
                % showdetails(trajmsg)
                
                trajmsg_d.Header.FrameId = 'base_link';
                trajmsg_a.Header.FrameId = 'base_link';
                
                counter = 0;
                
                for i = 1:k:size(QQ,2)
                    counter = counter+1;
                    posemsg_d(counter).Pose.Position.X = cartesianState(1,i);
                    posemsg_d(counter).Pose.Position.Y  = cartesianState(2,i);
                    posemsg_d(counter).Pose.Position.Z  = cartesianState(3,i);
                    posemsg_d(counter).Pose.Orientation.X  = 0;
                    posemsg_d(counter).Pose.Orientation.Y  = 0;
                    posemsg_d(counter).Pose.Orientation.Z  = 0;
                    posemsg_d(counter).Pose.Orientation.W  = 1;
                end
                counter = 0;
                
                for i = 5000:k:7100
                    counter = counter+1;
                    HT = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name), QQQ(i,:)', robot.numericalParameters.Geometry);
                    state = HT(1:3,4);
                    posemsg_a(counter).Pose.Position.X = state(1);
                    posemsg_a(counter).Pose.Position.Y  = state(2);
                    posemsg_a(counter).Pose.Position.Z  = state(3);
                    posemsg_a(counter).Pose.Orientation.X  = 0;
                    posemsg_a(counter).Pose.Orientation.Y  = 0;
                    posemsg_a(counter).Pose.Orientation.Z  = 0;
                    posemsg_a(counter).Pose.Orientation.W  = 1;
                end
                trajmsg_d.Poses = posemsg_d;
                trajmsg_a.Poses = posemsg_a;
                for k=1:robot.nbDOF
                    HT=feval(sprintf('HT_dh%d_world_%s', k ,robot.name),robot.numericalParameters.Q0, robot.numericalParameters.Geometry);%*robot.numericalParameters.H_URDF_DH(:,:,k);
                    quatrot = rotm2quat(HT(1:3,1:3));
                    tfStampedMsgDH(k) = rosmessage('geometry_msgs/TransformStamped');
                    tfStampedMsgDH(k).Header.FrameId = "base_link";
                    tfStampedMsgDH(k).ChildFrameId = strcat('DH_',num2str(k));
                    
                    tfStampedMsgDH(k).Header.Stamp = rostime('now');
                    %             tfStampedMsgDH(k).Header.Seq=counter;
                    tfStampedMsgDH(k).Transform.Rotation.W = quatrot(1);
                    tfStampedMsgDH(k).Transform.Rotation.X = quatrot(2);
                    tfStampedMsgDH(k).Transform.Rotation.Y = quatrot(3);
                    tfStampedMsgDH(k).Transform.Rotation.Z = quatrot(4);
                    tfStampedMsgDH(k).Transform.Translation.X = HT(1,4);
                    tfStampedMsgDH(k).Transform.Translation.Y = HT(2,4);
                    tfStampedMsgDH(k).Transform.Translation.Z = HT(3,4);
                    sendTransform(tftree, tfStampedMsgDH(k));
                end
                for k=1:robot.nbDOF
                    tfStampedMsgDH(k) = rosmessage('geometry_msgs/TransformStamped');
                    tfStampedMsgDH(k).Header.FrameId = strcat('DH_',num2str(k));
                    tfStampedMsgDH(k).ChildFrameId = strcat('COM_',num2str(k));
                    
                    tfStampedMsgDH(k).Header.Stamp = rostime('now');
                    %             tfStampedMsgDH(k).Header.Seq=counter;
                    tfStampedMsgDH(k).Transform.Rotation.W = 1;
                    tfStampedMsgDH(k).Transform.Rotation.X = 0;
                    tfStampedMsgDH(k).Transform.Rotation.Y = 0;
                    tfStampedMsgDH(k).Transform.Rotation.Z = 0;
                    tfStampedMsgDH(k).Transform.Translation.X = robot.numericalParameters.GeometryCOM(1,k);
                    tfStampedMsgDH(k).Transform.Translation.Y = robot.numericalParameters.GeometryCOM(2,k);
                    tfStampedMsgDH(k).Transform.Translation.Z = robot.numericalParameters.GeometryCOM(3,k);
                    
                    sendTransform(tftree, tfStampedMsgDH(k));
                    
                    %- set the frame information and names of marker
                    markerPub(k) = rospublisher(strcat('com_',num2str(k)),'visualization_msgs/Marker');
                    marker(k) = rosmessage(markerPub(k));
                    marker(k).Header.FrameId = 'base_link';
                    marker(k).Ns = 'basic_shapes';
                    marker(k).Text = 'sphere';
                    
                    %- set the time
                    marker(k).Header.Stamp = rostime('now','system');
                    
                    %- set the ID of the shape
                    marker(k).Id = 0;
                    
                    %- set the scale of the shape
                    marker(k).Scale.X = 0.05;
                    marker(k).Scale.Y = 0.05;
                    marker(k).Scale.Z = 0.05;
                    HT = feval(sprintf('HT_dh%d_world_%s', k ,robot.name),robot.numericalParameters.Q0, robot.numericalParameters.Geometry)*[robot.numericalParameters.GeometryCOM(:,k);1];
                    %- set position and orientation
                    Pos = rosmessage('geometry_msgs/Point');
                    Pos.X = HT(1);
                    Pos.Y = HT(2);
                    Pos.Z = HT(3);
                    Ori = rosmessage('geometry_msgs/Quaternion');
                    Ori.W = 1;
                    marker(k).Pose.Position = Pos;
                    marker(k).Pose.Orientation = Ori;
                    marker(k).Type = 2;  % 1: cube, 2: sphere, 3: cylinder
                    %- set color
                    Color = rosmessage('std_msgs/ColorRGBA');
                    Color.R = 0;
                    Color.G = 1;
                    Color.B = 0;
                    Color.A = 1;
                    marker(k).Color = Color;
                    send(markerPub(k),marker(k));
                    
                end
                send(trajpub_d,trajmsg_d);
                send(trajpub_a,trajmsg_a);
                
                waitfor(rate);
            end
            % rosshutdown
        end
        
        
        % /base_imu
        % /clock
        % /h1/skin/left/flank/skinArmExcitation
        % /h1/skin/right/lower_arm/skinArmExcitation
        % /h1/skin/right/upper_arm/skinArmExcitation
        % /joint_states
        % /left_ankle_ft
        % /right_ankle_ft
        % /right_arm_controller/state
        % /right_wrist_ft
        
        % joint_states = rossubscriber('/joint_states');
        
        
        % joint_states_sub = rossubscriber('/joint_states', @function1);
        % accel_flank = rossubscriber('/h1/skin/left/flank/skinArmExcitation');
        % accel_lower_arm = rossubscriber('/h1/skin/right/lower_arm/skinArmExcitation');
        % accel_upper_arm = rossubscriber('/h1/skin/right/upper_arm/skinArmExcitation');
        
        % accel_flank_data = receive(accel_flank,10)
        % joint_states_data = receive(joint_states_sub,10)
        % joint_states_sub.Position
        
        % figure
        % function function1(src,msg)
        % plot(msg.Position(8:15)); % Right Arm Position
        % end
        
        
        % jointState = csvread('jointStates_100_NB_2.csv',1,106);
        %
        % start1 = 4975;
        % stop1 = start1 + nbSamples;
        %
        % figure('Name','Joint positions')
        % stairs(Qd','LineWidth',2)
        % hold on
        % stairs(jointState(start1:stop1,8:14),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint positions')
        % legend
        %
        % figure('Name','Joint velocities')
        % stairs(Qpd','LineWidth',2)
        % hold on
        % stairs(jointState(start1:stop1,8+102:14+102),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint velocities')
        % legend
        %
        % figure('Name','Control command (motor current)')
        % stairs(Qppd','LineWidth',2)
        % hold on
        % stairs(jointState(start1:stop1,8+102+102:14+102+102),'LineWidth',2);
        % grid on
        % grid minor
        % title('Control command (motor current)')
        % legend
        
        
        % accel_flank = csvread('accel_flank_80_NB_1.csv',1,1);
        % figure
        % cellIndex = 40;
        % index = find(accel_flank(:,1)==cellIndex);
        % plot(accel_flank(index,9:11),'LineWidth',2);
        % grid on
        % grid minor
        % title(sprintf('Cell %d acceleration',cellIndex))
        % legend
        
        % start2 = 1247;
        % stop2 = 1247+500;
        % ctrlState = csvread('ctrlState_100_NB_2.csv',1,11);
        %
        % figure('Name','Joint positions ')
        % stairs(ctrlState(start2:stop2,1:7),'LineWidth',2);
        % hold on
        % stairs(ctrlState(start2:stop2,23:29),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint positions')
        % legend
        %
        % figure('Name','Joint velocities ')
        % stairs(ctrlState(start2:stop2,8:14),'LineWidth',2);
        % hold on
        % stairs(ctrlState(start2:stop2,30:36),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint velocities')
        % legend
        %
        % figure('Name','Joint accelerations')
        % stairs(ctrlState(start2:stop2,15:20),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint accelerations')
        % legend
        %
        % figure('Name','Joint position errors')
        % stairs(ctrlState(start2:stop2,38:44),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint position errors')
        % legend
        %
        % figure('Name','Joint velocity errors')
        % stairs(ctrlState(start2:stop2,45:51),'LineWidth',2);
        % grid on
        % grid minor
        % title('Joint velocity errors')
        % legend
        %
        %
        % controlData.torqueRatio = robot.controlParameters.torqueRatio;
        % controlData.samplingFrequency = robot.controlParameters.samplingFrequency;
        % controlData.controlFrequency = robot.controlParameters.controlFrequency;
        % controlData.desiredJointAngles = Qd';
        % controlData.desiredJointVelocity = Qpd';
        % controlData.desiredJointAcceleration = Qppd';
        % controlData.actualJointAngles = jointState(start1:stop1-1,8:14);
        % controlData.actualJointVelocity = jointState(start1:stop1-1,8+102:14+102);
        % controlData.actualMotorCurrents = jointState(start1:stop1-1,8+102+102:14+102+102);
        % save(sprintf('Benchmark/Robot_Identification_Results/%s/controlData_%s.mat',robot.name,robot.name), 'controlData', '-v7.3');
        
        statusFlag = true;

	case 'RV2SQ'

	nbDOF = robot.nbDOF;
        t_i = benchmarkSettings.t_i;
        t_f = benchmarkSettings.t_f;
        nbSamples = benchmarkSettings.nbSamples;
        
        
        %% Computation of the trajectory
        timeSamples = linspace(t_i, t_f, nbSamples);
        augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(timeSamples, benchmarkSettings.interpolationAlgorithm); % Initial state
        cartesianState = zeros(3,nbSamples);
        
        for i=1:nbSamples
            Qd(:,i) = augmentedDesiredState(2*robot.nbDOF+1:end,i);
            Qpd(:,i) = augmentedDesiredState(robot.nbDOF+1:2*robot.nbDOF,i);
            Qppd(:,i) = augmentedDesiredState(1:robot.nbDOF,i);
            HT = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name), Qd(:,i), robot.numericalParameters.Geometry);
            cartesianState(:,i) = HT(1:3,4);
        end
        
        %% Fill trajectory files for the real robot:
        if options.generateRealRobot == true
            QQ = augmentedDesiredState(2*nbDOF+1:end,:);
            Qd=QQ';
            file = fopen('Benchmark/Robot_Generated_Data/RV2SQ/Experiment_Data/arm_excitation.yaml', 'w');
            
            fprintf(file, "action_name: /arm_controller/follow_joint_trajectory\n");
            fprintf(file, "\n");
            fprintf(file, "joints:\n");
            fprintf(file, "        - j1\n");
            fprintf(file, "        - j2\n");
            fprintf(file, "        - j3\n");
            fprintf(file, "        - j4\n");
            fprintf(file, "        - j5\n");
            fprintf(file, "        - j6\n");
            fprintf(file, "\n");
            fprintf(file, "trans_time: 0.0071\n");
            fprintf(file, "\n");
            fprintf(file, "way_points:\n");
            fprintf(file, "        [\n");
            fprintf(file, "\n");
            
            for i = 1:nbSamples
                fprintf(file, "[");
                for j = 1:nbDOF
                    if j == nbDOF
                        fprintf(file, "%d ", Qd(i,j));
                    else
                        fprintf(file, "%d, ", Qd(i,j));
                    end
                end
                if i == nbSamples
                    fprintf(file, "]\n");
                else
                    fprintf(file, "],\n");
                end
            end
            
            fprintf(file, "        ]\n");
            fprintf(file, "\n");
            fprintf(file, "way_point_velocities:\n");
            fprintf(file, "        [\n");
            fprintf(file, "\n");
            
            QQ = augmentedDesiredState(2*nbDOF+1:end,:);
            Qd=QQ';
            QQp = augmentedDesiredState(nbDOF+1:2*nbDOF,:);
            Qpd=QQp';
            
            for i = 1:nbSamples
                fprintf(file, "[");
                for j = 1:nbDOF
                    if j == nbDOF
                        fprintf(file, "%d ", Qd(i,j));
                    else
                        fprintf(file, "%d, ", Qpd(i,j));
                    end
                end
                if i == nbSamples
                    fprintf(file, "]\n");
                else
                    fprintf(file, "],\n");
                end
                
            end
            fprintf(file, "        ]\n");
            fclose(file);
            
            
            
        end

	statusFlag = true;
    otherwise
        warning("So far the Experiment_ROS_Trajectory function is only working on the REEM-C and RV2SQ robot...");
        statusFlag = false;
end

end
