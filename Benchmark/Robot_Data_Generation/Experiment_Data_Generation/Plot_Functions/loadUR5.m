function [L0, UR5,UR5_add,time] = loadUR5(robot, Q)

tic

%% LOAD UR5 ROBOT GEOMETRY AND SET THE CORRECT ORIGIN AND ORIENTATION OF BLOCKS

L0 = stlread('UR5/L0.STL');
L0.vertices=0.001*L0.vertices;
for i = 1 : size(L0.vertices,1)
    C = L0.vertices(i,:);
    L0.vertices(i,:)=(RotY(-pi/2)*C')';% OK
end
UR5(1) = stlread('UR5/L1.STL');
UR5(1).vertices=0.001*UR5(1).vertices;
T_stl(:,:,1) = eye(4);
T_stl(1:3,1:3,1) = RotZ(-pi/2)*RotX(-pi/2);% OK
T_stl(1:3,4,1) = [-0.108; 0.009; 0.089]; % OK

UR5(2) = stlread('UR5/L2.STL');
UR5(2).vertices=0.001*UR5(2).vertices;
T_stl(:,:,2) = eye(4);
T_stl(1:3,1:3,2) = RotZ(-3*pi/2)*RotY(pi/2);% OK
T_stl(1:3,4,2) = [0.09; -0.1075; 0.145]; % OK

UR5(3) = stlread('UR5/L3.STL');
UR5(3).vertices=0.001*UR5(3).vertices;
T_stl(:,:,3) = eye(4);
T_stl(1:3,1:3,3) = RotZ(pi/2)*RotX(pi/2);% OK
T_stl(1:3,4,3) = [-0.11; 0.03; 0.01]; % OK

UR5(4) = stlread('UR5/L45.STL');
UR5(4).vertices=0.001*UR5(4).vertices;
T_stl(:,:,4) = eye(4);
T_stl(1:3,1:3,4) = RotX(pi/2);% OK      
T_stl(1:3,4,4) = [0.03; 0.112; -0.0085];% OK

UR5(5)=UR5(4);
T_stl(:,:,5) = eye(4);
T_stl(1:3,1:3,5) = RotX(-pi/2)*RotZ(pi);% OK
T_stl(1:3,4,5) = [-0.03; -0.1085; -0.0065];% OK

UR5(6) = stlread('UR5/L6.STL');
UR5(6).vertices=0.001*UR5(6).vertices;
T_stl(:,:,6) = eye(4);
T_stl(1:3,1:3,6) = RotY(-pi/2);% OK
T_stl(1:3,4,6) = [0; 0.001; -0.032];% OK

UR5_add(2) = stlread('UR5/Dog-assembly.STL');
UR5_add(2).vertices=0.001*UR5_add(2).vertices;
T_stl_add(:,:,1) = eye(4);
T_stl_add(1:3,1:3,1) = RotY(pi);
T_stl_add(1:3,4,1) = [0; 0.1; 0];

UR5_add(2)= stlread('UR5/Tunnel-assembly.STL');
UR5_add(2).vertices=0.001*UR5_add(2).vertices;
T_stl_add(:,:,2) = eye(4);
T_stl_add(1:3,1:3,2) = RotY(-pi/2);
T_stl_add(1:3,4,2) = [0; 0.001; -0.032];

%% APPLY THE JOINT ANGLES

for indexJoint = 1:6
    fprintf('Processing .STL of joint %d...\n', indexJoint);
    H_link = feval(sprintf('HT_dh%d_world_%s', indexJoint,robot.name), Q, robot.numericalParameters.Geometry);
    T_STL = H_link*T_stl(:,:,indexJoint);
    sz = size(UR5(indexJoint).vertices,1);
    UR5(indexJoint).vertices = UR5(indexJoint).vertices*transpose(T_STL(1:3,1:3));
    UR5(indexJoint).vertices = UR5(indexJoint).vertices + [T_STL(1,4)*ones(sz,1) T_STL(2,4)*ones(sz,1) T_STL(3,4)*ones(sz,1)];
end

time=toc;
end

