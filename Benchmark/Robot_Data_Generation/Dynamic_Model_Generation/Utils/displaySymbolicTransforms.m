function displaySymbolicTransforms(robot)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% A simple display function for the symbolic expressions of the robot kinematics.

disp(' ')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('%%%%%%  DISPLAYING SYMBOLIC EXPRESSIONS OF ROBOT KINEMATICS  %%%%%%%');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(' ')

%% Display the symbolic expression of the base parameters:

Beta = feval(sprintf('Regressor_Beta_%s', robot.name),robot.symbolicParameters.Xhi);
fprintf('Beta = ');
fprintf('\n');
disp(Beta);

%% Display the symbolic expression of each robot link's Homogeneous Transformation w.r.t its parent link:
for i=1:robot.nbDOF
    Hd = feval(sprintf('HT_dh%d_dh%d_%s', i, i-1,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry);
    fprintf('HT_dh%d_dh%d_%s = ', i, i-1,robot.name);
    fprintf('\n');
    disp(Hd);
end

%% Display the symbolic expression of each robot link's Homogeneous Transformation w.r.t world frame:
for i=1:robot.nbDOF
    Hd = feval(sprintf('HT_dh%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry);
    fprintf('HT_dh%d_world_%s = ', i, i-1,robot.name);
    fprintf('\n');
    disp(Hd);
end

%% Display the symbolic expression of each robot link's Jacobian Matrix w.r.t world frame:
for i=1:robot.nbDOF
    J = feval(sprintf('J_dh%d_world_%s', i ,robot.name),robot.symbolicParameters.Q, robot.symbolicParameters.Geometry);
    fprintf('J_dh%d_world_%s = ', i, i-1,robot.name);
    fprintf('\n');
    disp(J);
end

end

