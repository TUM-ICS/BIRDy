classdef Trajectory < handle
    
    % Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
    % 
    % This class is a data container for robot trajectories. A given robot
    % trajectory can be stored in a Trajectory object provided that it is
    % in the form of a set of points with the following fields: [time, Q,
    % Qp,Qpp]. The Trajectory class provides dedicated interpolation
    % routines that allows getting a reference point at any epoch. 
    % 
    % The process is the following: 
    %
    % Assuming that you already have the data fields "time", "Q", "Qp" and
    % "Qpp" availabe in the workspace, you should first instantiate a
    % Trajectory object and fill it with the data:
    %
    % traj = Trajectory;
    % traj.setTrajectoryData(time, Q, Qp, Qpp);
    % 
    % To get the reference trajectory at any epoch t, one then simply have
    % to  call: 
    % 
    % traj.getTrajectoryData(t);
    
    properties
        data = struct('time', [], 'Q', [], 'Qp', [], 'Qpp', []);
        % time [s]
        % Q [rad]
        % Qp [rad/s]
        % Qpp [rad/s^2]
    end
    
    properties (Dependent = true)
        t_i                 % First trajectory epoch [s].
        t_f                 % Last trajectory epoch [s].
        samplingFrequency   % Sampling frequency of the trajectory data points [Hz].
        nbDOF               % Number of Degrees of Freedom of the robot.
    end
    
    methods
        function this = setTrajectoryData(this, time, Q, Qp, Qpp)
            [mq,nq] = size(Q);
            [mqp,nqp] = size(Qp);
            [mqpp,nqpp] = size(Qpp);
            
            if (mq ~= mqp) || (mqp ~= mqpp) ||(mq ~= mqpp) || (nq ~= nqp) || (nqp ~= nqpp) ||(nq ~= nqpp)
                error('Trajectory engine: incorrect data dimention!')
            elseif mq>nq
                this.data.time = time;
                this.data.Q = Q';
                this.data.Qp = Qp';
                this.data.Qpp = Qpp';
            else
                this.data.time = time;
                this.data.Q = Q;
                this.data.Qp = Qp;
                this.data.Qpp = Qpp;
            end
            
        end
        
        function augmentedState = getTrajectoryData(this, time, method, varargin)
            if nargin == 2
                for i = 1:size(this.data.Q,1)
                    Q(i,:) = interp1(this.data.time,this.data.Q(i,:),time);
                    Qp(i,:) = interp1(this.data.time,this.data.Qp(i,:),time);
                    Qpp(i,:) = interp1(this.data.time,this.data.Qpp(i,:),time);
                end
                augmentedState = [Qpp; Qp; Q];
            elseif nargin == 3
                % interpolation method = 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
                for i = 1:size(this.data.Q,1)
                    Q(i,:) = interp1(this.data.time,this.data.Q(i,:),time,method);
                    Qp(i,:) = interp1(this.data.time,this.data.Qp(i,:),time,method);
                    Qpp(i,:) = interp1(this.data.time,this.data.Qpp(i,:),time,method);
                end
                augmentedState = [Qpp; Qp; Q];
            else
                error('Trajectory engine: incorrect number of arguments!');
            end
        end
        
        function value = get.samplingFrequency(this) % in [Hz]
            value = (this.data.time(end) - this.data.time(1))/numel(this.data.time);
        end
        
        function value = get.t_i(this) % in [s]
            value = this.data.time(1);
        end
        
        function value = get.t_f(this) % in [s]
            value = this.data.time(end);
        end
    end
end

