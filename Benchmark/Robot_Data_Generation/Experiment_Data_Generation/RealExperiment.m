classdef RealExperiment < handle
    
    % Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
    %
    % This class is a data container for robot real experiment data. The experiment data 
    % acquired on a real robot can be stored in a RealExperiment object provided that it is
    % in the form of a set of points, regrouped in a structure with the following fields: 
    % [time, Qm, Qpm, Xm, Xpm, Qd, Qpd, Qppd, Xd, Xpd, Taum].
    % The Experiment class provides dedicated interpolation routines that allows getting a
    % data point at any epoch.
    %
    % The process is the following:
    %
    % Assuming that you already have the experiment data fields availabe in the
    % workspace, you should first instantiate a Experiment object and fill it with the data:
    %
    % exp = RealExperiment;
    % exp.setExperimentData(experimentData);
    %
    % To get the experiment data at any epoch t, one then simply have to  call:
    %
    % exp.getExperimentData(t);
    
    properties
        data = struct('time', [], 'Qd', [], 'Qpd', [], 'Qppd', [], 'Xd', [], 'Xpd', [], 'Qm', [], 'Qpm', [], 'Xm', [], 'Xpm', [], 'Taum', []);
        % time [s]
        % Q [rad]
        % Qp [rad/s]
        % Qpp [rad/s^2]
        % X [m]
        % Xp [m/s]
        % Tau [N.m]
    end
    
    properties (Dependent = true)
        t_i                 % First trajectory epoch [s].
        t_f                 % Last trajectory epoch [s].
        samplingFrequency   % Sampling frequency of the trajectory data points [Hz].
        nbDOF               % Number of Degrees of Freedom of the robot.
    end
    
    methods
        function this = setExperimentData(this, experimentData)
            [a_q,b_q,c_q] = size(experimentData.Qm);
            
            if ~isequal(size(experimentData.Qd), size(experimentData.Qpd), size(experimentData.Qppd), size(experimentData.Qm), size(experimentData.Qpm), size(experimentData.Taum))
                error('Experiment data container: incorrect data dimention!')
            elseif a_q>b_q
                this.data.time = experimentData.time;
                this.data.Qd = permute(experimentData.Qd,[2,1,3]);
                this.data.Qpd = permute(experimentData.Qpd,[2,1,3]);
                this.data.Qppd = permute(experimentData.Qppd,[2,1,3]);
                this.data.Xd = permute(experimentData.Xd,[2,1,3]);
                this.data.Xpd = permute(experimentData.Xpd,[2,1,3]);
                this.data.Qm = permute(experimentData.Qm,[2,1,3]);
                this.data.Qpm = permute(experimentData.Qpm,[2,1,3]);
                this.data.Xm = permute(experimentData.Xm,[2,1,3]);
                this.data.Xpm = permute(experimentData.Xpm,[2,1,3]);
                this.data.Taum = permute(experimentData.Taum,[2,1,3]);
            else
                this.data.time = experimentData.time;
                this.data.Qd = experimentData.Qd;
                this.data.Qpd = experimentData.Qpd;
                this.data.Qppd = experimentData.Qppd;
                this.data.Xd = experimentData.Xd;
                this.data.Xpd = experimentData.Xpd;
                this.data.Qm = experimentData.Qm;
                this.data.Qpm = experimentData.Qpm;
                this.data.Xm = experimentData.Xm;
                this.data.Xpm = experimentData.Xpm;
                this.data.Taum = experimentData.Taum;
            end
        end
        
        function experimentDataStruct = getExperimentData(this, time, expNb, method, varargin)
            if nargin == 3
                for i = 1:size(this.data.Qm,1)
                    Qd(i,:) = interp1(this.data.time,this.data.Qd(i,:,expNb),time);
                    Qpd(i,:) = interp1(this.data.time,this.data.Qpd(i,:,expNb),time);
                    Qppd(i,:) = interp1(this.data.time,this.data.Qppd(i,:,expNb),time);
                    Qm(i,:) = interp1(this.data.time,this.data.Qm(i,:,expNb),time);
                    Qpm(i,:) = interp1(this.data.time,this.data.Qpm(i,:,expNb),time);
                    Taum(i,:) = interp1(this.data.time,this.data.Taum(i,:,expNb),time);
                end
                for j = 1:size(this.data.Xm,1)
                    Xd(j,:) = interp1(this.data.time,this.data.Xd(j,:,expNb),time);
                    Xpd(j,:) = interp1(this.data.time,this.data.Xpd(j,:,expNb),time);
                    Xm(j,:) = interp1(this.data.time,this.data.Xm(j,:,expNb),time);
                    Xpm(j,:) = interp1(this.data.time,this.data.Xpm(j,:,expNb),time);
                end
            elseif nargin == 4
                % interpolation method = 'linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
                for i = 1:size(this.data.Qm,1)
                    Qd(i,:) = interp1(this.data.time,this.data.Qd(i,:,expNb),time,method);
                    Qpd(i,:) = interp1(this.data.time,this.data.Qpd(i,:,expNb),time,method);
                    Qppd(i,:) = interp1(this.data.time,this.data.Qppd(i,:,expNb),time,method);
                    Qm(i,:) = interp1(this.data.time,this.data.Qm(i,:,expNb),time,method);
                    Qpm(i,:) = interp1(this.data.time,this.data.Qpm(i,:,expNb),time,method);
                    Taum(i,:) = interp1(this.data.time,this.data.Taum(i,:,expNb),time,method);
                end
                for j = 1:size(this.data.Xm,1)
                    Xd(j,:) = interp1(this.data.time,this.data.Xd(j,:,expNb),time,method);
                    Xpd(j,:) = interp1(this.data.time,this.data.Xpd(j,:,expNb),time,method);
                    Xm(j,:) = interp1(this.data.time,this.data.Xm(j,:,expNb),time,method);
                    Xpm(j,:) = interp1(this.data.time,this.data.Xpm(j,:,expNb),time,method);
                end
            else
                error('Experiment data container: incorrect number of arguments!');
            end
            experimentDataStruct.Qd = Qd;
            experimentDataStruct.Qpd = Qpd;
            experimentDataStruct.Qppd = Qppd;
            experimentDataStruct.Xd = Xd;
            experimentDataStruct.Xpd = Xpd;
            experimentDataStruct.Qm = Qm;
            experimentDataStruct.Qpm = Qpm;
            experimentDataStruct.Taum = Taum;
            experimentDataStruct.Xm = Xm;
            experimentDataStruct.Xpm = Xpm;
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

