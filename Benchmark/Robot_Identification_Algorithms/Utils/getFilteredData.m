function [t, Tau, Qpp, Qp, Q, t_decim, Tau_decim, Qpp_decim, Qp_decim, Q_decim, options] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb, algName)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%

% Sampling epochs:
time = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples);
t = time(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
t_decim = decimate(t,benchmarkSettings.decimRate);
nbSample = numel(t);
nbSampleDecim = numel(t_decim);
options.nbSampleDecim = nbSampleDecim;

% Important: the 1/benchmarkSettings.f time shift at the beginning is the result of the 'previous' zero-order interpolation method
data = experimentDataStruct.getExperimentData(time, expNb, benchmarkSettings.interpolationAlgorithmExp);

Tau = zeros(robot.nbDOF, nbSample);
Q = zeros(robot.nbDOF, nbSample);
Qp = zeros(robot.nbDOF, nbSample);
Qpp = zeros(robot.nbDOF, nbSample);
Q_decim = zeros(robot.nbDOF, nbSampleDecim);
Qp_decim = zeros(robot.nbDOF, nbSampleDecim);
Qpp_decim = zeros(robot.nbDOF, nbSampleDecim);
Tau_ref = zeros(robot.nbDOF, nbSample);
Q_ref = zeros(robot.nbDOF, nbSample);
Qp_ref = zeros(robot.nbDOF, nbSample);
Qpp_ref = zeros(robot.nbDOF, nbSample);

switch benchmarkSettings.experimentOnTrueRobot
    
    case true
        
        switch options.filter
            
            case 'no'
                
                options.debugPlot = sprintf('Unfiltered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    
                    % Torque
                    if benchmarkSettings.identifyDriveGains == true 
                        tau = data.Taum(i,:);
                    else
                        tau = robot.controlParameters.Ktau_sign(i)*robot.controlParameters.Ktau(i,i)*data.Taum(i,:);
                    end
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = data.Qm(i,:);
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                end
            
            case 'lowpass'
                
                options.debugPlot = sprintf('Low-pass-filtered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    
                    % Torque
                    % tau = filter(benchmarkSettings.filter.lowpass.Hd,data.Taum(i,:));
                    if benchmarkSettings.identifyDriveGains == true 
                        tau = data.Taum(i,:);
                    else
                        tau = robot.controlParameters.Ktau_sign(i)*robot.controlParameters.Ktau(i,i)*data.Taum(i,:);
                    end
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = filter(benchmarkSettings.filter.lowpass.Hd,data.Qm(i,:));
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                end
                
            case 'butterworth'
                
                options.debugPlot = sprintf('Butterworth-filtered %s', algName);
                
                % Butterworth parameters
                ob = benchmarkSettings.filter.butterworth.freq_fil/benchmarkSettings.fnyq;
                [b,a] = butter(benchmarkSettings.filter.butterworth.nfilt,ob);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    
                    % Torque
                    % tau = filtfilt(b,a,data.Taum(i,:));
                    if benchmarkSettings.identifyDriveGains == true 
                        tau = data.Taum(i,:);
                    else
                        tau = robot.controlParameters.Ktau_sign(i)*robot.controlParameters.Ktau(i,i)*data.Taum(i,:);
                    end
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = filtfilt(b,a,data.Qm(i,:));
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                end
                
            otherwise
                
                options.debugPlot = sprintf('Unknown filter type ! Selecting unfiltered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    
                    % Torque
                    if benchmarkSettings.identifyDriveGains == true 
                        tau = data.Taum(i,:);
                    else
                        tau = robot.controlParameters.Ktau_sign(i)*robot.controlParameters.Ktau(i,i)*data.Taum(i,:);
                    end
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = data.Qm(i,:);
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                end
                
        end
        
        
    case false
        
        switch options.filter
            
            case 'no'
                
                options.debugPlot = sprintf('Unfiltered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    clear 'q_ref' 'qp_ref' 'qpp_ref' 'tau_ref'
                    
                    % Torque
                    tau = data.Taum(i,:);
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = data.Qm(i,:);
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                    % Reference data for standard deviation computation in ML:
                    
                    % Torque
                    tau_ref = data.Tau(i,:);
                    Tau_ref(i,:) = tau_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_ref_decim(i,:) = decimate(Tau_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q_ref = data.Q(i,:);%filtfilt(b,a,data.Q(i,:));
                    Q_ref(i,:) = q_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_ref_decim(i,:) = decimate(Q_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp_ref = diffcent(q_ref,benchmarkSettings.dt); %     qp_ref = data.Qp(i,:);
                    Qp_ref(i,:) = qp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_ref_decim(i,:) = decimate(Qp_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp_ref = diffcent(qp_ref,benchmarkSettings.dt); %data.Qppd(i,:);
                    Qpp_ref(i,:) = qpp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_ref_decim(i,:) = decimate(Qpp_ref(i,:),benchmarkSettings.decimRate);
                end
                
            case 'lowpass'
                
                options.debugPlot = sprintf('Low-pass-filtered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    clear 'q_ref' 'qp_ref' 'qpp_ref' 'tau_ref'
                    
                    % Torque
                    % tau = filter(benchmarkSettings.filter.lowpass.Hd,data.Taum(i,:));
                    tau = data.Taum(i,:);
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = filter(benchmarkSettings.filter.lowpass.Hd,data.Qm(i,:));
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                    % Reference data for standard deviation computation in ML:
                    
                    % Torque
                    tau_ref = data.Tau(i,:);
                    Tau_ref(i,:) = tau_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_ref_decim(i,:) = decimate(Tau_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q_ref = filter(benchmarkSettings.filter.lowpass.Hd,data.Q(i,:));
                    Q_ref(i,:) = q_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_ref_decim(i,:) = decimate(Q_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp_ref = diffcent(q_ref,benchmarkSettings.dt);
                    Qp_ref(i,:) = qp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_ref_decim(i,:) = decimate(Qp_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp_ref = diffcent(qp_ref,benchmarkSettings.dt);
                    Qpp_ref(i,:) = qpp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_ref_decim(i,:) = decimate(Qpp_ref(i,:),benchmarkSettings.decimRate);
                end
                
            case 'butterworth'
                
                options.debugPlot = sprintf('Butterworth-filtered %s', algName);
                
                % Butterworth parameters
                ob = benchmarkSettings.filter.butterworth.freq_fil/benchmarkSettings.fnyq;
                [b,a] = butter(benchmarkSettings.filter.butterworth.nfilt,ob);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    clear 'q_ref' 'qp_ref' 'qpp_ref' 'tau_ref'
                    
                    % Torque
                    % tau = filtfilt(b,a,data.Taum(i,:));
                    tau = data.Taum(i,:);
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = filtfilt(b,a,data.Qm(i,:));
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                    % Reference data for standard deviation computation in ML:
                    
                    % Torque
                    tau_ref = data.Tau(i,:);
                    Tau_ref(i,:) = tau_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_ref_decim(i,:) = decimate(Tau_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q_ref = filtfilt(b,a,data.Q(i,:));
                    Q_ref(i,:) = q_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_ref_decim(i,:) = decimate(Q_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp_ref = diffcent(q_ref,benchmarkSettings.dt);
                    Qp_ref(i,:) = qp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_ref_decim(i,:) = decimate(Qp_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp_ref = diffcent(qp_ref,benchmarkSettings.dt);
                    Qpp_ref(i,:) = qpp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_ref_decim(i,:) = decimate(Qpp_ref(i,:),benchmarkSettings.decimRate);
                end
                
            otherwise
                
                options.debugPlot = sprintf('Unknown filter type ! Selecting unfiltered %s', algName);
                
                for i = 1:robot.nbDOF
                    clear 'q' 'qp' 'qpp' 'tau'
                    clear 'q_ref' 'qp_ref' 'qpp_ref' 'tau_ref'
                    
                    % Torque
                    tau = data.Taum(i,:);
                    Tau(i,:) = tau(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_decim(i,:) = decimate(Tau(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q = data.Qm(i,:);
                    Q(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_decim(i,:) = decimate(Q(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp = diffcent(q,benchmarkSettings.dt);
                    Qp(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_decim(i,:) = decimate(Qp(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp = diffcent(qp,benchmarkSettings.dt);
                    Qpp(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_decim(i,:) = decimate(Qpp(i,:),benchmarkSettings.decimRate);
                    
                    % Reference data for standard deviation computation in ML:
                    
                    % Torque
                    tau_ref = data.Tau(i,:);
                    Tau_ref(i,:) = tau_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Tau_ref_decim(i,:) = decimate(Tau_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Position
                    q_ref = data.Q(i,:);%filtfilt(b,a,data.Q(i,:));
                    Q_ref(i,:) = q_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Q_ref_decim(i,:) = decimate(Q_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Velocity
                    qp_ref = diffcent(q_ref,benchmarkSettings.dt); %     qp_ref = data.Qp(i,:);
                    Qp_ref(i,:) = qp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qp_ref_decim(i,:) = decimate(Qp_ref(i,:),benchmarkSettings.decimRate);
                    
                    % Acceleration
                    qpp_ref = diffcent(qp_ref,benchmarkSettings.dt); %data.Qppd(i,:);
                    Qpp_ref(i,:) = qpp_ref(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
                    Qpp_ref_decim(i,:) = decimate(Qpp_ref(i,:),benchmarkSettings.decimRate);
                end
                
        end
        
        % figure()
        % plot((Q_ref_decim-Q_decim).')
        %
        % figure()
        % plot((Qp_ref_decim-Qp_decim).')
        %
        % figure()
        % plot((Qpp_ref_decim-Qpp_decim).')
        
        options.sigma_Q = std(Q_ref_decim-Q_decim,0,2);
        options.sigma_Qp = std(Qp_ref_decim-Qp_decim,0,2);
        options.sigma_Qpp = std(Qpp_ref_decim-Qpp_decim,0,2);
        options.sigma_Tau = std(Tau_ref_decim-Tau_decim,0,2);
        
    otherwise
        
        error('Unknown option')
end
% 
% 
% augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t, benchmarkSettings.interpolationAlgorithm); 
% for i = 1:3*robot.nbDOF
% augmentedDesiredState_decim(i,:) = decimate(augmentedDesiredState(i,:),benchmarkSettings.decimRate);
% end
% T = decimate(t,benchmarkSettings.decimRate);
% augmentedDesiredState_decim2 = benchmarkSettings.trajectoryData.getTrajectoryData(T, benchmarkSettings.interpolationAlgorithm); 
% 
% figure()
% plot(Q_decim.','r')
% hold on
% plot(augmentedDesiredState_decim(13:18,:).','b')
% hold on
% plot(augmentedDesiredState_decim2(13:18,:).','g--')
% 
% figure()
% plot(Qp_decim.','r')
% hold on
% plot(augmentedDesiredState_decim(7:12,:).','b')
% hold on
% plot(augmentedDesiredState_decim2(7:12,:).','g--')

% figure()
% plot(q)
% 
% figure()
% plot(qp)
% 
% figure()
% plot(data.Qpm.')
% pause

%  % Butterworth parameters
%                 ob = benchmarkSettings.filter.butterworth.freq_fil/benchmarkSettings.fnyq;
%                 [b,a] = butter(benchmarkSettings.filter.butterworth.nfilt,ob);
% 
% for i = 1:robot.nbDOF
%     clear 'q' 'qp' 'qpp' 'tau'
%     
%     % Position
%     q = filtfilt(b,a,data.Qm(i,:));
%     Qd(i,:) = q(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
%     
%     % Velocity
%     qp = diffcent(q,benchmarkSettings.dt);
%     Qpd(i,:) = qp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
%     
%     % Acceleration
%     qpp = diffcent(qp,benchmarkSettings.dt);
%     Qppd(i,:) = qpp(benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
%     
% end
% 
% trajectoryData.Q= Qd;
% trajectoryData.Qp= Qpd;
% trajectoryData.Qpp= Qppd;
% trajectoryData.t=t;
% 
% save('traj.mat', 'trajectoryData', '-v7.3')

% pause
end
