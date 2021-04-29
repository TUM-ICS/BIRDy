function [Beta_KF, State, Covariance, pNoise, errorFlag] = KF_identification(robot, benchmarkSettings, experimentDataStruct, expNb, Beta_0, optionsKF)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Parameter identification using Nonlinear Kalman filtering.

%% Data Decimation and Filtering:

optionsKF.filter = 'no'; % Otherwise there is no interest at all in using a Kalman filter with an extended state right ?
[t, ~, ~, ~, ~, t_decim, Tau_decim, Qpp_decim, Qp_decim, Q_decim, optionsKF] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, optionsKF, expNb, optionsKF.type);

%% Kalman Filter Parameter Initialization

[x_aug, P, S, Rv, Rn, Sv, Sn, Particules, w, alpha, beta, kappa, h, paramSize, pNoiseAdaptParams, State, Covariance, pNoise] = initKalman(robot, benchmarkSettings, Tau_decim, Qpp_decim, Qp_decim, Q_decim, expNb, Beta_0, optionsKF);

errorFlag = 1;
it=1;

%% Kalman Filter Loop

% iDq=zeros(robot.nbDOF,1);
counter=1;
if optionsKF.debug == true
    StateDebug = [];
    figure('Name','Covariance Matrix');
end

for index=2:optionsKF.nbSampleDecim
    
    % Acquire measurement:
    y = acquireMeasurement(Tau_decim, Qpp_decim, Qp_decim, Q_decim, index);
    
    % Process noise annealing for the parameters (inspired by the ReBEL toolkit from Rudolph Van der Merwe):
    [Rv, Sv, pNoiseAdaptParams] = annealingNoiseCovariance(P, Rv, Sv, pNoiseAdaptParams, paramSize, optionsKF);
    Tau = Tau_decim(:, index-1);
    % Compute torque command:
    if optionsKF.useComputedTorque == true
        % Use the computed torque as a control input: 
        t_ctrl = linspace(t_decim(index-1), t_decim(index), (t_decim(index)-t_decim(index-1))/benchmarkSettings.dt_ctrl);  % Control epochs btween two Kalman iterations
        augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_ctrl, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
        % Nonlinear Kalman Filter:
        [x_aug, P, S, Particules, w, status] = iterateKalman(robot, benchmarkSettings, index, t_decim, optionsKF, x_aug, Tau, P, S, y, Rv, Rn, Sv, Sn, Particules, w, alpha, beta, kappa, h, augmentedDesiredState, optionsKF.useComputedTorque);
    else
        % Use the measured torque as a control input:
        Tau = Tau_decim(:, index);
        % Nonlinear Kalman Filter:
        [x_aug, P, S, Particules, w, status] = iterateKalman(robot, benchmarkSettings, index, t_decim, optionsKF, x_aug, Tau, P, S, y, Rv, Rn, Sv, Sn, Particules, w, alpha, beta, kappa, h, 0, optionsKF.useComputedTorque);
    end
    if optionsKF.debug == true
        StateDebug = [StateDebug; x_aug.'];
    end
    if strcmp(status, 'error')
        errorFlag = 0;
        break;
    end
    
    if mod(it,optionsKF.samplingFactor)==0
        if optionsKF.debug == true
            fprintf('Debug: Initial parameter error = %d\n', norm(Beta_0-benchmarkSettings.Beta_obj));
            fprintf('Debug: Current parameter error = %d\n', norm(x_aug(end-paramSize+1:end)-benchmarkSettings.Beta_obj));
        else
            fprintf('.')
        end
        pNoise(counter) = pNoiseAdaptParams.cov(1);
        State(counter,:) = x_aug.';
        Covariance(:,:,counter) = P;
        if optionsKF.debug == true
            image(1e3*P);
            title('Covariance Matrix')
            drawnow
        end
        counter = counter+1;
    end
    it = it+1;
end

fprintf('.\n')

%% Debug plot

if optionsKF.debug == true
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    for i = 1:3*robot.nbDOF
        augmentedDesiredState_decim(i,:) = decimate(augmentedDesiredState(i,:),benchmarkSettings.decimRate);
    end
    
    figure('Name',sprintf('%s: Position Estimation',optionsKF.type));
    for jointNb = 1:robot.nbDOF
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        graph(1) = plot(augmentedDesiredState_decim(2*robot.nbDOF+jointNb,:),'Color',[0,0,0.5], 'LineWidth',1);
        hold on
        graph(2) = plot(Q_decim(jointNb,2:end),'Color',[0.8,0,0],'LineWidth',1);
        hold on
        graph(3) = plot(StateDebug(:,robot.nbDOF+jointNb),'Color',[0,0.75,0], 'LineWidth',1);
        xlabel('\textbf{Iterations}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint position [rad]}','Interpreter', 'LaTeX');
        title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
        grid on
        grid minor
    end
    sgtitle('Nonlinear Kalman Position Estimation','Interpreter', 'LaTeX')
    legendInfo{1}='Desired';
    legendInfo{2}='Measured';
    legendInfo{3}='Kalman estimate';
    legend(graph,legendInfo{:},'Interpreter', 'LaTeX')
    
    
    figure('Name',sprintf('%s: Velocity Estimation',optionsKF.type));
    for jointNb = 1:robot.nbDOF
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        graph(1) = plot(augmentedDesiredState_decim(robot.nbDOF+jointNb,:),'Color',[0,0,0.5], 'LineWidth',1);
        hold on
        graph(2) = plot(Qp_decim(jointNb,2:end).','Color',[0.8,0,0], 'LineWidth',1);
        hold on
        graph(3) = plot(StateDebug(:,jointNb),'Color',[0,0.75,0], 'LineWidth',1);
        xlabel('\textbf{Iterations}','Interpreter', 'LaTeX');
        ylabel('\textbf{Joint velocity [rad/s]}','Interpreter', 'LaTeX');
        title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
        grid on
        grid minor
    end
    sgtitle('Nonlinear Kalman Velocity Estimation','Interpreter', 'LaTeX')
    legendInfo{1}='Desired';
    legendInfo{2}='Measured';
    legendInfo{3}='Kalman estimate';
    legend(graph,legendInfo{:},'Interpreter', 'LaTeX')
    
    figure('Name',sprintf('%s: Parameter Estimation',optionsKF.type));
    cc=parula(robot.paramVectorSize);
    for i=1:robot.paramVectorSize
        plot(StateDebug(:,2*robot.nbDOF+i),'LineWidth',2 ,'color',cc(i,:))
        hold on
        plot(repmat(benchmarkSettings.Beta_obj(i),1,numel(t_decim)-1)','--','LineWidth',2,'color',cc(i,:))
    end
    legend('Identified Parameters','True Parameters')
    xlabel('Iterations');
    ylabel('Beta');
    title('Nonlinear Kalman Parameter Estimation')
    grid on
    grid minor
    
    err = State(2:end-9,2*robot.nbDOF+1:end)-repmat(benchmarkSettings.Beta_obj',size(State,1)-10,1);
    err2 = sum(err'.^2,1);
    
    figure('Name','Convergence');
    subplot(2,1,1)
    plot(err2,'LineWidth',2 )
    legend('Squared Error')
    ylim([0 10]);
    xlabel('Iterations');
    ylabel('Error');
    title('Convergence of KF Parameter Estimation')
    grid on
    grid minor
    subplot(2,1,2)
    plot(pNoise(2:end),'LineWidth',2)
    legend('Annealing factor')
    xlabel('Samples');
    ylabel('Annealing');
    title('Covariance Matrix Annealing')
    grid on
    grid minor
    
    pause
end

Beta_KF = x_aug(end-paramSize+1:end);

end

