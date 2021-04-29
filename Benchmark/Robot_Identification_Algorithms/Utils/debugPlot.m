function debugPlot(robot, benchmarkSettings, experimentDataStruct, Beta_0, estimatedBeta, options, expNb, algName, Betas, it, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Generate a set of debug plots allowing to visualize the relevance of a
% given set of parameters along a given trajectory.

if nargin < 9
    Betas = estimatedBeta;
    it = 0;
end

if options.debug == true
    [estimatedBeta benchmarkSettings.Beta_obj]
    options.filter = 'butterworth';
    [~, Tau, Qpp, Qp, Q, ~, ~, ~, ~, ~, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb, algName);
    
    jointNb = 3;
    
    tau = Tau(jointNb,:)';
    q = Q(jointNb,:)';
    qp = Qp(jointNb,:)';
    qpp = Qpp(jointNb,:)';
    
    % Simulation of the robot using the computed parameters:
    t_control = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_control, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    
    % Simulation of the robot:
    if strcmp(benchmarkSettings.codeImplementation,'optim')
        % Simulation of the robot using the current parameters:
        if Beta_0 ~= estimatedBeta % In case the algorithm requires an initial estimate (e.g. CLIE, CLOE, IV, DIDIM...)
            [~, State_0, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, Beta_0, robot.name, robot.numericalParameters.Geometry, ...
                robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
                robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
                robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
                robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
        end
        [~, State_it, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, estimatedBeta, robot.name, robot.numericalParameters.Geometry, ...
            robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
            robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
            robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
            robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    elseif strcmp(benchmarkSettings.codeImplementation,'classic')
        % Simulation of the robot using the current parameters:
        if Beta_0 ~= estimatedBeta % In case the algorithm requires an initial estimate (e.g. CLIE, CLOE, IV, DIDIM...)
            [~, State_0, ~] = integrateClosedLoopDynamics(augmentedDesiredState, Beta_0, robot.name, robot.numericalParameters.Geometry, ...
                robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
                robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
                robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
                robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
        end
        [~, State_it, ~] = integrateClosedLoopDynamics(augmentedDesiredState, estimatedBeta, robot.name, robot.numericalParameters.Geometry, ...
            robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
            robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
            robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
            robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    else
        error('%s: unknown implementation option', algName);
    end
    
    
    % Compute the observation matrix:
    W = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);
    
    referenceTau = W*benchmarkSettings.Beta_obj;
    recomputedTau = W*estimatedBeta;
    recomputedTau0 = W*Beta_0;
    
    figure('Name',sprintf('Torque %s',options.debugPlot));
    plot1 = plot(tau,'b','LineWidth',0.5);
    hold on
    plot(referenceTau(jointNb:robot.nbDOF:end),'g', 'LineWidth',2);
    hold on
    plot(recomputedTau0(jointNb:robot.nbDOF:end),'r--','LineWidth',1);
    hold on
    plot(recomputedTau(jointNb:robot.nbDOF:end),'m','LineWidth',1);
    legend({'real noisy signal', 'reference parameters', 'initial parameters',sprintf('%s parameters',algName)})
    plot1(1,1).Color(4)=0.25;
    xlabel('Samples');
    ylabel('Tau');
    title('Recomputed torque')
    grid on
    grid minor
    
    if Beta_0 ~= estimatedBeta % In case the algorithm requires an initial estimate (e.g. CLIE, CLOE, IV, DIDIM...), plot both the initial estimate and the final estimate trajectories.
        Err=(q-State_it(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)');
        Err0=(q-State_0(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)');
        figure('Name',sprintf('%s',options.debugPlot));
        subplot(4,1,1)
        plot(State_0(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)','--','LineWidth',2)
        hold on
        plot(q,'LineWidth',1)
        legend(sprintf('q%s simulated',jointNb),sprintf('q%s robot',jointNb))
        xlabel('Samples');
        ylabel('Q');
        title('Trajectory initial parameter set')
        grid on
        grid minor
        subplot(4,1,2)
        plot(100*Err0)
        legend('100 x Error')
        xlabel('Samples');
        ylabel('Joint Error');
        title('Trajectory Error initial parameter set')
        grid on
        grid minor
        subplot(4,1,3)
        plot(State_it(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)','--','LineWidth',2)
        hold on
        plot(q,'LineWidth',1)
        legend(sprintf('q%s simulated',jointNb),sprintf('q%s robot',jointNb))
        xlabel('Samples');
        ylabel('Q');
        title(sprintf('Trajectory %s',algName))
        grid on
        grid minor
        subplot(4,1,4)
        plot(100*Err)
        legend('100 x Error')
        xlabel('Samples');
        ylabel('Joint Error');
        title(sprintf('Trajectory Error %s',algName))
        grid on
        grid minor
    else % Otherwise just plot both the final estimate trajectory...
        Err=(q-State_it(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)');
        figure('Name',sprintf('%s',algName));
        subplot(2,1,1)
        plot(State_it(robot.nbDOF+jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder)','--','LineWidth',2)
        hold on
        plot(q,'LineWidth',1)
        legend(sprintf('q%s simulated',jointNb),sprintf('q%s robot',jointNb))
        xlabel('Samples');
        ylabel('Q');
        title(sprintf('Trajectory %s', algName))
        grid on
        grid minor
        subplot(2,1,2)
        plot(100*Err)
        legend('100 x Error')
        xlabel('Samples');
        ylabel('Joint Error');
        title(sprintf('Trajectory Error %s',algName))
        grid on
        grid minor
    end
    
    if strcmp(algName,'ANN') || strcmp(algName,'HTRNN')
        if options.debug == true
            cc=parula(robot.paramVectorSize);
            figure('Name',sprintf('%s',options.debugPlot));
            for i=1:robot.paramVectorSize
                plot(Betas(i,:)','LineWidth',2 ,'color',cc(i,:))
                hold on
                plot(repmat(benchmarkSettings.Beta_obj(i),1,it-1)','--','LineWidth',2,'color',cc(i,:))
            end
            hold off
            legend('Identified Parameters','True Parameters')
            xlabel('Iterations');
            ylabel('Beta');
            title(sprintf('Parameter Convergence of %s Parameter Estimation',algName))
            ylim([-5,5])
            grid on
            grid minor
            
            err = Betas-repmat(benchmarkSettings.Beta_obj,1,it-1);
            err2 = sum(err.^2,1);
            
            figure('Name','Convergence');
            plot(err2,'LineWidth',2 )
            legend('Squared Error')
            xlabel('Iterations');
            ylabel('Error');
            title(sprintf('Convergence of %s Parameter Estimation',algName))
            grid on
            grid minor
        end
    end
    pause
end
end

