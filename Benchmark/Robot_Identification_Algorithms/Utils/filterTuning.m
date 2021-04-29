function [flag] = filterTuning(benchmarkSettings, experimentDataStruct)

t = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples); % Sampling times
data = experimentDataStruct.getExperimentData(t, 1, benchmarkSettings.interpolationAlgorithmExp);

nbDOF = numel(data.Qm(:,1));
robotName = benchmarkSettings.robotName;

%% No Filter:

for i = 1:nbDOF
    
    % Position
    Q_nf(i,:) = data.Qm(i,:);
    
    % Velocity
    Qp_nf(i,:) = diffcent(Q_nf(i,:),benchmarkSettings.dt);
    
    % Acceleration
    Qpp_nf(i,:) = diffcent(Qp_nf(i,:),benchmarkSettings.dt);
    
    % Torque
    Tau_nf(i,:) = data.Taum(i,:);
    
end

%% Low-Pass Filter:

fvtool(benchmarkSettings.filter.lowpass.Hd)

for i = 1:nbDOF
    
    % Position
    Q_lpf(i,:) = filter(benchmarkSettings.filter.lowpass.Hd, data.Qm(i,:));
    
    % Velocity
    Qp_lpf(i,:) = diffcent(Q_lpf(i,:),benchmarkSettings.dt);
    
    % Acceleration
    Qpp_lpf(i,:) = diffcent(Qp_lpf(i,:),benchmarkSettings.dt);
    
    % Torque
    Tau_lpf(i,:) = filter(benchmarkSettings.filter.lowpass.Hd, data.Taum(i,:));
    
end

%% 2 Pass Butterworth Filter:

ob = benchmarkSettings.filter.butterworth.freq_fil/benchmarkSettings.fnyq;
[b,a] = butter(benchmarkSettings.filter.butterworth.nfilt,ob);
freqz(b,a)

for i = 1:nbDOF
    
    % Position
    Q_bwf(i,:) = filtfilt(b,a,data.Qm(i,:));
    
    % Velocity
    Qp_bwf(i,:) = diffcent(Q_bwf(i,:),benchmarkSettings.dt);
    
    % Acceleration
    Qpp_bwf(i,:) = diffcent(Qp_bwf(i,:),benchmarkSettings.dt);
    
    % Torque
    Tau_bwf(i,:) = filtfilt(b,a,data.Taum(i,:));
    
end

%% Plots

if benchmarkSettings.experimentOnTrueRobot == true
    
    Qm = data.Qm;
    Taum = data.Taum;
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Qm(i,:));
        hold on
        plot(t, Q_lpf(i,:), 'linewidth', 2);
        hold on
        plot(t, Q_bwf(i,:), 'linewidth', 2);
        hold off
        legend('Raw', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Q')
        title(sprintf('\\textbf{Filtered Position} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Qp_nf(i,:), 'linewidth', 2);
        hold on
        plot(t, Qp_lpf(i,:), 'linewidth', 2);
        hold on
        plot(t, Qp_bwf(i,:), 'linewidth', 2);
        hold off
        legend('Unfiltered', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Qp')
        title(sprintf('\\textbf{Filtered Velocity} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Qpp_nf(i,:), 'linewidth', 2);
        hold on
        plot(t, Qpp_lpf(i,:), 'linewidth', 2);
        hold on
        plot(t, Qpp_bwf(i,:), 'linewidth', 2);
        hold off
        legend('Unfiltered', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Qpp')
        title(sprintf('\\textbf{Filtered Acceleration} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Taum(i,:));
        hold on
        plot(t, Tau_lpf(i,:), 'linewidth', 2);
        hold on
        plot(t, Tau_bwf(i,:), 'linewidth', 2);
        hold off
        legend('Raw', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Tau')
        title(sprintf('\\textbf{Filtered Torque} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
else
    Q = data.Q;
    Qp = data.Qp;
    Qpp = data.Qpp;
    Tau = data.Tau;
    
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Q(i,:));
        hold on
        plot(t, Q_nf(i,:), 'r');
        hold on
        plot(t, Q_lpf(i,:), 'linewidth', 1);
        hold on
        plot(t, Q_bwf(i,:), 'g--', 'linewidth', 1);
        hold off
        legend('True', 'Unfiltered', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Q')
        title(sprintf('\\textbf{Filtered Position} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Qp(i,:));
        hold on
        plot(t, Qp_nf(i,:), 'r');
        hold on
        plot(t, Qp_lpf(i,:), 'linewidth', 1);
        hold on
        plot(t, Qp_bwf(i,:), 'g--', 'linewidth', 1);
        hold off
        legend('True', 'Unfiltered', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Qp')
        title(sprintf('\\textbf{Filtered Velocity} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Qpp(i,:));
        hold on
        plot(t, Qpp_nf(i,:), 'r');
        hold on
        plot(t, Qpp_lpf(i,:), 'linewidth', 1);
        hold on
        plot(t, Qpp_bwf(i,:), 'g--', 'linewidth', 1);
        hold off
        legend('True', 'Unfiltered', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Qpp')
        title(sprintf('\\textbf{Filtered Acceleration} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
    
    figure()
    for i = 1:nbDOF
        subplot(ceil(nbDOF/2),2,i)
        plot(t, Tau(i,:));
        hold on
        plot(t, Tau_lpf(i,:), 'linewidth', 1);
        hold on
        plot(t, Tau_bwf(i,:), 'linewidth', 1);
        hold off
        legend('True', 'Low-Pass Filtered', '2-Pass-Butterworth', 'Orientation','horizontal')
        xlabel('Time [s]')
        ylabel('Tau')
        title(sprintf('\\textbf{Filtered Torque} \\boldmath{$%s$} \\textbf{joint %d}',robotName, i),'Interpreter', 'LaTeX')
        grid on
        grid minor
    end
end

pause

end
