function [Xhi_U, Xhi_L, Beta_obj, Beta_U, Beta_L, Initial_Beta, Initial_Xhi] = generateInitParamEst(robot, benchmarkSettings, experimentDataStruct)

% Authors: Julien Roux, Quentin Leboutet, Alexandre Janot, Gordon Cheng
% This function returns the set of initial parameters used for identification.

numberOfInitialEstimates = benchmarkSettings.numberOfInitialEstimates;

% Real parameters:
Xhi_obj = robot.numericalParameters.Xhi;

% Parameter bounds (to be replaced by LMI):



switch robot.frictionIdentModel
    % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
    % Nonlinear friction models require state dependant parameter identification
    case 'no'
        for i = 1:robot.nbDOF
            Xhi_U(12*(i-1)+1:12*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;100];           % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
            Xhi_L(12*(i-1)+1:12*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;-100];                % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
        end
    case 'Viscous'
        for i = 1:robot.nbDOF
            Xhi_U(13*(i-1)+1:13*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;10;100];        % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, tau_offi];
            Xhi_L(13*(i-1)+1:13*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;0;-100];              % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, tau_offi];
        end
    case 'Coulomb'
        for i = 1:robot.nbDOF
            Xhi_U(13*(i-1)+1:13*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;10;100];        % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fci, tau_offi];
            Xhi_L(13*(i-1)+1:13*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;0;-100];              % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fci, tau_offi];
        end
    case 'ViscousCoulomb'
        for i = 1:robot.nbDOF
            Xhi_U(13*(i-1)+1:13*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;10;10];     % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, Fci];
            Xhi_L(13*(i-1)+1:13*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;0;0];            % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, Fci];
        end
    case 'ViscousCoulombOff'
        for i = 1:robot.nbDOF
            Xhi_U(14*(i-1)+1:14*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;10;10;100];     % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, Fci, tau_offi];
            Xhi_L(14*(i-1)+1:14*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;0;0;-100];            % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, Fvi, Fci, tau_offi];
        end
    case 'Stribeck'
        for i = 1:robot.nbDOF
            Xhi_U(12*(i-1)+1:12*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;100];           % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
            Xhi_L(12*(i-1)+1:12*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;-100];                % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
        end
    case 'LuGre'
        for i = 1:robot.nbDOF
            Xhi_U(12*(i-1)+1:12*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;100];           % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
            Xhi_L(12*(i-1)+1:12*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;-100];                % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
        end
    otherwise
        for i = 1:robot.nbDOF
            Xhi_U(12*(i-1)+1:12*i,1)  = [10;10;10;10;10;10;1;1;1;10;10;100];           % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
            Xhi_L(12*(i-1)+1:12*i,1) = [0;0;0;0;0;0;-1;-1;-1;0;0;-100];                % [XXi, XYi, XZi, YYi, YZi, ZZi, Xi, Yi, Zi, Mi, Iai, tau_offi];
        end
end


if strcmp(robot.name, 'TX40') || strcmp(robot.name, 'RX90') % Take the coupling between joints 5 and 6 into account
    Xhi_U = [Xhi_U;10;10];
    Xhi_L = [Xhi_L;0;0];
end

% Real value of the parameter vector:
Beta_obj = feval(sprintf('Regressor_Beta_%s', robot.name),Xhi_obj);

% Montecarlo for theta bounds:

Beta_L = zeros(numel(Beta_obj),1);
Beta_U = zeros(numel(Beta_obj),1);

for i=1:1000
    Xhi = min(Xhi_U,max(Xhi_L,5*randn(size(Xhi_obj))));
    Beta = feval(sprintf('Regressor_Beta_%s', robot.name),Xhi);
    for j = 1:numel(Beta_obj)
        if Beta(j)<Beta_L(j)
            Beta_L(j) = Beta(j);
        end
        if Beta(j)>Beta_U(j)
            Beta_U(j) = Beta(j);
        end
    end
end

switch benchmarkSettings.initialParamType
    case 'RefSd'
        
        % Initial parameter estimates:
        
        rng('default') % Initialization of the pseudo-random numbers in order to have the same results each time the benchmark is started
        
        Initial_Beta = []; % Some special initial points
        Initial_Xhi = []; % Some special initial points
        % Random initial points around the objective:
        while size(Initial_Beta, 2)<numberOfInitialEstimates
            physicallyInconsistent = true;
            trialNumber = 0;
            while physicallyInconsistent
                physicallyInconsistent = false;
                trialNumber = trialNumber+1;
                % Generate a candidate:
                Xhi_0 =  min(Xhi_U,max(Xhi_L,Xhi_obj + benchmarkSettings.sdOfInitialEstimates*randn(size(Xhi_obj)).*Xhi_obj));
                
                % Check physical consistency of the candaidate:
                counter = 0;
                for i = 1:robot.nbDOF
                    P = pseudoInertiaMatrix_Xhi(Xhi_0(counter + 1: counter + robot.numericalParameters.numParam(i)));
                    counter = counter + robot.numericalParameters.numParam(i);
                    % Check positive-definiteness of the pseudo-inertia matrix P:
                    [~,FLAG] = chol(P);
                    if FLAG % Physical inconsistency detected
                        physicallyInconsistent = true;
                    end
                end
            end
            Initial_Xhi = [Initial_Xhi, Xhi_0];
            Initial_Beta = [Initial_Beta, feval(sprintf('Regressor_Beta_%s', robot.name),Xhi_0)];
        end
        disp('Generated set of physically consistent initial parameters !');
        Initial_Xhi
    case 'LS'
        error('This code section is still to be implemented !')
        %         benchmarkSettings.codeImplementation = 'classic';
        %         optionsIDIM_LS.debug = false;
        %         optionsIDIM_LS.solver ='backslash';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function
        %         optionsIDIM_LS.alg = 'OLS';                                         % [OLS]: Ordinary Least Squares, [WLS]: Weighted Least Squares, [TLS]: Total Least Squares, [RR]: Ridge Regression, [WRR]: Weighted Ridge Regression
        %         optionsIDIM_LS.filter = 'no';                                       % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        %         [Initial_Beta, ~, ~] = IDIM_LS_identification(robot, experimentDataStruct, 1, benchmarkSettings, zeros(robot.paramVectorSize,1), optionsIDIM_LS);
        %
    case 'LS_f'
        error('This code section is still to be implemented !')
        %         benchmarkSettings.codeImplementation = 'classic';
        %         optionsIDIM_LS.debug = false;
        %         optionsIDIM_LS.solver ='backslash';                                 % [backslash]: use the matlab optimized function (x=A\b), [pinv]: use the matlab pseudoinverse function
        %         optionsIDIM_LS.alg = 'OLS';                                         % [OLS]: Ordinary Least Squares, [WLS]: Weighted Least Squares, [TLS]: Total Least Squares, [RR]: Ridge Regression, [WRR]: Weighted Ridge Regression
        %         optionsIDIM_LS.filter = 'butterworth';                              % [no]: no filter, [lowpass]: low pass filter, [butterworth]: zero-shift butterworth filter
        %         [Initial_Beta, ~, ~] = IDIM_LS_identification(robot, experimentDataStruct, 1, benchmarkSettings, zeros(robot.paramVectorSize,1), optionsIDIM_LS);
        %
    otherwise
        error('Unknown option for initial parmeter estimate generation.');
end

