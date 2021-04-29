function [flag] = postProcessing(robot, benchmarkSettings, experimentDataStruct)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot, Gordon Cheng
%%

% We start by loading the desired experiment data in proper data structures: 
initL2Error = zeros(1, benchmarkSettings.numberOfInitialEstimates);
finalL2Error = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg);
finalTime = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg);
iterConv = ones(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg);
modelRecalcConv = ones(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg);
paramError = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg, robot.paramVectorSize);
paramValue = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg, robot.paramVectorSize);
paramInitValue = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, robot.paramVectorSize);
decimRate = zeros(benchmarkSettings.nbAlg, 1);
averageTime = zeros(1, benchmarkSettings.nbAlg);
stdTime = zeros(1, benchmarkSettings.nbAlg);
averageIter = zeros(1, benchmarkSettings.nbAlg);
stdIter = zeros(1, benchmarkSettings.nbAlg);
averageError = zeros(1, benchmarkSettings.nbAlg);
convergence  = zeros(benchmarkSettings.numberOfInitialEstimates, benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.nbAlg);
averageParamError = zeros(robot.paramVectorSize, benchmarkSettings.nbAlg);
stdParamError = zeros(robot.paramVectorSize, benchmarkSettings.nbAlg);
stdError = zeros(1, benchmarkSettings.nbAlg);

resFlag = false;
for index=1:benchmarkSettings.nbAlg
    % Loading data:
    if benchmarkSettings.identificationMethods.showResults(index)==1 % If the method is selected
        resFlag = true;
        resultFile = sprintf('%s_%s/decim_%d/results_%s_%s.mat', robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate, benchmarkSettings.identificationMethods.algName{index}, robot.name)
        resultVariable = sprintf('results_%s',benchmarkSettings.identificationMethods.algName{index});
        if exist(fullfile(resultFile),'file') == 2
            resultDataStruct{index} = load(resultFile);
            Beta_obj = eval(sprintf('resultDataStruct{index}.%s.benchmarkSettings.Beta_obj',resultVariable));
            for i=1:benchmarkSettings.numberOfInitialEstimates
                
                initL2Error(i) = norm(eval(sprintf('resultDataStruct{index}.%s.benchmarkSettings.Initial_Beta(:, i)',resultVariable))-Beta_obj);
                for j=1:benchmarkSettings.numberOfExperimentsPerInitialPoint
                    theta = reshape(eval(sprintf('resultDataStruct{index}.%s.Betas(i, j, :)',resultVariable)), robot.paramVectorSize, 1);
                    finalTime(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.times(i, j)',resultVariable));
                    
                    error = norm(Beta_obj-theta);
                    if (error>50*norm(Beta_obj-benchmarkSettings.Initial_Beta(:,i))) && ~strcmp(benchmarkSettings.identificationMethods.algName{index}, 'LS') % If final error is greater than the initial error
                        convergence(i,j,index) = 0;
                        finalL2Error(i, j, index) = 0;
                        paramError(i, j, index, :) = 0*(Beta_obj-theta);
                        paramValue(i, j, index, :) = theta;
                    else
                        convergence(i,j,index) = 1;
                        finalL2Error(i, j, index) = error;
                        paramError(i, j, index, :) = Beta_obj-theta;
                        paramValue(i, j, index, :) = theta;
                    end
                    paramInitValue(i, j, :) = eval(sprintf('resultDataStruct{index}.%s.benchmarkSettings.Initial_Beta(:, i)',resultVariable));
                    decimRate(index) = eval(sprintf('resultDataStruct{index}.%s.benchmarkSettings.decimRate',resultVariable));
                    if strcmp(benchmarkSettings.identificationMethods.algName{index}, 'DIDIM') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'IV')
                        iterConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j)',resultVariable));
                        modelRecalcConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j)',resultVariable));
                    end
                    if strcmp(benchmarkSettings.identificationMethods.algName{index}, 'PC_DIDIM') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'PC_IV')
                        iterConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j)',resultVariable));
                        modelRecalcConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j)',resultVariable));
                    end
                    if strcmp(benchmarkSettings.identificationMethods.algName{index}, 'CLOE') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'CLIE')
                        modelRecalcConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j).funcCount',resultVariable));
                        iterConv(i, j, index) = eval(sprintf('resultDataStruct{index}.%s.iteration(i, j).iterations',resultVariable));
                    end
                end
                for k=1:robot.paramVectorSize
                    stdParamError(k,index) =  stdParamError(k,index)+std(paramError(i, find(convergence(i,:,index)), index,k));
                end
            end
            
            
            
            %             if strcmp(benchmarkSettings.identificationMethods.algName{index}, 'EKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'UKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'SRUKF')|| strcmp(benchmarkSettings.identificationMethods.algName{index}, 'CDKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'SRCDKF')
            %                 paramEvolution(:,:,:,index) = eval(sprintf('resultDataStruct{index}.%s.Betas_iteration',resultVariable));
            %             end
            convIndex = find(convergence(:,:,index)); % index of the points which converged.
            fE = reshape(finalL2Error(:,:, index), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
            fT(:,index) = reshape(finalTime(:,:, index), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
            fI(:,index) = reshape(iterConv(:,:, index), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
            fR(:,index) = reshape(modelRecalcConv(:,:, index), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
            for i = 1:robot.paramVectorSize
                fEP(:,i, index) = reshape(paramError(:,:, index,i), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
                fP(:,i, index) = reshape(paramValue(:,:, index,i), benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
            end
            
            averageIter(index) = mean(fI(convIndex));
            averageTime(index) = mean(fT(convIndex, index));
            averageError(index) = mean(fE(convIndex));
            stdError(index) = std(reshape(fE(convIndex), numel(fE(convIndex)), 1));
            stdTime(index) = std(reshape(finalTime(:, :, index), numel(finalTime(:, :, index)), 1));
            stdIter(index) = std(reshape(iterConv(:, :, index), numel(iterConv(:, :, index)), 1));
            
            averageParamError(:,index) = mean(fEP(convIndex,:, index),1);
            averageParam(:,index) = mean(fP(convIndex,:, index),1);
            %             errorFunction(robot, benchmarkSettings, experimentDataStruct, averageParam(:,index), index, i)
            for i = 1:robot.paramVectorSize
                ss = reshape(eval(sprintf('resultDataStruct{index}.%s.Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint, i)',resultVariable)),benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
                stdParamError(i,index) =  std(ss(convIndex));
            end
            %             if strcmp(benchmarkSettings.identificationMethods.algName{index}, 'EKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'UKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'SRUKF')|| strcmp(benchmarkSettings.identificationMethods.algName{index}, 'CDKF') || strcmp(benchmarkSettings.identificationMethods.algName{index}, 'SRCDKF')
            %                 getCovarianceVideo(eval(sprintf('resultDataStruct{index}.%s.Covariance_iteration',resultVariable)), benchmarkSettings.identificationMethods.algName{index}, 100, false)
            %             end
        else
            resultDataStruct{index} = 0;
        end
    end
end




%% Run result analysis tools:
if resFlag == true
    methods = categorical(benchmarkSettings.identificationMethods.algName);
    methods = reordercats(methods,benchmarkSettings.identificationMethods.algName);
    
    
    % generation of a tabular with average and std parameter errors for latex
% generateLaTeXparamTab(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)
% generateLaTeXparamTabPercent(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)
% generateLaTeXparamTabPercent2(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)
% % % % % generateLaTeXFOMTab(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, stdParamError, averageTime, averageIter)
%generateLaTeXFOMTabTot(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, stdParamError, averageTime, averageIter)
% generateReport(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, averageParamError, stdParamError, averageTime, averageIter);
    %     plotRecalculatedClosedLoopDDM(robot, benchmarkSettings, resultDataStruct, experimentDataStruct);
    %     plotRawResults(benchmarkSettings, fEP, fT);
    %     plotRawResultsHistogram(benchmarkSettings, fEP, fT);
    %     plotRawResultsParamwise(robot, benchmarkSettings, fEP, fT)
    %     plot3DMeanResults(robot, benchmarkSettings, averageParamError);
    %         plotSummary(benchmarkSettings, averageTime, averageIter, averageError, stdError, methods);
%             plotConvergenceStatus(robot, benchmarkSettings, convergence, decimRate);
    %         plotParamwiseHoriz(robot, benchmarkSettings, averageParamError, stdParamError);
%             plotParamwiseVert(robot, benchmarkSettings, averageParamError, stdParamError);

%     plotRecalculatedTorquesFull(robot, benchmarkSettings, experimentDataStruct, averageParam, false);
    
    
% % % % % % % %         plotParamwiseVertNew(robot, benchmarkSettings, averageParamError, stdParamError, decimRate);
    %
    %     for i=1:robot.nbDOF
    %         plotRecalculatedTorques(robot, benchmarkSettings, Q, Qp, Qpp, Tau, t, averageParam, i);
    %     end
    %
%             plotErrorHistogram(robot, benchmarkSettings, experimentDataStruct, decimRate, averageParam, stdParamError);
    %
%         checkStatHypotheses(robot, benchmarkSettings, experimentDataStruct, decimRate, averageParam, stdParamError);
    
plotParamwiseMatrix(robot, benchmarkSettings, averageParamError, stdParamError, decimRate);
    
    %
    %     %% Parameter Convergence Kalman
    %     cc=parula(robot.paramVectorSize);
    %     index = find(benchmarkSettings.identificationMethods.showResults);
    %     counter = 0;
    %     indexK = [];
    %     for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    %         if strcmp(benchmarkSettings.identificationMethods.algName{index(i)}, 'EKF') || strcmp(benchmarkSettings.identificationMethods.algName{index(i)}, 'UKF') || strcmp(benchmarkSettings.identificationMethods.algName{index(i)}, 'SRUKF')|| strcmp(benchmarkSettings.identificationMethods.algName{index(i)}, 'CDKF') || strcmp(benchmarkSettings.identificationMethods.algName{index(i)}, 'SRCDKF')
    %             counter = counter+1;
    %             indexK(counter) = index(i);
    %         end
    %         if counter == 1
    %             figure('Name','Parameter Convergence')
    %         end
    %         for i=1:counter
    %             subplot(counter,1,i)
    %             for j=1:robot.paramVectorSize
    %                 plot(paramEvolution(j,:,1200,indexK(i))','LineWidth',2 ,'color',cc(j,:))
    %                 hold on
    %                 plot(repmat(benchmarkSettings.Beta_obj(j),1,size(paramEvolution,2))','--','LineWidth',2,'color',cc(j,:))
    %             end
    %             title(sprintf('\\textbf{Convergence} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{indexK(i)}), 'Interpreter', 'LaTeX')
    %         end
    %     end
    %
end
end


%%%%%%%%%%%%%%%%%%%%%%%
%% generateReport:
%%%%%%%%%%%%%%%%%%%%%%%


% mkdir(sprintf('%s/%s_%s/decim%d/paramError',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i))))

function []=generateReport(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, averageParamError, stdParamError, averageTime, averageIter)

if strcmp(robot.name, 'TX40_uncoupled')
    name = 'TX40';
else
    name = robot.name;
end


% Import the DOM and Report API packages so you do not have to use long class names.
import mlreportgen.report.*;
import mlreportgen.dom.*;

imgStyle = {ScaleToFit(true)};

% Create a container to hold the report content.
% To create a Word report, change the output type from "pdf" to "docx".
% rpt = mlreportgen.report.Report("PortraitAndLandscapeReport", "pdf");
rpt = Report(sprintf('%s_%s_decim_%d',robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate),"pdf");
open(rpt);

% Specify the page orientation, height, and width.
pageLayoutObj.PageSize.Orientation = "portrait";
pageLayoutObj.PageSize.Height = "297mm";
pageLayoutObj.PageSize.Width = "210mm";

% Specify the page margins.
pageLayoutObj.PageMargins.Top = "0cm";
pageLayoutObj.PageMargins.Bottom = "0cm";
pageLayoutObj.PageMargins.Left = "0cm";
pageLayoutObj.PageMargins.Right = "0cm";

pageLayoutObj.PageMargins.Header = "0cm";
pageLayoutObj.PageMargins.Footer = "0cm";

% Add the page layout object to the report.
add(rpt,pageLayoutObj);



% Genrate a table in LaTeX format, containing Figures of Merit (FOM) for
% the set of considered identification methods
N_fom = 5;
options.filter = 'butterworth';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];

for expNb = 1:1%benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    %     [~, ~, ~, ~, ~, ~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    [~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~, ~,~,~,~,~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end

% Generation of the tabularcontatinning the average error and standard deviation of the error.
index = find(benchmarkSettings.identificationMethods.showResults);
% file = fopen(sprintf('%s/%s_%s/decim%d/LaTeX/FOMtab.tex',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1))), 'w');


%
% fprintf(file, "\\cellcolor{green!45}\\textbf{%s, MCS, decim %d} & \\cellcolor{gray!25}$E(d_{\\boldsymbol{q}}), \\sigma_{\\boldsymbol{q}}$ & \\cellcolor{gray!25}$E(d_{\\boldsymbol{\\tau}}), \\sigma_{\\boldsymbol{\\tau}}$& \\cellcolor{gray!25}$E(d_{t}), \\sigma_{t}$  &\\cellcolor{gray!25} $E(d_{N_{it}}), \\sigma_{N_{it}}$ & \\cellcolor{gray!25}$E(d_{N_{sim}}), \\sigma_{N_{sim}}$ \\\\ \n",name ,benchmarkSettings.decimRate(index(1)));
% fprintf(file, "\\midrule\n");

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    nameAlgo = getNameId(benchmarkSettings,index,i)
    % Compute relative torque error:
    [~,nbSamples]=size(Tau_decim);
    W = observationMatrixOrdered(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
    %     Y_tau_ref = W*benchmarkSettings.Beta_obj;          % Recomputed torque with real parameter values
    Y_tau_ref = torqueVectorOrdered(Tau_decim);
    Y_tau_recomputed = W*averageParam(:,index(i));
    
    Y_error_tau = Y_tau_ref - Y_tau_recomputed;   % Recomputed torque with mean value of the estimated parameters
    
    Y_error_tau_joint=zeros(nbSamples,robot.nbDOF);
    Y_tau_joint=zeros(nbSamples,robot.nbDOF);
    Y_tau_recomputed_joint=zeros(nbSamples,robot.nbDOF);
    for jointNb = 1:robot.nbDOF
        Y_error_tau_joint(:,jointNb) = Y_error_tau((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_joint(:,jointNb) = Y_tau_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_recomputed_joint(:,jointNb) = Y_tau_recomputed((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
    
    Y_error_tau_med = sqrt(sum(Y_error_tau_joint.^2,2));
    Y_tau_med = sqrt(sum(Y_tau_joint.^2,2));
    d_tau(i,1) = 100*mean(Y_error_tau_med./Y_tau_med);
    sigma_d_tau(i,1) = 100*std(Y_error_tau_med./Y_tau_med);
    
    % Compute relative joint position error:
    t_control = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_control, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    [~, stateVector, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, averageParam(:,index(i)), robot.name, robot.numericalParameters.Geometry, ...
        robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
        robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
        robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
        robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    Q_sim = stateVector(robot.nbDOF+1:2*robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    Qp_sim = stateVector(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    
    t_sample = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples);  % Control epochs
    stateVectorDesired = benchmarkSettings.trajectoryData.getTrajectoryData(t_sample, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    Q_sim_decim = [];
    Qp_sim_decim = [];
    Qd_decim = [];
    Qpd_decim = [];

    for jointNb = 1:robot.nbDOF
        Q_sim_decim(jointNb,:) = decimate(Q_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
        Qp_sim_decim(jointNb,:) = decimate(Qp_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
        Qd_decim(jointNb,:) = decimate(stateVectorDesired(2*robot.nbDOF + jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
        Qpd_decim(jointNb,:) = decimate(stateVectorDesired(robot.nbDOF + jointNb,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
    end
    
    
    
    Y_error_Q = torqueVectorOrdered(Q_decim-Q_sim_decim);
    Y_Q_ref = torqueVectorOrdered(Q_decim);
    for jointNb = 1:robot.nbDOF
        Y_error_Q_joint(:,jointNb) = Y_error_Q((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_Q_joint(:,jointNb) = Y_Q_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
    Y_error_Q_med = sqrt(sum(Y_error_Q_joint.^2,2));
    Y_Q_med = sqrt(sum(Y_Q_joint.^2,2));
    d_q(i,1) = 100*mean(Y_error_Q_med./Y_Q_med);
    sigma_d_q(i,1) = 100*std(Y_error_Q_med./Y_Q_med);
    
    % Compute average parameter error:
    paramRef = repmat(benchmarkSettings.Beta_obj.',benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
    relativeAverageParamErrorsMC = (2/robot.paramVectorSize)*sum(abs(fEP(:,:,index(i)))./(abs(paramRef)+abs(fP(:,:,index(i)))),2); % series of d_\beta for each MC experiment
    d_beta(i,1) = 100*mean(relativeAverageParamErrorsMC);
    sigma_d_beta(i,1) = 100*std(relativeAverageParamErrorsMC);
    
    % Get the mean time required to compute one parameter estimate:
    d_t(i,1) = mean(fT(:,index(i)));
    sigma_d_t(i,1) = 100*std(fT(:,index(i)))/mean(fT(:,index(i)));
    
    % If applicable, get the number of iterations required to compute one
    % parameter estimate:
    d_i(i,1) = mean(fI(:,index(i)));
    sigma_d_i(i,1) = 100*std(fI(:,index(i)))/mean(fI(:,index(i)));
    
    % If applicable, get the number of model recalculations required to compute one
    % parameter estimate:
    d_r(i,1) = mean(fR(:,index(i)));
    if mean(fR(:,index(i)))~=0
        sigma_d_r(i,1) = 100*std(fR(:,index(i)))/mean(fR(:,index(i)));
    else
        sigma_d_r(i,1) = -1;
    end
    
    %     mean_fom(1,i) = d_beta(i);
    mean_fom(1,i) = d_q(i);
    mean_fom(2,i) = d_tau(i);
    mean_fom(3,i) = d_t(i);
    mean_fom(4,i) = d_i(i);
    mean_fom(5,i) = d_r(i);
    
    %     unit{1} = '\%';
    unit{1} = '\%';
    unit{2} = '\%';
    unit{3} = 's';
    unit{4} = '';
    unit{5} = '';
    
    %     sigma_fom(1,i) = sigma_d_beta(i);
    sigma_fom(1,i) = sigma_d_q(i);
    sigma_fom(2,i) = sigma_d_tau(i);
    sigma_fom(3,i) = sigma_d_t(i);
    sigma_fom(4,i) = sigma_d_i(i);
    sigma_fom(5,i) = sigma_d_r(i);
    
    Algorithm{i,1} = getNameId(benchmarkSettings,index,i);
    
    
    %     fprintf(file, "\\textbf{%s}", nameAlg);
    %
    %     for j = 1:N_fom
    %         if j == N_fom
    %             if ~strcmp(nameAlg,'CLIE') && ~strcmp(nameAlg,'CLOE') ~= ~strcmp(nameAlg,'DIDIM') ~= ~strcmp(nameAlg,'IDIM-IV') ~= ~strcmp(nameAlg,'PC-IDIM-IV') ~= ~strcmp(nameAlg,'PC-DIDIM')
    %                 fprintf(file, " & N.A.");
    %             else
    %                 fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
    %             end
    %         elseif j == 1
    %             fprintf(file, " & %5.3f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
    %         else
    %             fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
    %         end
    %
    %     end
    
    %     fprintf(file, "\\\\  \n");
end

% % mltable = table(sigma_d_q,sigma_d_tau,sigma_d_t, sigma_d_i, sigma_d_r);
% mltable = table(Algorithm, d_q, d_tau, d_t, d_i, d_r);
% 
% mltableObj = MATLABTable(mltable);
% tbodyObj = mltableObj.Body;
% tbodyObj.TableEntriesStyle = {Color('blue')};
% tbODYObj.TableEntriesHAlign = 'center';
% 
% add(chapter,mltableObj)


% fclose(file);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

options.filter = 'butterworth';
set(0, 'DefaultFigureRenderer', 'painters');

for i=1:sum(benchmarkSettings.identificationMethods.showResults)

    chapter = Chapter("Title", sprintf('%s, %s, %s, decimation rate %d',name, benchmarkSettings.identificationMethods.algName{index(i)}, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate));
    section = Section("Title", "Parameter Errors");
    
    barh(1:robot.paramVectorSize, averageParamError(:,index(i)), 'FaceColor', [0 .39 .74], 'EdgeColor', [0 .59 .94], 'FaceAlpha',0.75);
    hold on
    errorbar(averageParamError(:,index(i)),1:robot.paramVectorSize, stdParamError(:,index(i)),'horizontal','r.','LineWidth',0.5)
    hold off
    grid on
    grid minor
    ylabel('Index');
    xlabel('Parameter Error');
    xlim([-0.5 0.5])
    x=0.5;
    nameAlgo = getNameId(benchmarkSettings,index,i);
    text(0.4*(-ones(1,robot.paramVectorSize)).^(0:robot.paramVectorSize-1), linspace(1-x,robot.paramVectorSize-x,robot.paramVectorSize),strcat('err(',strcat(num2str(linspace(1,robot.paramVectorSize,robot.paramVectorSize).','%d\n'),strcat(')=',strcat(strcat(num2str(averageParamError(:,index(i)),'%5.3f\n'),'±'),num2str(stdParamError(:,index(i)),'%5.3f\n'))))),'vert','bottom','horiz','center','Rotation',0,'FontSize', 8);
    title(sprintf('Parameter Identification %s %s', name, nameAlgo))
    legend({'mean', 'std'},'Location','southoutside','Orientation','horizontal')
    set(gcf,'OuterPosition',[0 0 960 540]);
    pbaspect([2 1 1])
    figMeanErrorVar=Figure();
    figMeanErrorVarImg = Image(getSnapshotImage(figMeanErrorVar, rpt));
    figMeanErrorVarImg.Style = imgStyle;
    
    % Add the table to the chapter
    add(section,figMeanErrorVarImg)
    add(chapter, section)
    
    
    Tau_ndecim = [];
    Qpp_ndecim = [];
    Qp_ndecim = [];
    Q_ndecim = [];
    
    for expNb = 1:1%benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
        [t_ndecim, Tau_ndecim_exp, Qpp_ndecim_exp, Qp_ndecim_exp, Q_ndecim_exp, ~, ~, ~, ~, ~, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
        Tau_ndecim = [Tau_ndecim Tau_ndecim_exp];
        Qpp_ndecim = [Qpp_ndecim Qpp_ndecim_exp];
        Qp_ndecim = [Qp_ndecim Qp_ndecim_exp];
        Q_ndecim = [Q_ndecim Q_ndecim_exp];
    end
    
    % Compute the observation matrix:
    W = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_ndecim, Qp_ndecim, Qpp_ndecim);
    %     num = [   1,    0.0781,    0.2344;... % Crimson Red
    %      0,    0.9000,         0.1;... % Real Green
    %          0.1172,    0.5625,    1.0000;... % Deep Sky Blue
    %          1.0000,         0,    1.0000;... % Magenta
    %          0,    1.0000,    1.0000;... % Cyan
    %          0.8000,    0.2695,         0;... % Orange
    %          0,    1.0000,    0.25000;... % Light Green
    %          0,         0,    0.5000;... % Navy Blue
    %          0.5000,         0,    0.5000;... % Purple
    %          0.5391,    0.1680,    0.8828;... % BlueViolet
    %          0,         0,    1.0000;... % Blue
    %          0.5000,    0.5000,    0.5000];   % Gray
    %
    %      set(groot,'defaultAxesColorOrder',num)
    section = Section("Title", "Recomputed torques");
    for jointNb = 1:robot.nbDOF
        referenceTau = W*benchmarkSettings.Beta_obj;
        graph = plot(t_ndecim, Tau_ndecim(jointNb,:),'Color',[0,         0,    0.5000],'LineWidth',1);
        graph.Color(4)=0.1;
        hold on
        plot(t_ndecim,referenceTau(jointNb:robot.nbDOF:end),'Color',[0,    0.9000,         0.1], 'LineWidth',2);
        hold on
        recomputedTau = W*averageParam(:,index(i));
        plot(t_ndecim, recomputedTau(jointNb:robot.nbDOF:end),'Color',[1,    0.0781,    0.2344],'LineWidth',1);
        hold off
        xlabel('Time [s]');
        ylabel(sprintf('Torque [N.m]'));
        title(sprintf('Recomputed torque %s %s, joint %d',name, nameAlgo, jointNb))
        grid on
        grid minor
        if benchmarkSettings.experimentOnTrueRobot == true
            legend({'Real','CAD Parameters',sprintf('%s',nameAlgo)},'Location','southoutside','Orientation','horizontal')
        else
            legend({'Real','Reference',sprintf('%s',nameAlgo)},'Location','southoutside','Orientation','horizontal')
        end
        set(gcf,'OuterPosition',[0 0 960 540]);
        
        figTorqueError=Figure();
        figTorqueErrorImg = Image(getSnapshotImage(figTorqueError, rpt));
        figTorqueErrorImg.Style = imgStyle;
        % Add the figure to the chapter
        add(section,figTorqueErrorImg)
        
    end
    add(chapter,section)
%     section = Section("Title", "Recomputed velocities");
%     for jointNb = 1:robot.nbDOF
%         plot(t_decim, Qpd_decim(jointNb,:),'--','Color',[0.1172, 0.5625, 1.0000],'LineWidth',2);
%         hold on
%         plot(t_decim, Qp_decim(jointNb,:),'Color',[0, 0,    0.5000],'LineWidth',1.5);
%         hold on
%         plot(t_decim,Qp_sim_decim(jointNb,:),'Color',[0,    0.9000,         0.1], 'LineWidth',1);
%         hold on
%         plot(t_decim,100*(Qp_decim(jointNb,:)-Qp_sim_decim(jointNb,:)),'Color',[1,    0.0781,    0.2344], 'LineWidth',1);
%         hold off
%         xlabel('Time [s]');
%         ylabel(sprintf('Joint velocity [rad/s]'));
%         title(sprintf('Joint velocity %s joint %d in closed loop with the %s estimates',name, jointNb, nameAlgo))
%         grid on
%         grid minor
%         legend({'Desired','Real',sprintf('Recomputed %s',nameAlgo), 'Error: 100 x (Real - Recomputed)'},'Location','southoutside','Orientation','horizontal')
%         set(gcf,'OuterPosition',[0 0 960 540]);
%         
%         figQpError=Figure();
%         figQpErrorImg = Image(getSnapshotImage(figQpError, rpt));
%         figQpErrorImg.Style = imgStyle;
%         % Add the figure to the chapter
%         add(section,figQpErrorImg)
%     end
%     add(chapter,section)
%     section = Section("Title", "Recomputed positions");
%     for jointNb = 1:robot.nbDOF
%         plot(t_decim, Qd_decim(jointNb,:),'--','Color',[0.1172, 0.5625, 1.0000],'LineWidth',2);
%         hold on
%         plot(t_decim, Q_decim(jointNb,:),'Color',[0, 0,    0.5000],'LineWidth',1.5);
%         hold on
%         plot(t_decim,Q_sim_decim(jointNb,:),'Color',[0,    0.9000,         0.1], 'LineWidth',1);
%         hold on
%         plot(t_decim,100*(Q_decim(jointNb,:)-Q_sim_decim(jointNb,:)),'Color',[1,    0.0781,    0.2344], 'LineWidth',1);
%         hold off
%         xlabel('Time [s]');
%         ylabel(sprintf('Joint position [rad]'));
%         title(sprintf('Joint position %s joint %d in closed loop with the %s estimates',name, jointNb, nameAlgo))
%         grid on
%         grid minor
%         legend({'Desired','Real',sprintf('Recomputed %s',nameAlgo), 'Error: 100 x (Real - Recomputed)'},'Location','southoutside','Orientation','horizontal')
%         set(gcf,'OuterPosition',[0 0 960 540]);
%         
%         figQpError=Figure();
%         figQpErrorImg = Image(getSnapshotImage(figQpError, rpt));
%         figQpErrorImg.Style = imgStyle;
%         % Add the figure to the chapter
%         add(section,figQpErrorImg)
%     end
%     add(chapter,section)
   
    
    % Add the chapter to the report
    add(rpt, chapter);
    delete(gcf);
end

% Generate and display the report
close(rpt);
rptview(rpt);

end

%%%%%%%%%%%%%%%%%%%%%%%
%% generateLaTeXFOMTab:
%%%%%%%%%%%%%%%%%%%%%%%

function []=generateLaTeXFOMTab(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, stdParamError, averageTime, averageIter)

% Genrate a table in LaTeX format, containing Figures of Merit (FOM) for
% the set of considered identification methods
N_fom = 5;
options.filter = 'butterworth';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];

for expNb = 1:1%benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    %     [~, ~, ~, ~, ~, ~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    [~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~, ~,~,~,~,~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end

% Generation of the tabularcontatinning the average error and standard deviation of the error.
index = find(benchmarkSettings.identificationMethods.showResults);
file = fopen(sprintf('%s/%s_%s/decim%d/LaTeX/FOMtab.tex',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1))), 'w');


if strcmp(robot.name, "TX40_uncoupled")
    name = "TX40";
else
    name = robot.name;
end

fprintf(file, "\\cellcolor{green!45}\\textbf{%s, MCS, decim %d} & \\cellcolor{gray!25}$E(d_{\\boldsymbol{q}}), \\sigma_{\\boldsymbol{q}}$ & \\cellcolor{gray!25}$E(d_{\\boldsymbol{\\tau}}), \\sigma_{\\boldsymbol{\\tau}}$& \\cellcolor{gray!25}$E(d_{t}), \\sigma_{t}$  &\\cellcolor{gray!25} $E(d_{N_{it}}), \\sigma_{N_{it}}$ & \\cellcolor{gray!25}$E(d_{N_{sim}}), \\sigma_{N_{sim}}$ \\\\ \n",name ,benchmarkSettings.decimRate(index(1)));
fprintf(file, "\\midrule\n");
            
for i=1:sum(benchmarkSettings.identificationMethods.showResults)

    % Compute relative torque error:
    [~,nbSamples]=size(Tau_decim);
    W = observationMatrixOrdered(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
%     Y_tau_ref = W*benchmarkSettings.Beta_obj;          % Recomputed torque with real parameter values
    Y_tau_ref = torqueVectorOrdered(Tau_decim);
    Y_tau_recomputed = W*averageParam(:,index(i));  
    
    Y_error_tau = Y_tau_ref - Y_tau_recomputed;   % Recomputed torque with mean value of the estimated parameters
      
    Y_error_tau_joint=zeros(nbSamples,robot.nbDOF);    
    Y_tau_joint=zeros(nbSamples,robot.nbDOF);
    Y_tau_recomputed_joint=zeros(nbSamples,robot.nbDOF);
    for jointNb = 1:robot.nbDOF
        Y_error_tau_joint(:,jointNb) = Y_error_tau((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_joint(:,jointNb) = Y_tau_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_recomputed_joint(:,jointNb) = Y_tau_recomputed((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
    figure()
    plot(Y_tau_joint(:,1),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,1),'b')
    
    figure()
    plot(Y_tau_joint(:,2),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,2),'b')
    
    figure()
    plot(Y_tau_joint(:,3),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,3),'b')
    
    figure()
    plot(Y_tau_joint(:,4),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,4),'b')
    
    figure()
    plot(Y_tau_joint(:,5),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,5),'b')
    
    figure()
    plot(Y_tau_joint(:,6),'r')
    hold on
    plot(Y_tau_recomputed_joint(:,6),'b')
    
    pause
    
    Y_error_tau_med = sqrt(sum(Y_error_tau_joint.^2,2));
    Y_tau_med = sqrt(sum(Y_tau_joint.^2,2));
    d_tau(i) = 100*mean(Y_error_tau_med./Y_tau_med);
    sigma_d_tau(i) = 100*std(Y_error_tau_med./Y_tau_med);
    
    % Compute relative joint position error:
    t_control = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_control, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    [~, stateVector, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, averageParam(:,index(i)), robot.name, robot.numericalParameters.Geometry, ...
        robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
        robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
        robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
        robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    Q_sim = stateVector(robot.nbDOF+1:2*robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    Qp_sim = stateVector(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    
    Q_sim_decim = [];
    Qp_sim_decim = [];
    for jointNb = 1:robot.nbDOF
        Q_sim_decim(jointNb,:) = decimate(Q_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
        Qp_sim_decim(jointNb,:) = decimate(Qp_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
    end
    
%     figure()
%     plot(Q_sim_decim.','b')
%     hold on
%     plot(Q_decim.','r')
%     
%     figure()
%     plot(Qp_sim_decim.','b')
%     hold on
%     plot(Qp_decim.','r')
    
    Y_error_Q = torqueVectorOrdered(Q_decim-Q_sim_decim);
    Y_Q_ref = torqueVectorOrdered(Q_decim);
    for jointNb = 1:robot.nbDOF
        Y_error_Q_joint(:,jointNb) = Y_error_Q((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_Q_joint(:,jointNb) = Y_Q_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
    Y_error_Q_med = sqrt(sum(Y_error_Q_joint.^2,2));
    Y_Q_med = sqrt(sum(Y_Q_joint.^2,2));
    d_q(i) = 100*mean(Y_error_Q_med./Y_Q_med);
    sigma_d_q(i) = 100*std(Y_error_Q_med./Y_Q_med);
    
    % Compute average parameter error:
    paramRef = repmat(benchmarkSettings.Beta_obj.',benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
    relativeAverageParamErrorsMC = (2/robot.paramVectorSize)*sum(abs(fEP(:,:,index(i)))./(abs(paramRef)+abs(fP(:,:,index(i)))),2); % series of d_\beta for each MC experiment
    d_beta(i) = 100*mean(relativeAverageParamErrorsMC);
    sigma_d_beta(i) = 100*std(relativeAverageParamErrorsMC);
    
    % Get the mean time required to compute one parameter estimate:
    d_t(i) = mean(fT(:,index(i)));
    sigma_d_t(i) = 100*std(fT(:,index(i)))/mean(fT(:,index(i)));
    
    % If applicable, get the number of iterations required to compute one
    % parameter estimate:
    d_i(i) = mean(fI(:,index(i)));
    sigma_d_i(i) = 100*std(fI(:,index(i)))/mean(fI(:,index(i)));
    
    % If applicable, get the number of model recalculations required to compute one
    % parameter estimate:
    d_r(i) = mean(fR(:,index(i)));
    if mean(fR(:,index(i)))~=0
        sigma_d_r(i) = 100*std(fR(:,index(i)))/mean(fR(:,index(i)));
    else
        sigma_d_r(i) = -1;
    end
    
%     mean_fom(1,i) = d_beta(i);
    mean_fom(1,i) = d_q(i);
    mean_fom(2,i) = d_tau(i);
    mean_fom(3,i) = d_t(i);
    mean_fom(4,i) = d_i(i);
    mean_fom(5,i) = d_r(i);
    
%     unit{1} = '\%';
    unit{1} = '\%';
    unit{2} = '\%';
    unit{3} = 's';
    unit{4} = '';
    unit{5} = '';
    
%     sigma_fom(1,i) = sigma_d_beta(i);
    sigma_fom(1,i) = sigma_d_q(i);
    sigma_fom(2,i) = sigma_d_tau(i);
    sigma_fom(3,i) = sigma_d_t(i);
    sigma_fom(4,i) = sigma_d_i(i);
    sigma_fom(5,i) = sigma_d_r(i);
    
    nameAlg = getNameId(benchmarkSettings,index,i);
    
    
    fprintf(file, "\\textbf{%s}", nameAlg);
    
    for j = 1:N_fom
        if j == N_fom
            if ~strcmp(nameAlg,'CLIE') && ~strcmp(nameAlg,'CLOE') ~= ~strcmp(nameAlg,'DIDIM') ~= ~strcmp(nameAlg,'IDIM-IV') ~= ~strcmp(nameAlg,'PC-IDIM-IV') ~= ~strcmp(nameAlg,'PC-DIDIM')
                fprintf(file, " & N.A.");
            else
                fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
            end
        elseif j == 1
            fprintf(file, " & %5.3f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
        else
            fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
        end
        
    end
    
    fprintf(file, "\\\\  \n");
end

fclose(file);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
% if isunix
%     unix(sprintf("sed -i 's/E/\\cdot10^{/g' FOMtab_%s.txt", robot.name));
% end

end


function []=generateLaTeXFOMTabTot(robot,  benchmarkSettings, experimentDataStruct, fP, fEP, fT, fI, fR, averageParam, stdParamError, averageTime, averageIter)

if strcmp(robot.name, "TX40_uncoupled")
    name = "TX40";
    nameTot = "Staubli TX40";
elseif strcmp(robot.name, "TX40")
    name = robot.name;
    nameTot = "Staubli TX40";
elseif strcmp(robot.name, "RV2SQ")
    name = robot.name;
    nameTot = "Mitsubishi RV2SQ";
else
    name = robot.name;
end

% Genrate a table in LaTeX format, containing Figures of Merit (FOM) for
% the set of considered identification methods
N_fom = 5;
options.filter = 'butterworth';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];

for expNb = 1:1%benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    %     [~, ~, ~, ~, ~, ~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    [~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~, ~,~,~,~,~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end
% Generation of the tabularcontatinning the average error and standard deviation of the error.
index = find(benchmarkSettings.identificationMethods.showResults);
folderPath = sprintf('Benchmark/Robot_Identification_Results/LaTeX/%s_%s/decim%d/LaTeX', robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1)))
templateFolderPath = 'Benchmark/Robot_Identification_Results/LaTeX/Template';
filePath = sprintf('Benchmark/Robot_Identification_Results/LaTeX/%s_%s/decim%d/LaTeX/FOMtab.tex', robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1)));
robotHasSymbolicModel = exist(folderPath, 'dir');

if robotHasSymbolicModel ==  0 % If the resut folder does not exist
    disp('The LaTeX report folder was not detected: creating it...');
    mkdir(folderPath)
    copyfile(templateFolderPath, folderPath)
    addpath(genpath('Benchmark'));  % Add the newly created directories to the matlab path
end
% 
% 
% if isunix
%     unix(sprintf("mkdir -p %s/%s_%s/decim%d/LaTeX/", benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1))));
%     unix(sprintf("touch %s/%s_%s/decim%d/LaTeX/FOMtab.tex", benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, benchmarkSettings.decimRate(index(1))));
% end

file = fopen(filePath, 'w');



fprintf(file, "\\documentclass[letterpaper, 10 pt, conference]{ieeeconf} \n");
fprintf(file, "\n");
fprintf(file, "\\IEEEoverridecommandlockouts\n");
fprintf(file, "\n");  
fprintf(file, "\\overrideIEEEmargins\n");
fprintf(file, "\n");  
fprintf(file, "\\usepackage{cite}\n");
fprintf(file, "\\usepackage{graphics} \n");
fprintf(file, "\\usepackage[table, dvipsnames]{xcolor} \n");
fprintf(file, "\\usepackage{epsfig} \n");
fprintf(file, "\\usepackage{times} \n");
fprintf(file, "\\usepackage{amsmath} \n");
fprintf(file, "\\usepackage{amssymb}  \n");
fprintf(file, "\\usepackage{color} \n");
fprintf(file, "\\usepackage{subfig} \n");
fprintf(file, "\\usepackage{tikz} \n");
fprintf(file, "\\usepackage{url} \n");
fprintf(file, "\\usepackage{algorithm2e} \n");
fprintf(file, "\\usepackage{threeparttable} \n");
fprintf(file, "\\usepackage{booktabs, dcolumn} \n");
fprintf(file, "\\newcommand\\mc[1]{\\multicolumn{1}{c}{#1}} \n");
fprintf(file, "\\usepackage{tabularx} \n");
fprintf(file, "\\newcolumntype{d}[1]{D{.}{.}{\\#1}} \n");
fprintf(file, "\\usepackage{makecell} \n");
fprintf(file, "\\usepackage{stfloats} \n");
fprintf(file, "\\makeatletter\n");
fprintf(file, "\\def\\endfigure{\\end@float} \n");
fprintf(file, "\\def\\endtable{\\end@float} \n");
fprintf(file, "\\makeatother \n");
fprintf(file, "\\usepackage{ulem} \n");
fprintf(file, "\\usepackage{stfloats} \n");
fprintf(file, "\\usepackage{geometry} \n");
fprintf(file, "\\geometry{ a4paper, total={170mm,257mm}, left=20mm, top=20mm }  \n");
fprintf(file, "\n");
fprintf(file, "\\begin{document} \n");
fprintf(file, "	\\title{\\LARGE \\bf Experiment Report %s Monte Carlo Simulation %d Samples, %s} \n", name, benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint, benchmarkSettings.noiseLevel);
fprintf(file, "	\\author{Quentin~Leboutet, Julien~Roux, Alexandre~Janot,\\\\ \n");
fprintf(file, "	 J.~Rogelio~Guadarrama-Olvera, and~Gordon~Cheng}  \n");
fprintf(file, "	\\maketitle \n");
fprintf(file, "\\noindent \\textbf{Experiment conditions: } \n");
fprintf(file, "\\begin{itemize} \n");
fprintf(file, "	\\item Robot: \\textbf{%s}, %d-DoF manipulator \n",nameTot, robot.nbDOF);
fprintf(file, "	\\item $%d\\times %d = %d$ standard parameters (no $\\boldsymbol{\\tau}_{off}$) \n", robot.nbDOF, robot.numericalParameters.numParam(1), robot.nbDOF*robot.numericalParameters.numParam(1) );
fprintf(file, "	\\item $%d$ base parameters \n", robot.paramVectorSize);
fprintf(file, "	\\item Control frequency: %d Hz \n",benchmarkSettings.f_ctrl);
fprintf(file, "	\\item Sampling frequency: %d Hz \n",benchmarkSettings.f);
fprintf(file, "	\\item Decimation frequency: %d Hz \n",benchmarkSettings.fdecim);
fprintf(file, "	\\item Decimation ratio: %d \n",benchmarkSettings.decimRate);
fprintf(file, "	\\item Butterworth filter cutoff frequency: %d Hz \n",benchmarkSettings.filter.butterworth.freq_fil);
fprintf(file, "	\\item Experiment duration: %d s \n",benchmarkSettings.t_f-benchmarkSettings.t_i);
fprintf(file, "	\\item Noise levels: \\textbf{%s} levels \n",benchmarkSettings.noiseLevel);
fprintf(file, "	\\begin{itemize} \n");
fprintf(file, "		\\item Torque noise standard deviation: $\\boldsymbol{\\sigma_\\tau} = %f N.m $ \n",robot.numericalParameters.sd_tau(1));
fprintf(file, "		\\item Position noise standard deviation: $\\boldsymbol{\\sigma_q} = %f deg $  \n",robot.numericalParameters.sd_q(1));
fprintf(file, "	\\end{itemize} \n");
fprintf(file, "	\\item \\textbf{No regularization} in general, except for PC-OLS-Euclidean, PC-OLS-Entropic, and PC-OLS-ConstPullback. In these cases, the regularization weight is set to $10^{-2}$. \n");
fprintf(file, "\\end{itemize} \n");
fprintf(file, "\\begin{table*}[t!] \n");
fprintf(file, "	\\small \n");
fprintf(file, "	\\centering \n");
fprintf(file, "	\\begin{threeparttable} \n");
fprintf(file, "		\\begin{tabular}{| l | c | c | c | c | c | c |} \n");
fprintf(file, "			\\toprule \n");
fprintf(file, "\\cellcolor{green!45}\\textbf{%s, MCS, decim %d} & \\cellcolor{gray!25}$E(d_{\\boldsymbol{q}}), \\sigma_{\\boldsymbol{q}}$ & \\cellcolor{gray!25}$E(d_{\\boldsymbol{\\tau}}), \\sigma_{\\boldsymbol{\\tau}}$& \\cellcolor{gray!25}$E(d_{t}), \\sigma_{t}$  &\\cellcolor{gray!25} $E(d_{N_{it}}), \\sigma_{N_{it}}$ & \\cellcolor{gray!25}$E(d_{N_{sim}}), \\sigma_{N_{sim}}$ \\\\ \n",name ,benchmarkSettings.decimRate(index(1)));
fprintf(file, "\\midrule\n");
            
for i=1:sum(benchmarkSettings.identificationMethods.showResults)

    % Compute relative torque error:
    [~,nbSamples]=size(Tau_decim);
    W = observationMatrixOrdered(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
%     Y_tau_ref = W*benchmarkSettings.Beta_obj;          % Recomputed torque with real parameter values
    Y_tau_ref = torqueVectorOrdered(Tau_decim);
    Y_tau_recomputed = W*averageParam(:,index(i));  
    
    Y_error_tau = Y_tau_ref - Y_tau_recomputed;   % Recomputed torque with mean value of the estimated parameters
      
    Y_error_tau_joint=zeros(nbSamples,robot.nbDOF);    
    Y_tau_joint=zeros(nbSamples,robot.nbDOF);
    Y_tau_recomputed_joint=zeros(nbSamples,robot.nbDOF);
    for jointNb = 1:robot.nbDOF
        Y_error_tau_joint(:,jointNb) = Y_error_tau((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_joint(:,jointNb) = Y_tau_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_tau_recomputed_joint(:,jointNb) = Y_tau_recomputed((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
%     figure()
%     plot(Y_tau_joint(:,1),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,1),'b')
%     
%     figure()
%     plot(Y_tau_joint(:,2),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,2),'b')
%     
%     figure()
%     plot(Y_tau_joint(:,3),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,3),'b')
%     
%     figure()
%     plot(Y_tau_joint(:,4),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,4),'b')
%     
%     figure()
%     plot(Y_tau_joint(:,5),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,5),'b')
%     
%     figure()
%     plot(Y_tau_joint(:,6),'r')
%     hold on
%     plot(Y_tau_recomputed_joint(:,6),'b')
%     
%     pause
    
    Y_error_tau_med = sqrt(sum(Y_error_tau_joint.^2,2));
    Y_tau_med = sqrt(sum(Y_tau_joint.^2,2));
    d_tau(i) = 100*mean(Y_error_tau_med./Y_tau_med);
    sigma_d_tau(i) = 100*std(Y_error_tau_med./Y_tau_med);
    
    % Compute relative joint position error:
    t_control = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
    augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_control, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];
    [~, stateVector, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, averageParam(:,index(i)), robot.name, robot.numericalParameters.Geometry, ...
        robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, ...
        robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, ...
        robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, ...
        robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    Q_sim = stateVector(robot.nbDOF+1:2*robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    Qp_sim = stateVector(1:robot.nbDOF,benchmarkSettings.samplingBorder+1:end-benchmarkSettings.samplingBorder);
    
    Q_sim_decim = [];
    Qp_sim_decim = [];
    for jointNb = 1:robot.nbDOF
        Q_sim_decim(jointNb,:) = decimate(Q_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
        Qp_sim_decim(jointNb,:) = decimate(Qp_sim(jointNb,:),benchmarkSettings.decimRate/benchmarkSettings.decimRate);
    end
    
% figure('Name', 'Q')
%     plot(Q_sim_decim.','b')
%     hold on
%     plot(Q_decim.','r')
%     
%     figure('Name', 'Qp')
%     plot(Qp_sim_decim.','b')
%     hold on
%     plot(Qp_decim.','r')
    
    Y_error_Q = torqueVectorOrdered(Q_decim-Q_sim_decim);
    Y_Q_ref = torqueVectorOrdered(Q_decim);
    for jointNb = 1:robot.nbDOF
        Y_error_Q_joint(:,jointNb) = Y_error_Q((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        Y_Q_joint(:,jointNb) = Y_Q_ref((jointNb-1)*nbSamples+1:jointNb*nbSamples);
    end
    Y_error_Q_med = sqrt(sum(Y_error_Q_joint.^2,2));
    Y_Q_med = sqrt(sum(Y_Q_joint.^2,2));
    d_q(i) = 100*mean(Y_error_Q_med./Y_Q_med);
    sigma_d_q(i) = 100*std(Y_error_Q_med./Y_Q_med);
    
    % Compute average parameter error:
    paramRef = repmat(benchmarkSettings.Beta_obj.',benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1);
    relativeAverageParamErrorsMC = (2/robot.paramVectorSize)*sum(abs(fEP(:,:,index(i)))./(abs(paramRef)+abs(fP(:,:,index(i)))),2); % series of d_\beta for each MC experiment
    d_beta(i) = 100*mean(relativeAverageParamErrorsMC);
    sigma_d_beta(i) = 100*std(relativeAverageParamErrorsMC);
    
    % Get the mean time required to compute one parameter estimate:
    d_t(i) = mean(fT(:,index(i)));
    sigma_d_t(i) = 100*std(fT(:,index(i)))/mean(fT(:,index(i)));
    
    % If applicable, get the number of iterations required to compute one
    % parameter estimate:
    d_i(i) = mean(fI(:,index(i)));
    sigma_d_i(i) = 100*std(fI(:,index(i)))/mean(fI(:,index(i)));
    
    % If applicable, get the number of model recalculations required to compute one
    % parameter estimate:
    d_r(i) = mean(fR(:,index(i)));
    if mean(fR(:,index(i)))~=0
        sigma_d_r(i) = 100*std(fR(:,index(i)))/mean(fR(:,index(i)));
    else
        sigma_d_r(i) = -1;
    end
    
%     mean_fom(1,i) = d_beta(i);
    mean_fom(1,i) = d_q(i);
    mean_fom(2,i) = d_tau(i);
    mean_fom(3,i) = d_t(i);
    mean_fom(4,i) = d_i(i);
    mean_fom(5,i) = d_r(i);
    
%     unit{1} = '\%';
    unit{1} = '\%';
    unit{2} = '\%';
    unit{3} = 's';
    unit{4} = '';
    unit{5} = '';
    
%     sigma_fom(1,i) = sigma_d_beta(i);
    sigma_fom(1,i) = sigma_d_q(i);
    sigma_fom(2,i) = sigma_d_tau(i);
    sigma_fom(3,i) = sigma_d_t(i);
    sigma_fom(4,i) = sigma_d_i(i);
    sigma_fom(5,i) = sigma_d_r(i);
    
    nameAlg = getNameId(benchmarkSettings,index,i);
    
    
    fprintf(file, "\\textbf{%s}", nameAlg);
    
    for j = 1:N_fom
        if j == N_fom
            if ~strcmp(nameAlg,'CLIE') && ~strcmp(nameAlg,'CLOE') ~= ~strcmp(nameAlg,'DIDIM') ~= ~strcmp(nameAlg,'IDIM-IV') ~= ~strcmp(nameAlg,'PC-IDIM-IV') ~= ~strcmp(nameAlg,'PC-DIDIM')
                fprintf(file, " & N.A.");
            else
                fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
            end
        elseif j == 1
            fprintf(file, " & %5.3f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
        else
            fprintf(file, " & %5.2f%s (%1.2f\\%%)", mean_fom(j,i), unit{j}, sigma_fom(j,i));
        end
        
    end
    
    fprintf(file, "\\\\  \n");
end
fprintf(file, "\\bottomrule\n");
fprintf(file, "	\\end{tabular} \n");
fprintf(file, "	\\end{threeparttable} \n");
fprintf(file, "	\\caption{Various figures of merit computed for the %s model, with a sampling frequency $f = %d Hz$ and a decimation rate of %d. These results are obtained using the average values $E(d)$ and standard deviations $\\sigma$ of the %d parameter estimates obtained during the MCS. } \n", nameTot, benchmarkSettings.f, benchmarkSettings.decimRate, benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint);
fprintf(file, "\\end{table*} \n");

fprintf(file, "\\end{document} \n");

fclose(file);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
% if isunix
%     unix(sprintf("sed -i 's/E/\\cdot10^{/g' FOMtab_%s.txt", robot.name));
% end

end

%%%%%%%%%%%%%%%%%%%%%%%
%% generateLaTeXparamTab:
%%%%%%%%%%%%%%%%%%%%%%%

function []=generateLaTeXparamTab(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)

N_param = robot.paramVectorSize;
N_methode = benchmarkSettings.nbAlg;


% Generation of the tabularcontatinning the average error and standard deviation of the error.
file = fopen('tab.txt', 'w');

for i = 1:N_param
    fprintf(file, "\\beta_{%d}= % 5.4f", i, benchmarkSettings.Beta_obj(i));
    
    for j = 1:N_methode
        if benchmarkSettings.identificationMethods.showResults(j)==1
            fprintf(file, " & %5.4f + %.0E}", averageParamError(i, j), stdParamError(i, j));
        end
        
    end
    fprintf(file, "\\\\  \n");
end

fclose(file);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
if isunix
    unix("sed -i 's/E/\\cdot10^{/g' tab.txt");
end

% Generation of the tabular containing the average identified parameters and there standard deviation
averageParameters = zeros(N_param, N_methode);
stdParameters = zeros(N_param, N_methode);

for i=1:N_methode
    if benchmarkSettings.identificationMethods.showResults(i)==1
        Betas = eval(sprintf('resultDataStruct{i}.results_%s.Betas',benchmarkSettings.identificationMethods.algName{i}));
        averageParameters(:,i) = reshape(mean(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize, 1);
        stdParameters(:, i) = reshape(std(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize,1); % maybe replace 0 by 1 to weight by N-1 instead of N iyswIm
    end
end

file_bis = fopen('tab_identified_param.txt', 'w');

for i = 1:N_param
    fprintf(file_bis, "\\beta_{%d}= % 5.4f", i, benchmarkSettings.Beta_obj(i));
    indexMethods = find(benchmarkSettings.identificationMethods.showResults(:));
    
    [~, indexMin(i)] = min(abs(averageParamError(i,indexMethods)));
    counter = 0;
    
    for j = indexMethods(1):indexMethods(end)
        counter = counter +1;
        if benchmarkSettings.identificationMethods.showResults(j)==1
            if counter == indexMin(i) % Best estimate
                fprintf(file_bis, " & \\color{blue}%5.4f \\color{black}+ %.0E}", averageParameters(i, j), stdParameters(i, j));
            else
                fprintf(file_bis, " & %5.4f + %.0E}", averageParameters(i, j), stdParameters(i, j));
            end
        end
    end
    
    fprintf(file_bis, "\\\\  \n");
end

fclose(file_bis);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
if isunix
    unix("sed -i 's/E/\\cdot10^{/g' tab_identified_param.txt");
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateLaTeXparamTabPercent:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=generateLaTeXparamTabPercent(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)

N_param = robot.paramVectorSize;
N_methode = benchmarkSettings.nbAlg;

% Generation of the tabular containing the average identified parameters and there standard deviation
averageParameters = zeros(N_param, N_methode);
stdParameters = zeros(N_param, N_methode);

for i=1:N_methode
    if benchmarkSettings.identificationMethods.showResults(i)==1
        Betas = eval(sprintf('resultDataStruct{i}.results_%s.Betas',benchmarkSettings.identificationMethods.algName{i}));
        averageParameters(:,i) = reshape(mean(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize, 1);
        stdParameters(:, i) = reshape(std(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize,1); % maybe replace 0 by 1 to weight by N-1 instead of N iyswIm
    end
end


file = fopen('tab_identified_param_percent.txt', 'w');

for i = 1:N_param
    fprintf(file, "$\\beta_{%d}$=\\textbf{%1.4f}", i, benchmarkSettings.Beta_obj(i));
    indexMethods = find(benchmarkSettings.identificationMethods.showResults(:));
    
    %     [~, indexMin(i)] = min(abs(averageParamError(i,indexMethods)));
    
    [~,indexMin(i,:)]=sort(abs(averageParamError(i,indexMethods)));
    counter = 0;
    
    for j = indexMethods(1):indexMethods(end) % For all the possible methods
        if benchmarkSettings.identificationMethods.showResults(j)==1
            counter = counter +1;
            if counter == indexMin(i,1) % Best estimate
                fprintf(file, "&\\cellcolor{blue!25}\\textbf{%1.4f}", averageParameters(i, j));
                %               fprintf(file, " & \\color{blue}%1.2f \\color{black} (%1.1f\\%%)", averageParameters(i, j), sqrt(stdParameters(i, j))/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,2) % Second best estimate
                fprintf(file, "&\\cellcolor{cyan!25}\\textbf{%1.4f}", averageParameters(i, j));
            elseif counter == indexMin(i,3) % Third best estimate
                fprintf(file, "&\\cellcolor{green!25}\\textbf{%1.4f}", averageParameters(i, j));
            elseif counter == indexMin(i,end) % Worst estimate
                fprintf(file, "&\\cellcolor{red!25}\\textbf{%1.4f}", averageParameters(i, j));
            elseif counter == indexMin(i,end-1) % Second worst estimate
                fprintf(file, "&\\cellcolor{orange!25}\\textbf{%1.4f}", averageParameters(i, j));
            elseif counter == indexMin(i,end-2) % Third worst estimate
                fprintf(file, "&\\cellcolor{yellow!25}\\textbf{%1.4f}", averageParameters(i, j));
            else
                fprintf(file, "&%1.4f", averageParameters(i, j));
                %               fprintf(file, " & %1.2f (%1.1f\\%%)", averageParameters(i, j), sqrt(stdParameters(i, j))/abs(averageParameters(i, j)));
            end
        end
    end
    fprintf(file, "\\\\  \n");
end
fclose(file);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
% if isunix
%     unix("sed -i 's/E/\\cdot10^{/g' tab_identified_param_percent.txt");
% end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateLaTeXparamTabPercent:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=generateLaTeXparamTabPercent2(robot,  benchmarkSettings, resultDataStruct, averageParamError, stdParamError)

N_param = robot.paramVectorSize;
N_methode = benchmarkSettings.nbAlg;

% Generation of the tabular containing the average identified parameters and there standard deviation
averageParameters = zeros(N_param, N_methode);
stdParameters = zeros(N_param, N_methode);

for i=1:N_methode
    if benchmarkSettings.identificationMethods.showResults(i)==1
        Betas = eval(sprintf('resultDataStruct{i}.results_%s.Betas',benchmarkSettings.identificationMethods.algName{i}));
        averageParameters(:,i) = reshape(mean(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize, 1);
        stdParameters(:, i) = reshape(std(reshape(Betas(1:benchmarkSettings.numberOfInitialEstimates, 1:benchmarkSettings.numberOfExperimentsPerInitialPoint,:), benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize,1); % maybe replace 0 by 1 to weight by N-1 instead of N iyswIm
    end
end


file = fopen('tab_identified_param_percent.txt', 'w');

for i = 1:N_param
    fprintf(file, "$\\beta_{%d}$=\\textbf{%1.4f}", i, benchmarkSettings.Beta_obj(i));
    indexMethods = find(benchmarkSettings.identificationMethods.showResults(:));
    
    %     [~, indexMin(i)] = min(abs(averageParamError(i,indexMethods)));
    
    [~,indexMin(i,:)]=sort(abs(averageParamError(i,indexMethods)));
    counter = 0;
    
    for j = indexMethods(1):indexMethods(end) % For all the possible methods
        if benchmarkSettings.identificationMethods.showResults(j)==1
            counter = counter +1;
            if counter == indexMin(i,1) % Best estimate
                fprintf(file, "&\\cellcolor{blue!25}\\textbf{%1.1f(%1.1f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
                %               fprintf(file, " & \\color{blue}%1.2f \\color{black} (%1.1f\\%%)", averageParameters(i, j), sqrt(stdParameters(i, j))/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,2) % Second best estimate
                fprintf(file, "&\\cellcolor{cyan!25}\\textbf{%1.1f(%1.1f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,3) % Third best estimate
                fprintf(file, "&\\cellcolor{green!25}\\textbf{%1.1f(%1.1f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,end) % Worst estimate
                fprintf(file, "&\\cellcolor{red!25}\\textbf{%1.1f(%1.0f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,end-1) % Second worst estimate
                fprintf(file, "&\\cellcolor{orange!25}\\textbf{%1.1f(%1.0f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
            elseif counter == indexMin(i,end-2) % Third worst estimate
                fprintf(file, "&\\cellcolor{yellow!25}\\textbf{%1.1f(%1.0f)}", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
            else
                fprintf(file, "&%1.1f(%1.1f)", averageParameters(i, j), stdParameters(i, j)/abs(averageParameters(i, j)));
                %               fprintf(file, " & %1.2f (%1.1f\\%%)", averageParameters(i, j), sqrt(stdParameters(i, j))/abs(averageParameters(i, j)));
            end
        end
    end
    fprintf(file, "\\\\  \n");
end
fclose(file);

% replace the format E of matlab with \cdot10^{ of latex using a unix
% command line -> DO NOT WORK IN WINDOWS
% if isunix
%     unix("sed -i 's/E/\\cdot10^{/g' tab_identified_param_percent.txt");
% end
end

%%%%%%%%%%%%%%%%%%%%%%
%% getCovarianceVideo:
%%%%%%%%%%%%%%%%%%%%%%

function []=getCovarianceVideo(Covariance_iteration, algName, iteration, writeVideo)

if nargin < 3
    writeVideo = false;
end

figure('Name','Covariance Matrix')
for i=1:size(Covariance_iteration,3)
    image(1000*Covariance_iteration(:,:,i,iteration));
    title(sprintf('Covariance Matrix %s, Iteration %d', algName, iteration-47));
    F(i) = getframe(gcf);
    drawnow
end

if writeVideo == true
    % create the video writer with 1 fps
    writerObj = VideoWriter(sprintf('covariance_it_%d.avi',iteration));
    writerObj.FrameRate = 10;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)
        % convert the image to a frame
        frame = F(i) ;
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotRecalculatedTorques:
%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotRecalculatedTorques(robot, benchmarkSettings, Q, Qp, Qpp, Tau, t, averageParam, jointNb)

% Compute the observation matrix:
W = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q, Qp, Qpp);

referenceTau = W*benchmarkSettings.Beta_obj;
h=figure('Name','Torques');
plot1 = plot(t, Tau(jointNb,:),'Color',[0.5 0.5 1],'LineWidth',0.5);
hold on
graph(2) = plot(t, referenceTau(jointNb:robot.nbDOF:end),'b', 'LineWidth',3.5);
index = find(benchmarkSettings.identificationMethods.showResults);
j=1;
type = {'--', '-'};
k = -1;
r=1;
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    hold on
    k = k+1;
    recomputedTau = W*averageParam(:,index(i));
    ax = gca;
    if k == 2
        r=r+1;
        k=0;
    end
    ax.ColorOrderIndex = r;
    graph(i+2) = plot(t, recomputedTau(jointNb:robot.nbDOF:end),type{j+1},'LineWidth',1);
    legendInfo{i+2}=sprintf('%s',benchmarkSettings.identificationMethods.algName{index(i)},'Interpreter', 'LaTeX');
    j= ~j;
    
end
hold off
plot1(1,1).Color(4)=0.25;
graph(1) = plot1;
legendInfo{1}='real';
legendInfo{2}='reference';
legend(graph,legendInfo{:})
xlabel('\textbf{Time [s]}','Interpreter', 'LaTeX');
ylabel('\textbf{Torque [N.m]}','Interpreter', 'LaTeX');
title(sprintf('\\textbf{Recomputed torque} \\boldmath{$%s$} \\textbf{joint %d}',robot.name, jointNb),'Interpreter', 'LaTeX')
grid on
grid minor

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,sprintf('%s/%s/recomputedTorqueJoint%d',benchmarkSettings.outputPath ,robot.name, jointNb),'-dsvg','-r0')
savefig(h,sprintf('%s/%s/recomputedTorqueJoint%d.fig',benchmarkSettings.outputPath ,robot.name, jointNb));
status = true;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotRecalculatedTorquesFull:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotRecalculatedTorquesFull(robot, benchmarkSettings, experimentDataStruct, averageParam, saveGraph)

options.filter = 'butterworth';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];

for expNb = 1:1%benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    [t_decim, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~, ~, ~, ~, ~, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end

% Compute the observation matrix:
W = observationMatrix(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
num = [   1,    0.0781,    0.2344;... % Crimson Red
     0,    0.9000,         0.1;... % Real Green
         0.1172,    0.5625,    1.0000;... % Deep Sky Blue
         1.0000,         0,    1.0000;... % Magenta
         0,    1.0000,    1.0000;... % Cyan
    0.8000,    0.2695,         0;... % Orange
         0,    1.0000,    0.25000;... % Light Green
         0,         0,    0.5000;... % Navy Blue
    0.5000,         0,    0.5000;... % Purple
    0.5391,    0.1680,    0.8828;... % BlueViolet
    0,         0,    1.0000;... % Blue
    0.5000,    0.5000,    0.5000];   % Gray

set(groot,'defaultAxesColorOrder',num)

figure('Name','Torques','Renderer', 'painters', 'Position', [10 10 1200 600]);
for jointNb = 1:robot.nbDOF
    referenceTau = W*benchmarkSettings.Beta_obj;
    subplot(ceil(robot.nbDOF/2),2,jointNb)
    graph(1) = plot(t_decim, Tau_decim(jointNb,:),'Color',[0,0,0.5],'LineWidth',1);
    %     hold on
    %     graph(2) = plot(referenceTau(jointNb:robot.nbDOF:end),'g', 'LineWidth',3.5);
    index = find(benchmarkSettings.identificationMethods.showResults);
    j=1;
    type = {'-', '-'};
    k = -1;
    r=1;
    NN = sum(benchmarkSettings.identificationMethods.showResults);
    for i=1:NN
        hold on
        k = k+1;
        recomputedTau = W*averageParam(:,index(i));
        ax = gca;
        if k == 1
            r=r+1;
            k=0;
        end
        nameAlg = getNameId(benchmarkSettings,index,i);
        graph(i+NN+1) = plot(t_decim, Tau_decim(jointNb,:).'-recomputedTau(jointNb:robot.nbDOF:end),'Color',[1/((i+1)^(i/2)),    1/((i+1)^(i/2)),    1/((i+1)^(i/2))],'LineWidth',1);
        legendInfo{i+NN+1}=sprintf('Error %s',nameAlg);
        ax.ColorOrderIndex = r;
        graph(i+1) = plot(t_decim, recomputedTau(jointNb:robot.nbDOF:end),type{j+1},'LineWidth',1);
        legendInfo{i+1}=sprintf('%s',nameAlg);
        
%         ax.ColorOrderIndex = r;
        
        j= ~j;
        
    end
    hold off
%         graph(1,1).Color(4)=1;
    %     graph(1) = plot1;
    legendInfo{1}='real';
    %     legendInfo{2}='reference';
    xlabel('\textbf{Time [s]}','Interpreter', 'LaTeX');
    ylabel(sprintf('\\textbf{Torque [N.m]}'),'Interpreter', 'LaTeX');
    title(sprintf('\\textbf{Recomputed torque} \\boldmath{$%s$} \\textbf{joint %d}',robot.name, jointNb),'Interpreter', 'LaTeX')
    grid on
    grid minor
end
legend(graph,legendInfo{:},'Interpreter', 'LaTeX','Orientation','horizontal')
status = true;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotRecalculatedClosedLoopDDM:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status]=plotRecalculatedClosedLoopDDM(robot, benchmarkSettings, resultDataStruct, experimentDataStruct)

% This function plot the trajectory errors obtained with each identification method
t_control = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples);  % Control epochs
augmentedInitialState = [experimentDataStruct.Qpp(:,1,1); experimentDataStruct.Qp(:,1,1); experimentDataStruct.Q(:,1,1)]; % X = [Qp;Q]:
augmentedDesiredState = benchmarkSettings.trajectoryData.getTrajectoryData(t_control, benchmarkSettings.interpolationAlgorithm); % augmentedState = [Qpp; Qp; Q];

t = linspace(benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbSamples); % Sampling times

for i=1:numel(benchmarkSettings.identificationMethods.showResults)
    if benchmarkSettings.identificationMethods.showResults(i)==1
        Betas = eval(sprintf('resultDataStruct{i}.results_%s.Betas',benchmarkSettings.identificationMethods.algName{i}));
        averageParameters(:,i) = reshape(mean(reshape(Betas, benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize, 1);
        stdParameters(:, i) = reshape(std(reshape(Betas, benchmarkSettings.numberOfInitialEstimates*robot.paramVectorSize,robot.paramVectorSize)), robot.paramVectorSize,1); % maybe replace 0 by 1 to weight by N-1 instead of N iyswIm
    end
end
foundResults = find(benchmarkSettings.identificationMethods.showResults);

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    % Simulate robot dynamic behavior with the estimated parameters
    [~, State_it(:,:,i), ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, averageParameters(:,foundResults(i)), robot.name, robot.numericalParameters.Geometry, ...
        robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, robot.controlParameters.Kp, ...
        robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U,...
        robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, ...
        robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);
    for j=1:size(State_it,2)
        H = feval(sprintf('HT_dh%d_world_%s', robot.nbDOF,robot.name), State_it(robot.nbDOF+1:end,j,i), robot.numericalParameters.Geometry);
        X(:,j,i) = H(1:3,4);
    end
end


figure('Name','Parameter validation')
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    subplot(sum(benchmarkSettings.identificationMethods.showResults),1,i)
    plot((experimentDataStruct.Q(:,:,1) - State_it(robot.nbDOF+1:end,:,i))');
    benchmarkSettings.identificationMethods.algName{foundResults(i)};
    grid on
    grid minor
    legend
    title(sprintf('Joint errors due to parameter mismatch of %s', benchmarkSettings.identificationMethods.algName{foundResults(i)}));
end


figure('Name','Parameter validation 3D')
realParameters = plot3(experimentDataStruct.X(1,:,1), experimentDataStruct.X(2,:,1), experimentDataStruct.X(3,:,1), 'LineStyle','-', 'Linewidth',2,'HandleVisibility','off');
hold on
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    identifiedParameters(i) = plot3(X(1,:,i), X(2,:,i), X(3,:,i), 'LineStyle','--', 'Linewidth',2,'HandleVisibility','off');
    k = benchmarkSettings.identificationMethods.algName{foundResults(i)};
    legendInfo{i}=sprintf('%s trajectory',k);
    hold on
end
grid on
grid minor
hold off
title('Trajectory');
legend([realParameters identifiedParameters],{'Real trajectory' legendInfo{:}})

status = true;
end

%%%%%%%%%%%%%%%%%
%% errorFunction:
%%%%%%%%%%%%%%%%%

function [ Error ] = errorFunction(robot, benchmarkSettings, experimentDataStruct, Beta, index, expNb)
q = experimentDataStruct.Q(:,:,expNb)';

% Simulation of the robot using the computed parameters:
[~, State_it, ~] = integrateClosedLoopDynamics_mex(augmentedDesiredState, averageParameters(:,foundResults(i)), robot.name, robot.numericalParameters.Geometry, ...
    robot.numericalParameters.Gravity, benchmarkSettings.t_i, benchmarkSettings.t_f, benchmarkSettings.nbCtrlSamples, benchmarkSettings.nbSamples, robot.controlParameters.Kp, ...
    robot.controlParameters.Ki, robot.controlParameters.Kd, robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U,...
    robot.physicalConstraints.limQp_L, robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, ...
    robot.physicalConstraints.limTau_U, benchmarkSettings.integrationAlgorithm);

size(q)
size(State_it(robot.nbDOF+1:end,:)')
Err=(q-State_it(robot.nbDOF+1:end,:)');
figure('Name',sprintf('%s',benchmarkSettings.identificationMethods.algName{index}));
subplot(2,1,1)
plot(State_it(robot.nbDOF+1:end,:)','--','LineWidth',2)
hold on
plot(q,'LineWidth',1)
legend('q1 simulated','q1 robot')
xlabel('Samples');
ylabel('Q');
title(sprintf('Trajectory %s',benchmarkSettings.identificationMethods.algName{index}))
grid on
grid minor
subplot(2,1,2)
plot(100*Err)
legend('100 x Error')
xlabel('Samples');
ylabel('Joint Error');
title(sprintf('Trajectory Error %s',benchmarkSettings.identificationMethods.algName{index}))
grid on
grid minor
Error = kstest(Err);
figure
hist(Err)
end

%%%%%%%%%%%%%%%
%% plotSummary:
%%%%%%%%%%%%%%%

function [status] = plotSummary(benchmarkSettings, averageTime, averageIter, averageError, stdError, methods)
% Summary
h = figure('Name','Summary','units','normalized','outerposition',[0 0 1 1]);
colormap parula
cmap = colormap;
subplot(4,1,1)
bm = bar(methods, averageError, 0.5, 'FaceColor', 'flat');
for k = 1:benchmarkSettings.nbAlg
    bm.CData(k,:) =  cmap(k,:);
end
text(1:length(averageError),averageError,num2str(averageError',3),'vert','bottom','horiz','center');
box off
% yticks(1:2:benchmarkSettings.nbAlg);
% ylim([0 benchmarkSettings.nbAlg]);
grid on
grid minor
title("Average error")

subplot(4,1,2);
bs = bar(methods, stdError, 0.5, 'FaceColor', 'flat');
for k = 1:benchmarkSettings.nbAlg
    bs.CData(k,:) =  cmap(k,:);
end
text(1:length(stdError),stdError,num2str(stdError','%.1e'),'vert','bottom','horiz','center');
box off
grid on
grid minor
% ylim([0 3.5])
title("Error Standard Deviation")

subplot(4,1,3)
bt = bar(methods, log10(averageTime+1), 0.5, 'FaceColor', 'flat');
for k = 1:benchmarkSettings.nbAlg
    bt.CData(k,:) =  cmap(k,:);
end
text(1:length(averageTime),log10(averageTime+1),num2str(averageTime','%.1us'),'vert','bottom','horiz','center');
box off
yticks(log10([1 2 10 60 600 3600]));
ylim([0 log10(20000)]);
grid on
grid minor
yticklabels({'0', '1s', '10s', '1min', '10min', '1h'});
title("Average Computation Time")

subplot(4,1,4)
it = bar(methods, averageIter, 0.5, 'FaceColor', 'flat');
for k = 1:benchmarkSettings.nbAlg
    it.CData(k,:) =  cmap(k,:);
end
text(1:length(averageIter),averageIter,num2str(averageIter','%.1u'),'vert','bottom','horiz','center');
box off
grid on
grid minor
title("Average Number of Model Recalculation to Convergence")

set(h,'Units','Inches');

pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Summary','-dpdf','-r0')
status = true;
end

%%%%%%%%%%%%%%%%%%%%%
%% plot3DMeanResults:
%%%%%%%%%%%%%%%%%%%%%

function [status] = plot3DMeanResults(robot, benchmarkSettings, averageParamError)

% Parameter error mean value
h=figure('Name','Parameter error mean value')
colormap jet
b = bar3(1:robot.paramVectorSize, averageParamError(:,find(benchmarkSettings.identificationMethods.showResults)));
%     for k = 1:numel(b)
%         cdata = get(b(k),'zdata');
%         cdata=repmat(max(abs(cdata),[],2),1,4);
%         set(b(k),'cdata',cdata,'facecolor','interp')
%     end
for k = 1:length(b)
    zdata = b(k).ZData;
    b(k).CData = zdata;
    b(k).FaceColor = 'interp';
end
%     daspect([3 20 0.0330]);
grid on
grid minor
title("Average Parameter error")
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Mean results','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotRawResultsParamwise:
%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotRawResultsParamwise(robot, benchmarkSettings, fEP, fT)

colormap hsv
cmap = colormap;
h=figure('Name','Raw Results')
index = find(benchmarkSettings.identificationMethods.showResults);

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    for j=1:robot.paramVectorSize
        graph(i) = plot3(fEP(:,j,index(i))',j*ones(benchmarkSettings.numberOfInitialEstimates*benchmarkSettings.numberOfExperimentsPerInitialPoint,1),log(fT(:,index(i))+1),'.','color',cmap(2*i,:));
        hold on
    end
    legendInfo{i}=sprintf('%s',benchmarkSettings.identificationMethods.algName{index(i)});
end
zlabel('\textbf{Computation Time [s]}','Interpreter', 'LaTeX');
xlabel('\textbf{Error}','Interpreter', 'LaTeX');
ylabel('\textbf{Parameter Index}','Interpreter', 'LaTeX');
grid on
grid minor
% ylim([0, 5])
hold off
title('Algotithm Performance');
legend(graph,legendInfo{:})
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Raw_paramwise','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%
%% plotRawResults:
%%%%%%%%%%%%%%%%%%

function [status] = plotRawResults(benchmarkSettings, fEP, fT)


marker = {'o','+','*','.','x','s','d','^','v','>','<','p','h','o','+','*','.','x','s','d','^','v','>','<','p','h'};
h=figure('Name','Raw Results')
index = find(benchmarkSettings.identificationMethods.showResults);

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    graph(i) = scatter(vecnorm(fEP(:,:,index(i))'),log2(fT(:,index(i))+1),40,marker{i});
    hold on
    legendInfo{i}=sprintf('%s',benchmarkSettings.identificationMethods.algName{index(i)});
end
ylabel('\textbf{Computation Time [s]}','Interpreter', 'LaTeX');
xlabel('\textbf{Error RMS}','Interpreter', 'LaTeX');
grid on
grid minor
ylim([0, 5])
hold off
title('Algotithm Performance');
legend(graph,legendInfo{:})
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Raw_mean','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotRawResultsHistogram:
%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotRawResultsHistogram(benchmarkSettings, fEP, fT)

% Graph computation time vs error RMS
h=figure('Name','Raw Results')
index = find(benchmarkSettings.identificationMethods.showResults);

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    graph(i) = histogram2(vecnorm(fEP(:,:,index(i))'),fT(:,index(i))');
    hold on
    legendInfo{i}=sprintf('%s',benchmarkSettings.identificationMethods.algName{index(i)});
end
ylabel('\textbf{Computation Time [s]}','Interpreter', 'LaTeX');
xlabel('\textbf{Error RMS}','Interpreter', 'LaTeX');
grid on
grid minor
ylim([0, 2])
hold off
title('Algotithm Performance');
legend(graph,legendInfo{:})
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'raw_hist','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotConvergenceStatus:
%%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotConvergenceStatus(robot, benchmarkSettings, convergence, decimRate)

% Convergence Status
h=figure('Name','Convergence')
index = find(benchmarkSettings.identificationMethods.showResults);
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    subplot(ceil(sum(benchmarkSettings.identificationMethods.showResults)/2),2,i)
    imagesc(repmat(convergence(:,:,index(i)),1,1,3));
%     colorbar
    title(sprintf('\\textbf{Convergence} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
    ylabel('Initial estimate', 'Interpreter', 'LaTeX')
    xlabel('Independant run', 'Interpreter', 'LaTeX')
end
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
versionMatlab = ver('matlab');
if str2double(versionMatlab.Version)>=9.8
    exportgraphics(h,sprintf('%s/%s/decim%d/paramError/Conv-%s%s',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)},'.pdf'),'BackgroundColor','none');
else
    print(h,sprintf('%s/%s/decim%d/paramError/Conv-%s',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}), '-dpdf','-r0');
end

status = true;

end

%%%%%%%%%%%%%%%%%%%%%%
%% plotParamwiseHoriz:
%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotParamwiseHoriz(robot, benchmarkSettings, averageParamError, stdParamError)

% Parameter error mean value and stdandard deviation
h=figure('Name','Parameter error mean value and stdandard deviation')
index = find(benchmarkSettings.identificationMethods.showResults);
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    subplot(sum(benchmarkSettings.identificationMethods.showResults),1,i)
    bar(1:robot.paramVectorSize, averageParamError(:,index(i)));
    hold on
    errorbar(averageParamError(:,index(i)), stdParamError(:,index(i)),'.','LineWidth',2)
    hold off
    grid on
    grid minor
    xlabel('\textbf{Index}','Interpreter', 'LaTeX');
    ylabel('\textbf{Error}','Interpreter', 'LaTeX');
    ylim([-0.26 0.26])
    %         text(1:length(averageParamError(:,index(i))),averageParamError(:,index(i)),num2str(averageParamError(:,index(i)),'%10.2e\n'),'Rotation',90);
    box off
    title(sprintf('\\textbf{Average Parameter Errors and Standard Deviations} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
end
legend({'mean', 'std'},'Interpreter', 'LaTeX')
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Hist_horiz','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%%%%
%% plotParamwiseVert:
%%%%%%%%%%%%%%%%%%%%%

function [status] = plotParamwiseVert(robot, benchmarkSettings, averageParamError, stdParamError)

% Parameter error mean value and stdandard deviation 2
h=figure('Name','Parameter error mean value and stdandard deviation 2')
index = find(benchmarkSettings.identificationMethods.showResults);
%     subplot(1,sum(benchmarkSettings.identificationMethods.showResults),1)
%         barh(1:robot.paramVectorSize, averageParamError(:,index(1)));
%         hold on
%         errorbar(averageParamError(:,index(1)),1:robot.paramVectorSize, stdParamError(:,index(1)),'horizontal','.','LineWidth',2)
%         hold off
%         grid on
%         grid minor
%         ylabel('\textbf{Index}','Interpreter', 'LaTeX');
%         xlabel('\textbf{Error}','Interpreter', 'LaTeX');
% %         xlim([-0.26 0.26])
% %         text(max(0,averageParamError(:,index(1))), 1:length(averageParamError(:,index(1))),strcat(strcat(num2str(averageParamError(:,index(1)),'%10.2e\n'),'±'),num2str(stdParamError(:,index(1)),'%10.2e\n')),'vert','bottom','horiz','left','Rotation',0);
%         box off
%         title(sprintf('\\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(1)}), 'Interpreter', 'LaTeX')
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    subplot(1,sum(benchmarkSettings.identificationMethods.showResults),i)
    barh(1:robot.paramVectorSize, averageParamError(:,index(i)));
    hold on
    errorbar(averageParamError(:,index(i)),1:robot.paramVectorSize, stdParamError(:,index(i)),'horizontal','r.','LineWidth',0.5)
    hold off
    grid on
    grid minor
    ylabel('\textbf{Index}','Interpreter', 'LaTeX');
    xlabel('\textbf{Parameter Error}','Interpreter', 'LaTeX');
    %     xlim([-1.5 1.6])
    %         text(max(0,averageParamError(:,index(i)))+0.025, 1:length(averageParamError(:,index(i))),strcat(strcat(num2str(averageParamError(:,index(i)),'%10.2e\n'),'±'),num2str(stdParamError(:,index(i)),'%10.2e\n')),'vert','bottom','horiz','left','Rotation',0);
    box off
    title(sprintf('\\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
end
legend({'mean', 'std'},'Interpreter', 'LaTeX')
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Hist_vert','-dpdf','-r0')
status = true;

end

%%%%%%%%%%%%%%%%%%%%%%%%
%% plotParamwiseVertNew:
%%%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotParamwiseVertNew(robot, benchmarkSettings, averageParamError, stdParamError, decimRate)

% Parameter error mean value and stdandard deviation 2
index = find(benchmarkSettings.identificationMethods.showResults);
%     subplot(1,sum(benchmarkSettings.identificationMethods.showResults),1)
%         barh(1:robot.paramVectorSize, averageParamError(:,index(1)));
%         hold on
%         errorbar(averageParamError(:,index(1)),1:robot.paramVectorSize, stdParamError(:,index(1)),'horizontal','.','LineWidth',2)
%         hold off
%         grid on
%         grid minor
%         ylabel('\textbf{Index}','Interpreter', 'LaTeX');
%         xlabel('\textbf{Error}','Interpreter', 'LaTeX');
% %         xlim([-0.26 0.26])
% %         text(max(0,averageParamError(:,index(1))), 1:length(averageParamError(:,index(1))),strcat(strcat(num2str(averageParamError(:,index(1)),'%10.2e\n'),'±'),num2str(stdParamError(:,index(1)),'%10.2e\n')),'vert','bottom','horiz','left','Rotation',0);
%         box off
%         title(sprintf('\\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(1)}), 'Interpreter', 'LaTeX')
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    h=figure('Name','Parameter error mean value and stdandard deviation 2');
    barh(1:robot.paramVectorSize, averageParamError(:,index(i)), 'FaceColor', [0 .39 .74], 'EdgeColor', [0 .59 .94], 'FaceAlpha',0.75);
    hold on
    errorbar(averageParamError(:,index(i)),1:robot.paramVectorSize, stdParamError(:,index(i)),'horizontal','r.','LineWidth',0.5)
    hold off
    grid on
    grid minor
    ylabel('\textbf{Index}','Interpreter', 'LaTeX');
    xlabel('\textbf{Parameter Error}','Interpreter', 'LaTeX');
    xlim([-0.5 0.5])
    x=0.5;
            text(0.4*(-ones(1,robot.paramVectorSize)).^(0:robot.paramVectorSize-1), linspace(1-x,robot.paramVectorSize-x,robot.paramVectorSize),strcat('err(',strcat(num2str(linspace(1,robot.paramVectorSize,robot.paramVectorSize).','%d\n'),strcat(')=',strcat(strcat(num2str(averageParamError(:,index(i)),'%5.3f\n'),'±'),num2str(stdParamError(:,index(i)),'%5.3f\n'))))), 'Interpreter', 'LaTeX','vert','bottom','horiz','center','Rotation',0,'FontSize', 10);
    box off
%     title(sprintf('\\textbf{Parameter Identification} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
    legend({'mean', 'std'},'Interpreter', 'LaTeX','Location','northoutside','Orientation','horizontal')
    set(h,'PaperPositionMode','Auto','Position', [0 0 960 540])
    h.Color='w';
    h.OuterPosition=h.InnerPosition;
    
    if ~exist(sprintf('%s/%s_%s/decim%d/paramError',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i))), 'dir')
        mkdir(sprintf('%s/%s_%s/decim%d/paramError',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i))))
    end

    print(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
    savefig(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
    
    versionMatlab = ver('matlab');
    if str2double(versionMatlab.Version)>=9.8
        exportgraphics(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)},'.pdf'),'BackgroundColor','none');
    else
        print(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}), '-dpdf','-r0');
    end
end
status = true;

end

function [status] = plotParamwiseMatrix(robot, benchmarkSettings, averageParamError, stdParamError, decimRate)

% Parameter error mean value and stdandard deviation 2
index = find(benchmarkSettings.identificationMethods.showResults);

averageParamErrorMat = [];
stdParamErrorMat = [];
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    averageParamErrorMat = [averageParamErrorMat averageParamError(:,index(i))];
    stdParamErrorMat = [stdParamErrorMat stdParamError(:,index(i))];
    l2Error(i) = norm(averageParamError(:,index(i)));
    name{i} = getNameId(benchmarkSettings,index,i);
end

[~,I] = sort(l2Error,'descend');

for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    namesSorted{i} = name{I(i)};
end
averageParamErrorMatSorted = averageParamErrorMat(:,I);
stdParamErrorMatSorted = stdParamErrorMat(:,I);
[X,Y] = meshgrid(1:robot.paramVectorSize,1:sum(benchmarkSettings.identificationMethods.showResults));
ticks = linspace(1,2*sum(benchmarkSettings.identificationMethods.showResults), sum(benchmarkSettings.identificationMethods.showResults));
figure('Name','Parameter error mean value and stdandard deviation 6');
bar3(ticks,abs(averageParamErrorMatSorted)',0.8);
% colormap(jet);
hold on
q = quiver3(X,Y, abs(averageParamErrorMatSorted)', zeros(size(averageParamErrorMatSorted')), zeros(size(averageParamErrorMatSorted')), abs(averageParamErrorMatSorted)'+abs(stdParamErrorMatSorted)','linewidth',2, 'color', 'r');
q.ShowArrowHead = 'off';
grid on
grid minor
xticks(1:sum(benchmarkSettings.identificationMethods.showResults));
xticklabels(namesSorted);
title("Average Parameter error")
% set(b,'Units','Inches');
% pos = get(b,'Position');
% set(b,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(b,'Mean results','-dpdf','-r0')





% Change the colormap to gray (so higher values are
                         %   black and lower values are white)

% textStrings = num2str(averageParamError(:), '%0.2f');       % Create strings from the matrix values
% textStrings = strtrim(cellstr(textStrings));  % Remove any space padding
% [x, y] = meshgrid(1:52);  % Create x and y coordinates for the strings
% hStrings = text(x(:), y(:), textStrings(:), ...  % Plot the strings
%                 'HorizontalAlignment', 'center');
% midValue = mean(get(gca, 'CLim'));  % Get the middle value of the color range
% textColors = repmat(averageParamError(:) > midValue, 1, 3);  % Choose white or black for the
                                               %   text color of the strings so
                                               %   they can be easily seen over
                                               %   the background color
% set(hStrings, {'Color'}, num2cell(textColors, 2));  % Change the text colors

% set(gca, 'XTick', 1:5, ...                             % Change the axes tick marks
%          'XTickLabel', {'A', 'B', 'C', 'D', 'E'}, ...  %   and tick labels
%          'YTick', 1:5, ...
%          'YTickLabel', {'A', 'B', 'C', 'D', 'E'}, ...
%          'TickLength', [0 0]);
% 
% for i=1:sum(benchmarkSettings.identificationMethods.showResults)
%     h=figure('Name','Parameter error mean value and stdandard deviation 2');
%     barh(1:robot.paramVectorSize, averageParamError(:,index(i)), 'FaceColor', [0 .39 .74], 'EdgeColor', [0 .59 .94], 'FaceAlpha',0.75);
%     hold on
%     errorbar(averageParamError(:,index(i)),1:robot.paramVectorSize, stdParamError(:,index(i)),'horizontal','r.','LineWidth',0.5)
%     hold off
%     grid on
%     grid minor
%     ylabel('\textbf{Index}','Interpreter', 'LaTeX');
%     xlabel('\textbf{Parameter Error}','Interpreter', 'LaTeX');
%     xlim([-0.5 0.5])
%     x=0.5;
%             text(0.4*(-ones(1,robot.paramVectorSize)).^(0:robot.paramVectorSize-1), linspace(1-x,robot.paramVectorSize-x,robot.paramVectorSize),strcat('err(',strcat(num2str(linspace(1,robot.paramVectorSize,robot.paramVectorSize).','%d\n'),strcat(')=',strcat(strcat(num2str(averageParamError(:,index(i)),'%5.3f\n'),'±'),num2str(stdParamError(:,index(i)),'%5.3f\n'))))), 'Interpreter', 'LaTeX','vert','bottom','horiz','center','Rotation',0,'FontSize', 10);
%     box off
% %     title(sprintf('\\textbf{Parameter Identification} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
%     legend({'mean', 'std'},'Interpreter', 'LaTeX','Location','northoutside','Orientation','horizontal')
%     set(h,'PaperPositionMode','Auto','Position', [0 0 960 540])
%     h.Color='w';
%     h.OuterPosition=h.InnerPosition;
%     
%     if ~exist(sprintf('%s/%s_%s/decim%d/paramError',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i))), 'dir')
%         mkdir(sprintf('%s/%s_%s/decim%d/paramError',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i))))
%     end
% 
%     print(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
%     savefig(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
%     
%     versionMatlab = ver('matlab');
%     if str2double(versionMatlab.Version)>=9.8
%         exportgraphics(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)},'.pdf'),'BackgroundColor','none');
%     else
%         print(h,sprintf('%s/%s_%s/decim%d/paramError/paramError-%s',benchmarkSettings.outputPath, robot.name, benchmarkSettings.noiseLevel, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}), '-dpdf','-r0');
%     end
% end
status = true;

end

function [status] = plotParamwiseSeparated(robot, benchmarkSettings, averageParamError, stdParamError)

%% Parameter error mean value and stdandard deviation 3

index = find(benchmarkSettings.identificationMethods.showResults);
subplot(1,sum(benchmarkSettings.identificationMethods.showResults),1)
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    h = figure('Name',sprintf('Parameter error mean value and stdandard deviation %s',benchmarkSettings.identificationMethods.algName{index(i)}),'units','normalized','outerposition',[0 0 1 1]);
    bar(1:robot.paramVectorSize, averageParamError(:,index(i)));
    hold on
    errorbar(averageParamError(:,index(i)), stdParamError(:,index(i)),'.','LineWidth',2)
    hold off
    grid on
    grid minor
    ylabel('\textbf{Index}','Interpreter', 'LaTeX');
    xlabel('\textbf{Error}','Interpreter', 'LaTeX');
    %         xlim([-0.1 0.26])
    %         text(max(0,averageParamError(:,index(i)))+0.025, 1:length(averageParamError(:,index(i))),strcat(strcat(num2str(averageParamError(:,index(i)),'%10.2e\n'),'±'),num2str(stdParamError(:,index(i)),'%10.2e\n')),'vert','bottom','horiz','left','Rotation',0);
    box off
    title(sprintf('\\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}), 'Interpreter', 'LaTeX')
    legend({'mean', 'std'},'Interpreter', 'LaTeX')
    set(h,'Units','Inches');
    pos = get(h,'Position');
    set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(h,sprintf('%s',benchmarkSettings.identificationMethods.algName{index(i)}),'-dpdf','-r0')
end
status = true;

end

%%%%%%%%%%%%%%%%%%%%%%%
%% checkStatHypotheses:
%%%%%%%%%%%%%%%%%%%%%%%

function [status] = checkStatHypotheses(robot, benchmarkSettings, experimentDataStruct, decimRate, averageParam, stdParamError)

options.filter = 'no';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];

for expNb = 1:1 %benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    [~, ~, ~, ~, ~, ~, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end

% [~, ~, ~, ~, ~, ~, Tau_decim, Qpp_decim, Qp_decim, Q_decim, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, 1,[]);

index = find(benchmarkSettings.identificationMethods.showResults);
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    Y_tau = torqueVectorOrdered(Tau_decim);
    W = observationMatrixOrdered(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
    error_e = Y_tau - W*averageParam(:,index(i));
    [~,nbSamples]=size(Tau_decim);
    disp('Relative torque error:')
    
    % Autocorrelation plots:
    h0 = figure('Name','Residuals Autocorrelations');
    for jointNb = 1:robot.nbDOF
        %         error_e_joint = error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        error_e_joint = filloutliers(error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples),'next');
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        [acf,lags,bounds,h] = autocorr(error_e_joint,'NumLags',20,'NumSTD',2);
        grid on
        grid minor
        title('')
        ylabel(sprintf('\\textbf{Link %d}',jointNb),'Interpreter', 'LaTeX');
        xlabel('\textbf{Lag}','Interpreter', 'LaTeX');
    end
    sgtitle(sprintf('\\textbf{Residuals Autocorrelations} \\boldmath{$%s$} \\boldmath{$%s$}', benchmarkSettings.identificationMethods.algName{index(i)}, robot.name),'Interpreter', 'LaTeX')
    
    % Kolmogorov-Smirnov test:
    for jointNb = 1:robot.nbDOF
        %         error_e_joint = error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        error_e_joint = filloutliers(error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples),'next');
        error_e_norm = error_e_joint/std(error_e_joint);
        if kstest(error_e_norm) == 1
            disp(sprintf('Kolmogorov-Smirnov goodness-of-fit hypothesis test %s joint %d: UNSUCCESSFUL',benchmarkSettings.identificationMethods.algName{index(i)}, jointNb))
        else
            disp(sprintf('Kolmogorov-Smirnov goodness-of-fit hypothesis test %s joint %d: SUCCESSFUL',benchmarkSettings.identificationMethods.algName{index(i)}, jointNb))
        end
    end
    
    % Saving figures:
%     set(h0,'Units','Inches');
%     pos = get(h0,'Position');
%     set(h0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%     print(h0,sprintf('%s/%s/decim%d/residualAutocorrelations/residualAutocorrelations-%s',robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
%     savefig(h0,sprintf('%s/%s/decim%d/residualAutocorrelations/residualAutocorrelations-%s.fig',robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
%     
end

status = true;

end

%%%%%%%%%%%%%%%%%%%%%%
%% plotErrorHistogram:
%%%%%%%%%%%%%%%%%%%%%%

function [status] = plotErrorHistogram(robot, benchmarkSettings, experimentDataStruct, decimRate, averageParam, stdParamError)

options.filter = 'no';
Tau_decim = [];
Qpp_decim = [];
Qp_decim = [];
Q_decim = [];
t_decim = [];

for expNb = 1:benchmarkSettings.numberOfExperimentsPerInitialPoint % Concatenating the experiment data
    [~, ~, ~, ~, ~, t_decim_exp, Tau_decim_exp, Qpp_decim_exp, Qp_decim_exp, Q_decim_exp, ~] = getFilteredData(robot, benchmarkSettings, experimentDataStruct, options, expNb,[]);
    if isempty(t_decim)
        dt = t_decim_exp(2)-2*t_decim_exp(1);
    else
        dt = t_decim(end)+t_decim_exp(2)-2*t_decim_exp(1);
    end
    t_decim = [t_decim t_decim_exp+dt];
    Tau_decim = [Tau_decim Tau_decim_exp];
    Qpp_decim = [Qpp_decim Qpp_decim_exp];
    Qp_decim = [Qp_decim Qp_decim_exp];
    Q_decim = [Q_decim Q_decim_exp];
end

index = find(benchmarkSettings.identificationMethods.showResults);
for i=1:sum(benchmarkSettings.identificationMethods.showResults)
    Y_tau = torqueVectorOrdered(Tau_decim);
    W = observationMatrixOrdered(robot.name, robot.paramVectorSize, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, Q_decim, Qp_decim, Qpp_decim);
    error_e = Y_tau - W*averageParam(:,index(i));
    [~,nbSamples]=size(Tau_decim);
    %     disp(' Relative error ')
    %     100*norm(error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples))/norm(Y_tau((jointNb-1)*nbSamples+1:jointNb*nbSamples))
    %     fitdist(error_e/std(error_e),'Normal')
    
    
    h0 =  figure('Name',sprintf('Torque error signal %s', benchmarkSettings.identificationMethods.algName{index(i)}),'units','normalized');
    for jointNb = 1:robot.nbDOF
        error_e_joint = error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        plot(t_decim, error_e_joint)
        %         plot(error_e_joint/std(error_e_joint))
        title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
        xlabel('\textbf{Time [s]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Torque error [N.m]}','Interpreter', 'LaTeX');
        grid on
        grid minor
    end
    
    h1 =  figure('Name',sprintf('Torque error signal %s no outliers', benchmarkSettings.identificationMethods.algName{index(i)}),'units','normalized');
    for jointNb = 1:robot.nbDOF
        error_e_joint = filloutliers(error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples),'next');
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        plot(t_decim, error_e_joint)
        %         plot(error_e_joint/std(error_e_joint))
        title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
        xlabel('\textbf{Time [s]}','Interpreter', 'LaTeX');
        ylabel('\textbf{Torque error [N.m]}','Interpreter', 'LaTeX');
        grid on
        grid minor
    end
    
    h2 =  figure('Name',sprintf('Normalized histogram of %s error and estimated Gaussian', benchmarkSettings.identificationMethods.algName{index(i)}),'units','normalized');
    for jointNb = 1:robot.nbDOF
        error_e_joint = error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples);
        subplot(ceil(robot.nbDOF/2),2,jointNb)
        hst = histfit(error_e_joint/std(error_e_joint));
        grid on
        grid minor
        title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
        xlabel('\textbf{Normalized torque}','Interpreter', 'LaTeX');
        xlim([-6,6]);
        ylabel('\textbf{Population}','Interpreter', 'LaTeX');
        hst(1).FaceColor = [.8 .8 1];
    end
    sgtitle(sprintf('\\textbf{Normalized histograms of} \\boldmath{$%s$} \\textbf{error and estimated Gaussians}', benchmarkSettings.identificationMethods.algName{index(i)}),'Interpreter', 'LaTeX')
    
%     h3 =  figure('Name',sprintf('Normalized histogram of %s error and estimated Gaussian no outliers', benchmarkSettings.identificationMethods.algName{index(i)}),'units','normalized');
%     for jointNb = 1:robot.nbDOF
%         error_e_joint = filloutliers(error_e((jointNb-1)*nbSamples+1:jointNb*nbSamples),'next');
%         size(error_e_joint)
%         subplot(ceil(robot.nbDOF/2),2,jointNb)
%         hst = histfit(error_e_joint/std(error_e_joint));
%         grid on
%         grid minor
%         title(sprintf('\\textbf{Joint %d}', jointNb),'Interpreter', 'LaTeX');
%         xlabel('\textbf{Normalized torque}','Interpreter', 'LaTeX');
%         xlim([-6,6]);
%         ylabel('\textbf{Population}','Interpreter', 'LaTeX');
%         hst(1).FaceColor = [.8 .8 1];
%     end
%     sgtitle(sprintf('\\textbf{Normalized histograms of} \\boldmath{$%s$} \\textbf{error and estimated Gaussians}', benchmarkSettings.identificationMethods.algName{index(i)}),'Interpreter', 'LaTeX')
    
    % Saving figures:
    set(h0,'Units','Inches');
    pos = get(h0,'Position');
    set(h0,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(h0,sprintf('%s/%s/decim%d/errorHistogram/torqueError-%s',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
    savefig(h0,sprintf('%s/%s/decim%d/errorHistogram/torqueError-%s.fig',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
    
    set(h1,'Units','Inches');
    pos = get(h1,'Position');
    set(h1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(h1,sprintf('%s/%s/decim%d/errorHistogram/torqueError-%sNoOutliers',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
    savefig(h1,sprintf('%s/%s/decim%d/errorHistogram/torqueError-%sNoOutliers.fig',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
    
    set(h2,'Units','Inches');
    pos = get(h2,'Position');
    set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(h2,sprintf('%s/%s/decim%d/errorHistogram/histogramTorqueError-%s',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
    savefig(h2,sprintf('%s/%s/decim%d/errorHistogram/histogramTorqueError-%s.fig',benchmarkSettings.outputPath ,robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
    
%     set(h3,'Units','Inches');
%     pos = get(h3,'Position');
%     set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%     print(h3,sprintf('%s/%s/decim%d/errorHistogram/histogramTorqueError-%sNoOutliers',robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}),'-dsvg','-r0')
%     savefig(h3,sprintf('%s/%s/decim%d/errorHistogram/histogramTorqueError-%sNoOutliers.fig',robot.name, decimRate(index(i)), benchmarkSettings.identificationMethods.algName{index(i)}));
%     
end

status = true;
end


%%%%%%%%%%%%%%%%%%%%%%%
%% torqueVectorOrdered:
%%%%%%%%%%%%%%%%%%%%%%%

function [Y_tau] = torqueVectorOrdered(Tau)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building the sampled torque vector.

[nbDOF,nbSamples]=size(Tau);
Y_tau = zeros(nbDOF*nbSamples, 1);

for i=1:nbDOF
    for j=1:nbSamples
        Y_tau((i-1)*nbSamples+j)=Tau(i,j);
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% observationMatrixOrdered:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [W_ord] = observationMatrixOrdered(robotName, paramVectorSize, Geometry, Gravity, Q, Qp, Qpp)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Building of the observation matrix W for the current iteration of Beta

[nbDOF,nbSamples]=size(Q);
W = zeros(nbSamples*nbDOF, paramVectorSize);
W_ord = zeros(nbSamples*nbDOF, paramVectorSize);

for i=1:nbSamples
    % Compute the observation matrix:
    switch robotName % [toyRobot, SCARA, UR3, UR5, UR10, TX40, TX40_uncoupled, RX90, PUMA560, REEMC_right_arm, ICUB_right_arm, NAO_right_arm]
        case 'TX40'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_TX40(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'TX40_uncoupled'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_TX40_uncoupled(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        case 'RV2SQ'
            W(nbDOF*(i-1)+1:nbDOF*i,:) = Regressor_Y_RV2SQ(Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity);
        otherwise
            W(nbDOF*(i-1)+1:nbDOF*i,:) = feval(sprintf('Regressor_Y_%s',robotName),Q(:,i), Qp(:,i), Qpp(:,i), Geometry, Gravity); % Much slower when compiled !
    end
end

for i=1:nbDOF
    W_ord(nbSamples*(i-1)+1:nbSamples*i,:) = W(i:nbDOF:end,:);
end


end


function nameAlg = getNameId(benchmarkSettings,index,i)
    switch benchmarkSettings.identificationMethods.algName{index(i)}
        case 'OLS'
            nameAlg = 'IDIM-OLS';
        case 'OLS_f'
            nameAlg = 'IDIM-OLS filtered';
        case 'WLS'
            nameAlg = 'IDIM-WLS';
        case 'WLS_f'
            nameAlg = 'IDIM-WLS filtered';
        case 'TLS'
            nameAlg = 'IDIM-TLS';
        case 'TLS_f'
            nameAlg = 'IDIM-TLS filtered';
        case 'ML'
            nameAlg = 'ML';
        case 'ML_f'
            nameAlg = 'ML filtered';
        case 'IV'
            nameAlg = 'IDIM-IV';
        case 'DIDIM'
            nameAlg = 'DIDIM';
        case 'CLIE'
            nameAlg = 'CLIE';
        case 'CLOE'
            nameAlg = 'CLOE';
        case 'EKF'
            nameAlg = 'EKF';
        case 'SREKF'
            nameAlg = 'SREKF';
        case 'UKF'
            nameAlg = 'UKF';
        case 'SRUKF'
            nameAlg = 'SRUKF';
        case 'CDKF'
            nameAlg = 'CDKF';
        case 'SRCDKF'
            nameAlg = 'SRCDKF';
        case 'ANN'
            nameAlg = 'ANN';
        case 'ANN_f'
            nameAlg = 'ANN filtered';
        case 'HTRNN'
            nameAlg = 'HTRNN';
        case 'HTRNN_f'
            nameAlg = 'HTRNN filtered';
        case 'IRLS'
            nameAlg = 'IRLS';
        case 'IRLS_f'
            nameAlg = 'IRLS filtered';
        case 'PC_OLS_Euclidean'
            nameAlg = 'PC-OLS Euclidean';
        case 'PC_OLS_Euclidean_f'
            nameAlg = 'PC-OLS filtered Euclidean';
        case 'PC_OLS_Entropic'
            nameAlg = 'PC-OLS Entropic';
        case 'PC_OLS_Entropic_f'
            nameAlg = 'PC-OLS filtered Entropic';
        case 'PC_OLS_ConstPullback'
            nameAlg = 'PC-OLS ConstPullback';
        case 'PC_OLS_ConstPullback_f'
            nameAlg = 'PC-OLS filtered ConstPullback';
        case 'PC_IV'
            nameAlg = 'PC-IDIM-IV';
        case 'PC_DIDIM'
            nameAlg = 'PC-DIDIM';
        case 'PC_OLS'
            nameAlg = 'PC-IDIM-OLS';
        case 'PC_OLS_f'
            nameAlg = 'PC-IDIM-OLS filtered';
        case 'PC_WLS'
            nameAlg = 'PC-IDIM-WLS';
        case 'PC_WLS_f'
            nameAlg = 'PC-IDIM-WLS filtered';
        case 'PC_IRLS'
            nameAlg = 'PC-IDIM-IRLS';
        case 'PC_IRLS_f'
            nameAlg = 'PC-IDIM-IRLS filtered';
        otherwise
            benchmarkSettings.identificationMethods.algName{index(i)}
            error('Unknown algorithm');
    end
end


