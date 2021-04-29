function [x_aug, P, S, Particules, w, status] = iterateKalman(robot, benchmarkSettings, index, t, optionsKF, x_aug, u, P, S, y, Rv, Rn, Sv, Sn, Particules, w, alpha, beta, kappa, h, augmentedDesiredState, useComputedTorque)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

status = 'ok';

switch (optionsKF.type)
    case 'ekf' % Extended Kalman Filter
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, P, ~] = ekf_opt(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, P, ~] = ekf_opt_mex(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
    case 'srekf' % Square Root Extended Kalman Filter
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, S, ~] = srekf_opt(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, S, ~] = srekf_opt_mex(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
        P=S*S';
    case 'ukf' % Unscented Kalman Filter
        switch optionsKF.sigmaCompute
            case 'chol'
                computeMethod = false;
            case 'svd'
                computeMethod = true;
        end
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, P, ~] = ukf_opt(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, alpha, beta, kappa, computeMethod, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, P, ~] = ukf_opt_mex(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, alpha, beta, kappa, computeMethod, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
    case 'srukf' % Square Root Unscented Kalman Filter
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, S, ~] = srukf_opt(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, alpha, beta, kappa, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, S, ~] = srukf_opt_mex(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, alpha, beta, kappa, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
        P=S*S';
    case 'cdkf' % Central Difference Kalman Filter
        switch optionsKF.sigmaCompute
            case 'chol'
                computeMethod = false;
            case 'svd'
                computeMethod = true;
        end
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, P, ~] = cdkf_opt(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, h, computeMethod, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, P, ~] = cdkf_opt_mex(t(index-1), t(index), x_aug, u, P, y, Rv, Rn, h, computeMethod, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
    case 'srcdkf' % Square Root Central Difference Kalman Filter
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, S, ~] = srcdkf_opt(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, h, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, S, ~] = srcdkf_opt_mex(t(index-1), t(index), x_aug, u, S, y, Sv, Sn, h, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
        P=S*S';
    case 'pf' % Particle Filter
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, Particules, w, ~] = pf_opt(t(index-1), t(index), Particules, w, u, y, Rv, Rn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, optionsKF.resampleThreshold, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, Particules, w, ~] = pf_opt_mex(t(index-1), t(index), Particules, w, u, y, Rv, Rn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, optionsKF.resampleThreshold, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
    case 'sppf' % Sigma Point Particle Filter
        if index == 2
            S = repmat(S,1,1,optionsKF.nbParticules);
        end
        
        if mod(index,optionsKF.resampleThreshold)==0
            resample = true;
        else
            resample = false;
        end
        switch (benchmarkSettings.codeImplementation)
            case 'classic'
                [x_aug, Particules, w, ~] = sppf_opt(t(index-1), t(index), Particules, w, u, y, S, Sv, Sn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, optionsKF.typeSPPF,  alpha, beta, kappa, h, resample, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            case 'optim'
                [x_aug, Particules, w, ~] = sppf_opt_mex(t(index-1), t(index), Particules, w, u, y, S, Sv, Sn, robot.name, robot.numericalParameters.Geometry, robot.numericalParameters.Gravity, optionsKF.typeSPPF,  alpha, beta, kappa, h, resample, ...
                    benchmarkSettings.integrationAlgorithm, benchmarkSettings.dt_ctrl, augmentedDesiredState, robot.controlParameters.Kp, robot.controlParameters.Ki, robot.controlParameters.Kd, ...
                    robot.controlParameters.Ktau, robot.controlParameters.antiWindup, robot.physicalConstraints.limQ_L, robot.physicalConstraints.limQ_U, robot.physicalConstraints.limQp_L, ...
                    robot.physicalConstraints.limQp_U, robot.physicalConstraints.limQpp_L, robot.physicalConstraints.limQpp_U, robot.physicalConstraints.limTau_L, robot.physicalConstraints.limTau_U, useComputedTorque);
            otherwise
                fprintf('\n Error : Incorrect filter type. \n');
                status = 'error';
        end
    otherwise
        fprintf('\n Error : Unknown filter type. \n');
        status = 'error';
end

end

