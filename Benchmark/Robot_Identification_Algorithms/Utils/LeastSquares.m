function [Beta_LS] = LeastSquares(robot, W, Y_tau, solver, regularizerType, gammaReg, physicalityConstraint, Xhi_0, Beta_0, Z, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the least square solution Beta_LS to the problem min ||W*Beta-Y_tau||^2
% The Z matrix is referred to as the instrument matrix, used in the instrumental variable approach.

if nargin<10
    
    assert(size(W,1) == length(Y_tau), 'LeastSquares.m: Matrix size incompatibility issue between W and Y_tau')
    instrumentalVariables = false;
    
else

    assert((size(W,1) == size(Z,1))&&(size(W,2) == size(Z,2)), 'LeastSquares.m: Matrix size incompatibility issue between W and Z')
    instrumentalVariables = true;
    
end

% Set of unidentifiable parameters:
Xhi_d = feval(sprintf('Regressor_Xhi_d_%s', robot.name),Xhi_0);
nd = length(Xhi_d);

switch solver
    
    case 'backslash'
        
        assert(~physicalityConstraint, 'Solving a physically consistent LS requires a SDP solver such as cvx or mosek')
        
        switch regularizerType
            
            case 'no'
                
                if instrumentalVariables
                    Beta_LS = inv(Z.'*W)*(Z.'*Y_tau);
                else
                    Beta_LS = W\Y_tau;
                end
                
            case 'Euclidean' % Euclidian regularization
                
                if instrumentalVariables
                    Beta_LS = inv(Z.'*W + gammaReg*eye(length(Beta_0)))*(Z.'*Y_tau+gammaReg*Beta_0);
                else
                    Beta_LS = (W.'*W  + gammaReg*eye(length(Beta_0)))\(W.'*Y_tau+gammaReg*Beta_0);
                end
                
            case 'Entropic' % Entropic divergence regularization
                
                error('Entropic divergence regularization is only implemented for cvx and mosek solvers.');
                
            case 'ConstPullback' % Constant Pullback distance regularization
                
                error('ConstPullback distance regularization is only implemented for cvx and mosek solvers.');
                
            otherwise
                
                error('Unknown regularizer type');
        end
        
    case 'pinv'
        
        assert(~physicalityConstraint, 'Solving a physically consistent LS requires a SDP solver such as cvx or mosek')
        assert(strcmp(regularizerType,'no'),'Regularizers are only implemented for backslash, cvx and mosek solvers.')
        assert(instrumentalVariables==false, 'pinv is unsuitable for use of instrumental variables');
        
        Beta_LS = pinv(W)*Y_tau;
        
    case {'cvx','mosek'}
        
        if physicalityConstraint == true
            cvx_begin sdp
        else
            cvx_begin
        end
        
        variable Beta_LS(length(Beta_0))         % Base parameter vector
        if physicalityConstraint == true
            variable P(7,7,robot.nbDOF) semidefinite    % Pseudo inertia
        end
        expression J_LS(1)                              % Least square error
        expression J_reg(1)                             % Regularizer
        
        if instrumentalVariables
            A = (Z/(Z'*Z))*(Z'*W);
            b = (Z/(Z'*Z))*(Z'*Y_tau);
            % Least squares objective
            % Previous formulation (2-norm) 
            %   J_LS = norm( A * Beta_LS - b,2);
            % New formulation (squared 2-norm)
            J_LS = Beta_LS'*A'*A*Beta_LS -2 * b' * A * Beta_LS + b'*b;
        else
            J_LS = norm(Y_tau-W*Beta_LS,2); % see http://cvxr.com/cvx/doc/advanced.html#quad-forms
        end
        
        switch regularizerType
            
            case 'no'
                
                J_reg = 0;
                
            case 'Euclidean' % Euclidian regularization
                
                Xhi_reconstructed = robot.Perm*[eye(length(Beta_0)), -robot.K_d;zeros(nd, length(Beta_0)), eye(nd)]*[Beta_LS;Xhi_d];
                J_reg = norm(Xhi_reconstructed-Xhi_0,2); % see http://cvxr.com/cvx/doc/advanced.html#quad-forms
                
            case 'Entropic' % Entropic divergence regularization
                
                P0 = buildLMI(Beta_0,Xhi_d,robot.K_d,robot.Perm,robot.nbDOF,robot.numericalParameters.numParam);
                P0_inv = zeros(size(P0));
                for i = 1:robot.nbDOF
                    P0_inv(:,:,i) = inv(P0(:,:,i));
                end
                for i = 1:robot.nbDOF
                    J_reg = J_reg -log_det(P(:,:,i)) + trace(P0_inv(:,:,i)*P(:,:,i));
                end
                
            case 'ConstPullback' % Constant Pullback distance regularization
                
                P0 = buildLMI(Beta_0,Xhi_d,robot.K_d,robot.Perm,robot.nbDOF,robot.numericalParameters.numParam);
                P0_inv = zeros(size(P0));
                for i = 1:robot.nbDOF
                    P0_inv(:,:,i) = inv(P0(:,:,i));
                end
                Xhi_reconstructed = robot.Perm*[eye(length(Beta_0)), -robot.K_d;zeros(nd, length(Beta_0)), eye(nd)]*[Beta_0;Xhi_d];
                [U,Sigma,V] = svd(pullback_metric(Xhi_reconstructed,P0_inv,robot.numericalParameters.numParam));
                Constant_pullback_metric_sqrt = U*sqrt(Sigma)*V';
                J_reg = norm(Constant_pullback_metric_sqrt * (Xhi_reconstructed -Xhi_0),2); % see http://cvxr.com/cvx/doc/advanced.html#quad-forms
            otherwise
                
                error('Unknown regularizer type');
        end
        
        minimize( J_LS + gammaReg * J_reg )
        
        if physicalityConstraint == true
            subject to
            P == buildLMI(Beta_LS,Xhi_d,robot.K_d,robot.Perm,robot.nbDOF,robot.numericalParameters.numParam);
        end
        
        cvx_end
        
    otherwise
        
        error("Least Squares: unknown solver");
end
end
