function [robot] = TX40_uncoupled(options, varargin)

[robot] = TX40(options);
robot.name = 'TX40_uncoupled';

if isfield(robot,'symbolicParameters')
robot.symbolicParameters.Xhi = robot.symbolicParameters.Xhi(1:end-2);
robot.symbolicParameters.Xhi_aug = robot.symbolicParameters.Xhi_aug(1:end-2);
end

robot.numericalParameters.Xhi = robot.numericalParameters.Xhi(1:end-2);
robot.numericalParameters.numParam(end) = robot.numericalParameters.numParam(end)-2;

switch options.noiseLevel
    
    case 'oldNoise'
        robot.numericalParameters.sd_q = [0.057e-3; 0.057e-3; 0.122e-3; 0.114e-3; 0.122e-3; 0.172e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5e-2*ones(robot.nbDOF,1);                % Noise standard deviation
   
   case 'lowPositionNoise'
        % Standard
        robot.numericalParameters.sd_q = [1e-4; 1e-4; 1e-4; 1e-4; 1e-4; 1e-4].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation 
        
    case 'standardNoise'
        % Standard
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highTorqueNoise'
        % High torque noise
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 10*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highPositionNoise'
        % High position noise
        robot.numericalParameters.sd_q = 10*[1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    case 'highPositionTorqueNoise'
        % High torque and position noise
        robot.numericalParameters.sd_q = 10*[1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 10*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
        
    otherwise
        % Standard
        robot.numericalParameters.sd_q = [1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3].*pi/180; % Noise standard deviation
        robot.numericalParameters.sd_tau = 5*[1e-2; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];   % Noise standard deviation
end

end

