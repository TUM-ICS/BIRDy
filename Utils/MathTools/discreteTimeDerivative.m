function [upt] = discreteTimeDerivative(ut, ut_1, dt)
% Compute the discrete-time derivative
upt  = (ut - ut_1)/dt;
end

