function [ J ] = computeNumJacobian(x, f, jacobianOptions)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Computes the numerical Jacobian matrix of the function "f"

f_x  = f(x);

numRows = length(f_x);
numCols = numel(x);

if numRows == 0 || numCols == 0
    error('Numerical Jacobian estimate provided with zero-size variable...');    
end

J = zeros(numRows, numCols);
dx = zeros([numCols, 1]);

hCalcGradFun = @(dx) (f(x + dx) - f_x) / jacobianOptions.epsVal;
for i = 1:numCols
    dx(i) = jacobianOptions.epsVal;
    J(:, i) = hCalcGradFun(dx);
    dx(i) = 0;
end

end


