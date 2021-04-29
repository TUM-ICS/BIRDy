function noisyState = noisyMeasurement(State, sd)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function adds Gaussian noise with a standard deviation sd to the measured quantity. 

noisyState = State + diag(sd)*randn(size(State));

end

