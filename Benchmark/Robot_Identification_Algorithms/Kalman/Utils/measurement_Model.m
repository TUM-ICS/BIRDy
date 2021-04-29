function y = measurement_Model(X, mNoise)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

nbDOF = numel(mNoise);

% Simple additive model:
% y = X(nbDOF+1:2*nbDOF); % Expected joint position measurement from the encoders
y = X(nbDOF+1:2*nbDOF) + mNoise; % Expected joint position measurement from the encoders

end
