function [iHT] = invHT(HT)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Returns the inverse of the homogeneous transformation HT. 

iHT=HT;
iHT(1:3,1:3)=HT(1:3,1:3)';
iHT(1:3,4)=-HT(1:3,1:3)'*HT(1:3,4);
end

