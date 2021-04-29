function [Xhi, numParam, Xhi_aug] = getStandardParameterVector(InertiaDH, Moment, Mass, Ia, frictionModel, frictionParameters, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% This function generates the vector of standard parameters Xhi of a robot
% from its separated dynamic and friction parameters 


% Check dimentionality

if (size(InertiaDH,3) == size(Moment,2)) && (size(InertiaDH,3) == length(Mass)) && (size(InertiaDH,3) == length(Ia)) 
    nbDOF = size(InertiaDH,3);
    numParam = zeros(nbDOF,1);
else
    error('getStandardParameterVector(): Error in parameter dimentionality !');
end

for i=1:nbDOF
    XXi = InertiaDH(1,1,i); 
    XYi = InertiaDH(1,2,i); 
    XZi = InertiaDH(1,3,i); 
    YYi = InertiaDH(2,2,i);
    YZi = InertiaDH(2,3,i);
    ZZi = InertiaDH(3,3,i);
    Mi = Mass(i);
    MXi = Moment(1,i);
    MYi = Moment(2,i);
    MZi = Moment(3,i);
    switch frictionModel
        % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
        % Nonlinear friction models require state dependant parameter identification
        case 'no'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Tau_off(i)];
        case 'Viscous'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fv(i); frictionParameters.Tau_off(i)];
        case 'Coulomb'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fc(i); frictionParameters.Tau_off(i)];
        case 'ViscousCoulomb'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fv(i); frictionParameters.Fc(i)];
        case 'ViscousCoulombOff'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fv(i); frictionParameters.Fc(i); frictionParameters.Tau_off(i)];
        case 'Stribeck'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Tau_off(i)];
        case 'LuGre'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Tau_off(i)];
        otherwise
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Tau_off(i)];
    end
    numParam(i) = numel(Xhi_block);
    Xhi((i-1)*numel(Xhi_block)+1:i*numel(Xhi_block),1)=Xhi_block;
    % Total Dynamic parameters vector:
    Xhi_tot_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fv(i); frictionParameters.Fc(i); frictionParameters.Fs(i); frictionParameters.Vs(i); frictionParameters.Es(i); frictionParameters.Sigma_0(i); frictionParameters.Sigma_1(i); frictionParameters.Sigma_2(i); frictionParameters.Z0(i)]; % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters...
    Xhi_aug((i-1)*numel(Xhi_tot_block)+1:i*numel(Xhi_tot_block),1) = Xhi_tot_block;
end
end

