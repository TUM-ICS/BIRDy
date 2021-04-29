function x_next = rk1_IntegrationStep(ode_fun,x,u,h)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Euler RK1 integration step.

x_next = x + h*ode_fun(x,u);

end