function x_next = rk2_IntegrationStep(ode_fun,x,u,h)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Heun RK2 integration step.

k1 = ode_fun(x,u);
k2 = ode_fun(x+h/2.*k1,u);
x_next = x + h*k2;

end