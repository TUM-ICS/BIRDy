function x_next = rk4_IntegrationStep(ode_fun,x,u,h)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng
%
% Runge-Kutta RK4 integration step.

k1 = ode_fun(x,u);
k2 = ode_fun(x+h/2.*k1,u);
k3 = ode_fun(x+h/2.*k2,u);
k4 = ode_fun(x+h.*k3,u);
x_next = x + h/6.*(k1+2*k2+2*k3+k4);

end

