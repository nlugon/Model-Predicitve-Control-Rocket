function [x_next] = RK4(X,U,h,f)
%%
% Runge-Kutta 4 integration
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%%

k1_x= f(X,U);
k2_x = f(X+h/2*k1_x,U);
k3_x = f(X+h/2*k2_x,U);
k4_x = f(X+h*k3_x,U);

x_next = X + h * (k1_x/6 + k2_x/3 + k3_x/3 + k4_x/6);
end
