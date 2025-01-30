function [t, etats] = simulation_drone()


PARAM.g = 9.81;                                 
PARAM.m = 1;                            
PARAM.l = 1;                                  
PARAM.r = 0.1;                                 
PARAM.h = 0.2;                                 
PARAM.I = [(1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0 0;0 (1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0;0 0 (1/2)*PARAM.m*PARAM.r^2];
PARAM.alpha = [1;-1;1;-1;1;-1];                 
PARAM.beta = [1;1;1;1;1;1];                     

%x0 = [V_0;omega_0;phi_0;theta_0;psi_0];

%x0 = [0;0;0;0;0;0;-pi/3;0;0];
%x0 = [0;0;0;0;0;0;0;pi/3;0];
%x0 = [0;0;0;0;0;0;0;-pi/3;-pi/3];
%x0 = [0;0;0;0;0;0;0;0;0];

%u = [1.2;1.2;1.2;1.2;1.2;1.2];
%u = [1.5,1.5,1.5,1.5,1.5,1.5];
%u = [1.5; 1.49; 1.5; 1.49; 1.5; 1.49];

u = [1.299;1.299;1.3;1.3;1.3;1.299];
x0 = [0;0;1;0;0;0;0;0;0];

tspan = 0:1e-2:2;
f = @(t, x) modele_dynamique(t, x, u, PARAM);
[t, etats] = ode45(f, tspan, x0);

end