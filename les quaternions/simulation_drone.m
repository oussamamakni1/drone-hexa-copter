function [t, etats] = simulation_drone()
    PARAM.g = 9.81;                                 
    PARAM.m = 1;                                    
    PARAM.l = 4;                                
    PARAM.r = 0.1;                                  
    PARAM.h = 0.2;                                  
    PARAM.I = [(1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0 0; 0 (1/12)*PARAM.m*(3*PARAM.r^2+PARAM.h^2) 0; 0 0 (1/2)*PARAM.m*PARAM.r^2]; 
    PARAM.alpha = [1; -1; 1; -1; 1; -1];            
    PARAM.beta = [1; 1; 1; 1; 1; 1];                

    % x0 = [V_0; omega_0; phi_0; theta_0; psi_0];
    V_0 = [0; 0; 1];
    Omega_0 = [0; 0; 0];
    q_0 = euler_to_quaternion(0, 0, 0); 
    x0 = [V_0; Omega_0; q_0];
    u = [1.3;1.3;1.30001;1.30001;1.30001;1.3];

    %u = [1.8; 1.79; 1.79; 1.8; 1.8; 1.8];
    %u = [2; 1.9; 2; 1.9; 2; 1.9];
    tspan = 0:0.03:15;
    f = @(t, x) modele_dynamique(t, x, u, PARAM);
    [t, etats] = ode45(f, tspan, x0);
end

