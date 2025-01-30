function x_dot = modele_dynamique(t, x, u, PARAM)
    
    V = x(1:3);
    Omega = x(4:6);
    q = x(7:10); 

    g = PARAM.g;           
    m = PARAM.m;          
    l = PARAM.l;                                               
    I = PARAM.I;               
    alpha = PARAM.alpha;       
    beta = PARAM.beta;

    couple_sum = zeros(3, 1);
    C = alpha .* u.^2;
    for i = 1:6
        couple_sum = couple_sum -l/2*beta(i)*u(i)^2 *[-sin(2*pi*(i-1)/6);cos(2*pi*(i-1)/6);0] + C(i) *[0;0;1];
    end

    % Résolution pour Ω̇
    Omega_dot = I \ (couple_sum - cross(Omega, I * Omega));

    force_sum = zeros(3, 1); 
    for i = 1:6
        P = [0; 0; beta(i) * u(i)^2]; 
        force_sum = force_sum + P;   
    end

    % Résolution pour V̇
    R = quat2rotm(q');
    gravity = [0; 0; m * g];
    V_dot = (1/m) * (force_sum - R' * gravity - cross(Omega, V));

    q_dot = quaternion_derivative(q, Omega);
    x_dot = [V_dot; Omega_dot; q_dot'];
end