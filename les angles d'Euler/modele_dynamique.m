function x_dot = modele_dynamique(t,x,u,PARAM)

V = x(1:3);
Omega = x(4:6);
phi = x(7);
theta = x(8);
psi = x(9);

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
R = get_rotation(phi,theta,psi);
gravity = [0; 0; m * g];
V_dot = (1/m) * (force_sum - R' * gravity - cross(Omega, V));

%Calcul des angles_dot phi theta psi
% Matrice Jacobienne
function J = jacobienne(phi,theta)
R_x = [1    0       0; ...
       0  cos(phi) -sin(phi); ...
       0  sin(phi) cos(phi)];
   
R_y = [cos(theta)  0   sin(theta); ...
       0           1         0; ...
      -sin(theta) 0   cos(theta)]; 
   

R1_t = R_x';
R2 = R_y * R_x;
R2_t = R2';
J = [[1;0;0],R1_t(:,2),R2_t(:,3)];

end

J = jacobienne(phi,theta);
angle_dot = J \ Omega;

phi_dot = angle_dot(1); 
theta_dot = angle_dot(2);  
psi_dot = angle_dot(3); 
x_dot = [V_dot;Omega_dot;phi_dot;theta_dot;psi_dot];
end

