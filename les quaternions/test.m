clear all;
close all;
clc;
[t, etats] = simulation_drone();

% Extraire les vitesses et quaternions
V = etats(:, 1:3);     % Vx, Vy, Vz
quaternions = etats(:, 7:10); % quaternions

% Calculer les positions par intégration des vitesses
positions = cumtrapz(t, V);



% Appeler la fonction d'animation
animation(positions, quaternions);