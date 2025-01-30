clear all;
close all;
clc;
[t, etats] = simulation_drone();

V = etats(:, 1:3);     
angles = etats(:, 7:9); 
positions = cumtrapz(t, V);

animation(positions, angles);
