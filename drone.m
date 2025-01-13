clc
clear
close all

% Parametri
g = 9.81;       % m/(s^2),
m = 0.45;       % kg, peso del drone
l = 0.23;       % m, Distanza dell'elica dal centro di massa
k = 7.5e-7;     % Coefficiente di portanza delle eliche
c = 3e-6;       % Coefficiente di resistenza aereodinamica

%%% Inerzie
% I_M = 3.357e-5;     % kg*m^2, Inertia Moment of the rotor
Ir = 6e-5;     % kg*m^2, Inertia of Gyroscopic Effect
Ixx = 5e-3;    % kg*m^2,
Iyy = 5e-3;    % kg*m^2,
Izz = 8e-3;    % kg*m^2,
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

%%%%%%%%%%%%% CONDIZIONI INIZIALI

%%%%%%%%%%%%%%%%%%%%%

% Equazioni del sistema
% x1 = y
% x2 = y_dot             % velocità di traslazione
% x3 = theta
% x4 = theta_dot = omega % velocità rotazionale

% Pitch (su e giù)
x1_dot_z = x2;
x2_dot_z = 1/m*(-g*x3 - c*x2);
x3_dot_z = x4;
x4_dot_z = p/Izz*u2;
x_dot_z = [x1_dot_z; x2_dot_z; x3_dot_z; x4_dot_z];

% Roll (destra e sinistra)
x1_dot_y = x2;
x2_dot_y = 1/m*(-g*x3 - c*x2);
x3_dot_y = x4;
x4_dot_y = p/Iyy*u2;
x_dot_y = [x1_dot_y; x2_dot_y; x3_dot_y; x4_dot_y];

% Forxza di thrust
T = cos(x3)/(m*g);

% Funzione di controllo
h_x = 
C = forwarding();

% Funzione per osservatore