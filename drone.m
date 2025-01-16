clc
clear
close all

%% Parametri
g = 9.81;       % m/(s^2),
m = 0.45;       % kg, peso del drone
l = 0.23;       % m, Distanza dell'elica dal centro di massa
kk = 7.5e-7;     % Coefficiente di portanza delle eliche
b = 3e-6;       % Coefficiente di resistenza aereodinamica
p = 1;

%%% Inerzie
% I_M = 3.357e-5;     % kg*m^2, Inertia Moment of the rotor
Ir = 6e-5;     % kg*m^2, Inertia of Gyroscopic Effect
Ixx = 5e-3;    % kg*m^2,
Iyy = 5e-3;    % kg*m^2,
Izz = 8e-3;    % kg*m^2,
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% CONDIZIONI INIZIALI
X_o = [2; 5];
% X_o = [0; 0; 0; 0];
Xhat_o = [0; 0];


% Equazioni del sistema
% x1 = y
% x2 = y_dot             % velocità di traslazione
% x3 = theta
% x4 = theta_dot = omega % velocità rotazionale

% % Pitch (su e giù)
% x1_dot_z = x2;
% x2_dot_z = 1/m*(-g*x3);
% x3_dot_z = x4;
% x4_dot_z = p/Izz*u2;
% x_dot_z = [x1_dot_z; x2_dot_z; x3_dot_z; x4_dot_z];

%% Funzione per osservatore di theta
global c1 c2 c3 c4 k omegad1 d1 xtheta xtheta_hat

c3 = 15; 
c4 = 50;  
k = 100;
c = [c1 c2 c3 c4];
% k = 100;

% valori disturbo
d1 = 0.1;
omegad1 = 2*pi*50;

odeoptions = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,x] = ode23(@osservatore_theta,[0 20],[X_o;Xhat_o],odeoptions);


%%
figure(1)
ax(1) = subplot(2,1,1);
plot(t,x(:,1),'-b',t,x(:,3),'--r','LineWidth',2);
legend('x_1','$\hat{x_1}$', 'Interpreter', 'latex');
title(ax(1), "Posizione")
% set(gca,'FontSize',18)
grid on

ax(2) = subplot(2,1,2);
plot(t,x(:,2),'-b',t,x(:,4),'--r','LineWidth',2);
legend('x_2','$\hat{x_2}$', 'Interpreter', 'latex');
xlabel('Time [s]')
title(ax(2), "Velocità")
% set(gca,'FontSize',18)
grid on

figure(2)
ax(1) = subplot(2,1,1);
plot(t,x(:,1)-x(:,3),'LineWidth',2);
legend('e_1');
title(ax(1), "Errore stima posizione")
% set(gca,'FontSize',18)
grid on

ax(2) = subplot(2,1,2);
plot(t,x(:,2)-x(:,4),'LineWidth',2);
legend('e_2');
xlabel('Time [s]')
title(ax(2), "Errore stima velocità")
% set(gca,'FontSize',18)
grid on


%% Funzione per osservatore di y
% CONDIZIONI INIZIALI
X_o = [2; 5];
% X_o = [0; 0; 0; 0];
Xhat_o = [0; 0];

c1 = 15; 
c2 = 50;  
k = 100;
c = [c1 c2 c3 c4];
% k = 100;
xtheta = x(end,2);
xtheta_hat = x(end,4);

% valori disturbo
d1 = 0.1;
omegad1 = 2*pi*50;

odeoptions = odeset('RelTol',1e-8,'AbsTol',1e-10);
[tt,xx] = ode23(@osservatore_y,[0 20],[X_o;Xhat_o],odeoptions);

%%
figure(3)
ax(1) = subplot(2,1,1);
plot(tt,xx(:,1),'-b',tt,xx(:,3),'--r','LineWidth',2);
legend('x_3','$\hat{x_3}$', 'Interpreter', 'latex');
title(ax(1), "Theta")
% set(gca,'FontSize',18)
grid on

ax(2) = subplot(2,1,2);
plot(tt,xx(:,2),'-b',tt,xx(:,4),'--r','LineWidth',2);
legend('x_4','$\hat{x_4}$', 'Interpreter', 'latex');
xlabel('Time [s]')
title(ax(2), "Omega")
% set(gca,'FontSize',18)
grid on

figure(4)
ax(1) = subplot(2,1,1);
plot(tt,xx(:,1)-xx(:,3),'LineWidth',2);
legend('e_3');
title(ax(1), "Errore stima theta")
% set(gca,'FontSize',18)
grid on

ax(2) = subplot(2,1,2);
plot(tt,xx(:,2)-xx(:,4),'LineWidth',2);
legend('e_4');
xlabel('Time [s]')
title(ax(2), "Errore stima omega")
% set(gca,'FontSize',18)
grid on


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FORWARDING
% il resto del sistema è dx = f(x) + g(x)*u
% Studio stabilità dx con u=0
% Verifica delle assunzioni, dal teorema sappiamo che u è GAS+LES
clc
forwarding();