%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           CONTROLLO FORWARDING
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;

%% 1. PARAMETRI E MODELLO DEL SISTEMA
% =========================================================================
% Parametri fisici
g   = 9.81;   % m/s^2
m   = 0.45;   % kg
Izz = 8e-3;   % kg*m^2
k   = 1/Izz;  % Guadagno attuatore

% Matrici del sistema a 4 stati (per la simulazione)
A_full = [0, 1,  0,    0;
          0, 0, -g/m,  0;
          0, 0,  0,    1;
          0, 0,  0,    0];
B_full = [0; 0; 0; k];
C_full = [1, 0, 0, 0]; % Misuriamo solo x1 (posizione/altitudine)

% Matrici del sottosistema x_a = [x2, x3, x4]' per il progetto
A_sub = [0, -g/m, 0;
         0, 0,    1;
         0, 0,    0];
B_sub = [0; 0; k];

%% 2. PASSO 1: CONTROLLO PRELIMINARE E LYAPUNOV
% =========================================================================
% Stabiliamo il sottosistema (x2, x3, x4)
% Poli desiderati per il sottosistema
% desired_poles = [-5, -6, -7];
desired_poles = [-1, -2, -3];

% Calcolo del guadagno K_sub con Ackermann
K_sub = acker(A_sub, B_sub, desired_poles);
fprintf('Guadagno K_sub calcolato: ');
disp(K_sub); % Dovrebbe essere [0.27, 11, 6] come appunti

% Matrice del sottosistema stabilizzato ("AA")
AA = A_sub - B_sub * K_sub;
fprintf('\nMatrice AA del sottosistema stabilizzato:\n');
disp(AA);

% Calcolo della matrice di Lyapunov P
% Risolviamo AA'*P + P*AA = -I
P = lyap(AA', eye(3));
fprintf('\nMatrice di Lyapunov P:\n');
disp(P);

KK = [0 K_sub];
BB = [0; B_sub];
A4 = vertcat([1 0 0],AA);
A4 = horzcat([0;0;0;0], A4);

%% 3. PASSO 2: TROVARE LA MANIFOLD M(x_a)
% =========================================================================
% Risolviamo il sistema di equazioni algebriche per trovare m = [m2, m3, m4]
% 0.27 * m4 = 1
% -21.80 * m2 + 11 * m4 = 0
% m3 - 6 * m4 = 0
% Nota: Usiamo i valori numerici precisi calcolati da AA e K_sub
% per evitare errori di arrotondamento.
c1 = AA(3,1); % 0.27...
c2 = AA(1,2); % -21.8...
c3 = AA(3,2); % 11
c4 = AA(3,3); % -6

% Creiamo la matrice e il vettore per il sistema lineare C*m' = d
C_matrix = [0,  0,  c1;
            c2, 0,  c3;
            0,  1,  c4];
d_vector = [1; 0; 0];

% Risolviamo per m = [m2, m3, m4]
m = (C_matrix \ d_vector)'; % m deve essere un vettore riga
fprintf('\nVettore m = [m2, m3, m4] calcolato:\n');
disp(m); % Dovrebbe essere [1.87, 22.2222, 3.7037] come appunti

% Il gradiente di M è costante e uguale a m
dM_dx_a = m;


%% Espressione simbolica del controllo u
syms x1 x2 x3 x4 real
syms gamma x1_des real
syms m2 m3 m4 real
syms P11 P12 P13 P22 P23 P33 real

x_a = [x2; x3; x4];
m_sym = [m2, m3, m4];
P_sym = [P11, P12, P13;
         P12, P22, P23;
         P13, P23, P33];

% u_forwarding = -(dL*dV - z*dM)*g_x; scegliamo L = V => dL = 1;
dV_dx_a = 2 * x_a.' * P_sym;      % vettore riga
z = x1 - x1_des - m_sym * x_a;    % scalare
control_vector = gamma * dV_dx_a - z * m_sym;
u_sym = -k * control_vector(3);
% Crea una mappa simbolo → valore
subs_m = [m2,     m3,     m4];
vals_m = [m(1),   m(2),   m(3)];

subs_P = [P11, P12, P13, P22, P23, P33];
vals_P = [P(1,1), P(1,2), P(1,3), P(2,2), P(2,3), P(3,3)];

% Sostituzione completa
u_con_valori = subs(u_sym, [subs_m, subs_P], [vals_m, vals_P]);
u_simplificato = simplify(vpa(u_con_valori, 2));
disp("Controllo u forwarding")
disp(u_con_valori)

u_pre = K_sub*[x1; x2; x3];

% Controllo completo
u_full = simplify(u_pre+u_con_valori);
disp("Controllo completo")
disp(u_full)

%% Osservatore 
O = obsv(A4, C_full);
rango = rank(O);
if(rango == 4)
    disp('(A, C) osservabile')
else
    error('(A, C) non osservabile')
end

epsilon = 1; % Scegli un valore velocità osservatore
L0 = place(A4', C_full', [-10 -12 -15 -18])'; % Calcola L0 con poli scelti
L_oss = (1/epsilon) * L0; % Scala L0 con il parametro di alto guadagno
disp('Matrice L per osservatore ad alto guadagno:')
disp(L_oss)
 

