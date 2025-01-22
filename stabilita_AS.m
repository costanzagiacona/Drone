
%function A_mod = stabilita_AS(g, m)
    % Calcolo dei parametri del sistema
    % k = 2*(g/m) - 3/2*(g^2/m^2);
    % c = g/m;

    % % Matrice A originale
    % A = [0, k, c;
    %      0, -1, 0;
    %      0, -1, -2];
    % 
    % % Matrice B: retroazione sullo stato z2
    % B = [0; 1; 1];
    % 
    % % Calcolare la matrice di controllabilità
    % C = ctrb(A, B);
    % 
    % % Verificare la controllabilità
    % if rank(C) == size(A, 1)
    %     disp('Il sistema è controllabile');
    % else
    %     disp('Il sistema non è controllabile');
    %     return;
    % end
    % 
    % % Definire i poli desiderati
    % poli_desiderati = [-1, -2, -3];  
    % 
    % % Calcolare la matrice di guadagno K usando il metodo di Ackermann
    % K = acker(A, B, poli_desiderati);
    % 
    % % Mostrare il risultato
    % disp('Matrice di guadagno K:');
    % disp(K);
    % 
    % % Matrice A chiusa in retroazione
    % A_chiuso = A - B * K
    % 
    % % Verificare la stabilità della matrice A chiusa
    % autovalori = eig(A_chiuso);
    % disp('Autovalori del sistema in retroazione:');
    % disp(autovalori);
    % 
    % % Controllare se tutti gli autovalori hanno parte reale negativa
    % if all(real(autovalori) < 0)
    %     disp('Il sistema in retroazione è stabile.');
    % else
    %     disp('Il sistema in retroazione non è stabile.');
    % end

    syms x2 x3 x4
    % Parametri del sistema
    g = 9.81;  % accelerazione gravitazionale (m/s^2)
    m = 0.45;     % massa (kg)
    Izz = 8e-3;    % kg*m^2
    p = 1;
    k = p/Izz;     % costante di guadagno
    
    % Matrice A e B del sistema
    A = [0, -g/m, 0;
         0, 0, 1;
         0, 0, 0];
    
    B = [0; 0; k];
    
    % Posizioni desiderate degli autovalori (ad esempio, -1, -2, -3)
    desired_poles = [-1, -2, -3];
    
    % Calcola il guadagno K che sposta gli autovalori desiderati
    %K = place(A, B, desired_poles)
    % Calcolo del guadagno K con il metodo di Ackermann
    K = acker(A, B, desired_poles)

    
    % Matriss modificata A - BK
    AA = A - B * K
    
    % Verifica che gli autovalori di A_mod abbiano parte reale negativa
    eig(AA)

    % Definizione della matrice identità I
    I = eye(size(A));

    % Risolvere l'equazione di Lyapunov per P
    %P = lyap(AA, -I);
    AAt = transpose(AA);
    P = sylvester(AAt, AA, -I);

    % Verifica che P sia simmetrica e definita positiva
    disp('Matrice P:');
    disp(P);
    disp('Verifica P = P'' (simmetrica):');
    disp(isequal(P, P'));

    % Verifica che P sia definita positiva
    disp('Autovalori di P (devono essere positivi):');
    disp(eig(P));

    %Troviamo V
    V = [x2, x3, x4]*P*transpose([x2, x3, x4]) 
    
    % Derivata rispetto a x2
    dV_dx2 = diff(V, x2);

    % Derivata rispetto a x3
    dV_dx3 = diff(V, x3);
    
    % Derivata rispetto a x4
    dV_dx4 = diff(V, x4);
    
    % Visualizzazione dei risultati
    % disp('Derivata di V rispetto a x2:');
    % disp(dV_dx2);
    % 
    % disp('Derivata di V rispetto a x3:');
    % disp(dV_dx3);
    % 
    % disp('Derivata di V rispetto a x4:');
    % disp(dV_dx4);
    % %

    dV = [dV_dx2 , dV_dx3 , dV_dx4]

    %% Trovo m(x)
    % Definiamo le variabili simboliche
    syms m2 m3 m4;
    
    % Equazioni
    eq1 = 0.24 * m4 == 1;               % 0.24*m4 = 1
    eq2 = -21.80 * m2 + 11 == 0;        % -21.80*m2 + 11 = 0
    eq3 = m3 - 6 * m4 == 0;             % m3 - 6*m4 = 0
    
    % Risolviamo il sistema di equazioni
    sol = solve([eq1, eq2, eq3], [m2, m3, m4]);
    
    % Mostriamo i risultati
    disp('Risultati:')
    fprintf('m2 = %.4f\n', sol.m2);
    fprintf('m3 = %.4f\n', sol.m3);
    fprintf('m4 = %.4f\n', sol.m4);
    
    %% Trovo M
    syms x2 x3 x4;
    m2 = sol.m2;
    m3 = sol.m3;
    m4 = sol.m4;
    m = [m2 , m3 , m4]

    % Calcola gli integrali per ciascuna funzione
    % int_m1 = integral(m2, 0, x2);
    % int_m2 = integral(m3, 0, x3);
    % int_m3 = integral(m4, 0, x4);

    M = m2*(x2/2) + m3*(x3/2) + m4*(x4/2);

    %% Definizione parametri
    dM = [m2 , m3 , m4]

    eD = dM - m

    %% Trovo ||f(x)||^2 serve per calcolo dL*dV*f + 1/2 * norm_f per trovare alpha
        % Definiamo le variabili simboliche
    syms x2 x3 x4;
    
    % Funzione f(x)
    f = [-21.80*x3; x4; 0.27*x2 + 11*x3 - 6*x4];
    
    % Calcoliamo ||f(x)||^2 come prodotto scalare f(x)^T * f(x)
    norm_f = f' * f;
    
    % Mostriamo il risultato
    % disp('||f(x)||^2 =');
    % disp(norm_f);

    %% calcolo L(V)
    syms mu
    alpha = 534,81;
    L = (alpha + mu)*V;
    dL = (alpha + mu);

    g_x = [0 0 1]';
    z = -dM*g_x

    r = 1;

    u_forwarding = -(dL*dV - z*dM)*g_x



    
