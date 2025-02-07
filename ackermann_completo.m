% Parametri del sistema
    g = 9.81;  % accelerazione gravitazionale (m/s^2)
    m = 0.45;     % massa (kg)
    Izz = 8e-3;    % kg*m^2
    p = 1;
    k = p/Izz;     % costante di guadagno
    
    % Matrice A e B del sistema
    A = [0 1 0 0;
        0 0, -g/m, 0;
        0 0, 0, 1;
        0 0, 0, 0];
    
    B = [0;0; 0; k];

    
    % Posizioni desiderate degli autovalori (ad esempio, -1, -2, -3)
    desired_poles = [-0.5, -1, -2, -3];
    
    % Calcola il guadagno K che sposta gli autovalori desiderati
    %K = place(A, B, desired_poles)
    % Calcolo del guadagno K con il metodo di Ackermann
    K = acker(A, B, desired_poles)

    % Matriss modificata A - BK
    AA = A - B * K;
    % Verifica che gli autovalori di A_mod abbiano parte reale negativa
    eig(AA)
