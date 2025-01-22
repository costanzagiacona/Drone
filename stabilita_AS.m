
function A_mod = stabilita_AS(g, m)
    % Calcolo dei parametri del sistema
    k = 2*(g/m) - 3/2*(g^2/m^2);
    c = g/m;

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

    % Parametri del sistema
    g = 9.81;  % accelerazione gravitazionale (m/s^2)
    m = 0.45;     % massa (kg)
    k = 1;     % costante di guadagno
    
    % Matrice A e B del sistema
    A = [0, -g/m, 0;
         0, 0, 1;
         0, 0, k];
    
    B = [0; 0; k];
    
    % Posizioni desiderate degli autovalori (ad esempio, -1, -2, -3)
    desired_poles = [-1, -2, -3];
    
    % Calcola il guadagno K che sposta gli autovalori desiderati
    K = place(A, B, desired_poles);
    
    % Matriss modificata A - BK
    A_mod = A - B * K;
    
    % Verifica che gli autovalori di A_mod abbiano parte reale negativa
    eig(A_mod)

end
