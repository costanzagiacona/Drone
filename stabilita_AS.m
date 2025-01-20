
function A_chiuso = stabilita_AS(g, m)
    % Calcolo dei parametri del sistema
    k = 2*(g/m) - 3/2*(g^2/m^2);
    c = g/m;

    % Matrice A originale
    A = [0, k, c;
         0, -1, 0;
         0, -1, -2];

    % Matrice B: retroazione sullo stato z2
    B = [0; 1; 1];

    % Calcolare la matrice di controllabilità
    C = ctrb(A, B);

    % Verificare la controllabilità
    if rank(C) == size(A, 1)
        disp('Il sistema è controllabile');
    else
        disp('Il sistema non è controllabile');
        return;
    end

    % Definire i poli desiderati
    poli_desiderati = [-1, -2, -3];  

    % Calcolare la matrice di guadagno K usando il metodo di Ackermann
    K = acker(A, B, poli_desiderati);

    % Mostrare il risultato
    disp('Matrice di guadagno K:');
    disp(K);

    % Matrice A chiusa in retroazione
    A_chiuso = A - B * K

    % Verificare la stabilità della matrice A chiusa
    autovalori = eig(A_chiuso);
    disp('Autovalori del sistema in retroazione:');
    disp(autovalori);

    % Controllare se tutti gli autovalori hanno parte reale negativa
    if all(real(autovalori) < 0)
        disp('Il sistema in retroazione è stabile.');
    else
        disp('Il sistema in retroazione non è stabile.');
    end
end
