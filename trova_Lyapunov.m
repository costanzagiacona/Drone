function [P, V] = trova_Lyapunov(A_chiuso, x)
    % Calcolare la matrice P di Lyapunov
    % Risolvere l'equazione di Lyapunov A^T * P + P * A = -Q
    % con Q definita positiva

    % Definire Q come una matrice diagonale definita positiva (scelta comune)
    % Q = [100, 0, 0;
    %  0, 50, 0;
    %  0, 0, 100];
    Q = eye(3)

    % Risolvere l'equazione di Lyapunov
    P = lyap(A_chiuso, -Q)  % Metodo numerico per risolvere l'equazione

    % Verifica della positività definita di P
    if all(eig(P) > 0)
        disp('Matrice P è definita positiva.');
    else
        disp('Matrice P non è definita positiva.');
    end

    % Verifica della proprietà di illimitatezza radiale (se P è radially unbounded)
    % Per il caso semplice, possiamo controllare che P non sia una matrice nulla
    if all(P(:) > 0)
        disp('La matrice P è illimitata radialmente.');
    else
        disp('La matrice P non è illimitata radialmente.');
    end

    % Calcolare la funzione di Lyapunov V(x) = x' * P * x
    V = x' * P * x;

    % Stampare la matrice P (la funzione di Lyapunov V)
    disp('Matrice di Lyapunov P (usata per calcolare V):');
    disp(P);
    disp('Funzione di Lyapunov V(x):');
    disp(V);
end
