function P = solveSylvester(A)
    % A è la matrice di input
    % Risolviamo l'equazione A^T * P + P * A = -I per P
    % Calcoliamo la trasposta di A
    A_T = A';
    % La matrice identità I (dimensione di A)
    I = eye(size(A));
    % Risolviamo l'equazione di Sylvester: A^T * P + P * A = -I
    P = sylvester(A_T, A, -I);
end