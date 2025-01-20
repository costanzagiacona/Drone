function verifica_dV_negativa(A_chiuso, P)
    % Calcolare la matrice A^T P + P A
    A_transpose_P_plus_P_A = A_chiuso' * P + P * A_chiuso;

    % Verifica se la matrice A^T P + P A è definita negativa
    eigenvalues = eig(A_transpose_P_plus_P_A);
    
    disp('Autovalori di A^T P + P A:');
    disp(eigenvalues);
    
    if all(eigenvalues < 0)
        disp('La derivata di Lyapunov è definita negativa.');
    else
        disp('La derivata di Lyapunov NON è definita negativa.');
    end

    % Stampare la matrice A^T P + P A per analisi
    disp('Matrice A^T P + P A:');
    disp(A_transpose_P_plus_P_A);
end
