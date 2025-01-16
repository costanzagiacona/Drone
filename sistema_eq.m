% Funzione che definisce il sistema di equazioni differenziali
% M3 derivata in funzione di z3 e z4
% Funzione che definisce il sistema di equazioni differenziali
% function dzdt = sistema_eq(t, z)
% 
%     g = 9.81;       % m/(s^2),
%     m = 0.45;       % kg, peso del drone
% 
%     z3 = z(1);    % z3
%     z4 = z(2);    % z4
%     M3_z3 = z(3); % M3 in funzione di z3
%     M3_z4 = z(4); % M3 in funzione di z4
%     % Equazioni differenziali
%     dM3dz3 = -(g/m)*(z4 - z3) - (M3_z4 * (2*z4 + z3)); 
%     dM3dz4 = -(g/m)*(z4 - z3) - (M3_z3 * z3);
%     % Restituisci le derivate come un vettore
%     dzdt = [dM3dz3; dM3dz4; M3_z3; M3_z4];
% end

function dzdt = sistema_eq(t, z)

    g = 9.81;       % m/(s^2),
    m = 0.45;       % kg, peso del drone
    % Definire le variabili z3, z4 e M3
    z3 = z(1);    % z3
    z4 = z(2);    % z4
    M3 = z(3);    % M3
    % Derivata di M3 rispetto a z3 e z4 (da definire)
    dM3dz3 = -(g/m)*(z4 - z3) - (M3 * z3);  % Relazione per la derivata di M3
    dM3dz4 = -(g/m)*(z4 - z3) - (M3 * (2*z4 + z3));  % Relazione per la derivata di M3
    % Derivate per l'integrazione del sistema
    dzdt = [dM3dz3; dM3dz4; M3];  % Restituisci le derivate come un vettore
end