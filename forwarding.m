function forwarding()

    g = 9.81;       % m/(s^2),
    m = 0.45;       % kg, peso del drone
    p = 1;
    
    %%% Inerzie
    Izz = 8e-3;    % kg*m^2,

    syms x1 x2 x3 x4 M3(z3, z4) z3 z4 
    % Pitch (su e giù)
    % x1_dot_z = x2;
    % x2_dot_z = 1/m*(-g*x3);
    % x3_dot_z = x4;
    % x4_dot_z = p/Izz*u2;

    % Partiamo dal basso e consideriamo x2 e x3
    % Troviamo la funzione di Lyapunov V(x) -> controllo preliminare per
    % stabilizzare il sistema dx4 e avere forma strict feedforward
    v = 1;
    u2 = Izz/p*(-x4 + v);
    V = (x4^2)/2;   % positiva e radialmente illimitata => dV = -x4^2 => x4=0 GES
    dV = -x4^2;
    % Il teorema ci assicura GAS + LES
    % v = -(dV-(x3-M)*dM);  

    % x4 = -(dM/dx4)*x4
    M4 = -x4;
    dM4 = -1;
    v = -(-x4-(x3-M4)*dM4);

    % Cambio coordinate
    z4 = x4;
    z3 = x3-M4;
    % abbiamo sostituito le x con le z in v
    v = -(z4+z3);

    % SECONDA ITERAZIONE
    % Nelle nuove coordinate
    dz4 = -2*z4-z3;
    dz3 = -z3;% dz3 = dx3 - dM;

    V3 = (z3^2)/2 + (z4^2)/2;
    dV3 = [z3; z4];

    % Calcolo della derivata di M3 rispetto a z3
    f_z3 = @(z3) -3 * g / (2 * m); % f(z3) = -3g/(2m)
    
    % Calcolo di M3(z3) mediante integrazione simbolica
    syms z3 C3
    M3 = int(f_z3(z3), z3) + C3;
    
    % Calcolo della derivata di M4 rispetto a z4
    g_z4 = @(z4) g / (2 * m); % g(z4) = g/(2m)
    
    % Calcolo di M4(z4) mediante integrazione simbolica
    syms z4 C4
    M4 = int(g_z4(z4), z4) + C4;
    
    % Visualizzazione delle soluzioni
    disp('La funzione M3(z3) è:');
    disp(M3);
    
    disp('La funzione M4(z4) è:');
    disp(M4);
end