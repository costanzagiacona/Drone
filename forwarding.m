function forwarding()

    g = 9.81;       % m/(s^2),
    m = 0.45;       % kg, peso del drone
    p = 1;
    
    %%% Inerzie
    Izz = 8e-3;    % kg*m^2,

    syms x1 x2 x3 x4 M3_z3 M3_z4 z1 z2 z3 z4 
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
    % z4 = x4;
    % z3 = x3-M4;
    % abbiamo sostituito le x con le z in v
    v = -(z4+z3);

    % SECONDA ITERAZIONE
    % Nelle nuove coordinate
    dz4 = -2*z4-z3;
    dz3 = -z3;% dz3 = dx3 - dM;

    V3 = (z3^2)/2 + (z4^2)/2;
    dV3 = [z3, z4];
    
    syms  m1 m2 M2
 
    % Definizione dell'equazione
    % lhs = (g/m)*(z4 - z3);
    % rhs = -m1*z3 - m2*(2*z4 + z3);
    lhs = (g/m)*z4 == -2*m2*z4;
    rhs = -(g/m)*z3 == - m1*z3 - m2*z3;
     
    % Risolvere il sistema di equazioni per m1 e m2
    % eq = lhs == rhs;
     
    % Risolvi il sistema per m1 e m2
    sol = solve([lhs, rhs], [m1, m2]);
     
    % Mostra le soluzioni
    disp('La soluzione per m1(z) è:');
    disp(sol.m1);
     
    disp('La soluzione per m2(z) è:');
    disp(sol.m2);

    dM3 = [sol.m1 sol.m2];

    % Integriamo per trovare M3(z3) ed M3(z4)
    M3_z3 = int(sol.m1,z3);
    M3_z4 = int(sol.m2,z4);

    disp('M3 è:');
    disp(M3_z3);
    disp(M3_z4);

    g2 = [1; 0];
    u3 = -(dV3 - z3*dM3)*g2;

    A = stabilita_AS(g, m);

    % x = [x1; x2; x3];
    % % Calcolare la matrice di Lyapunov P
    % [P, V] = trova_Lyapunov(A, x);
    % 
    % % Verificare che la derivata di Lyapunov sia definita negativa
    % verifica_dV_negativa(A, P);

    P = solveSylvester(A);
    disp(P);
    disp(eig(P))

    z = [z2; z3; z4];
    V2 = simplify(z'*P*z);
    dV2 = z2*diff(V2,z2) + z3*diff(V2, z3) + z4*diff(V2, z4);

    k = 669.2600;
 
    a = 0.0045;
     
    b = 4.0489;
     
    c = 21.800;
    
    d = 0.0489;
     
    e = 1.9511;
     

    % Condizioni iniziali (z2, z3, z4)
    z0 = [1; 0; 0]; % Esempio di condizioni iniziali (z2(0), z3(0), z4(0))
     
    % Intervallo di tempo
    tspan = [0 5]; % Tempo da 0 a 5 secondi
     
    % Definizione dell'equazione differenziale
    ode = @(t, z) [
        (-k*z(2) + c*z(3)) * (z(1) + (3*g/(2*m))*z(2) - (g/(2*m))*z(3));
        (a*z(1) - b*z(2) + d*z(3)) * (z(1) + (3*g/(2*m))*z(2) - (g/(2*m))*z(3));
        (a*z(1) - b*z(2) - e*z(3)) * (z(1) + (3*g/(2*m))*z(2) - (g/(2*m))*z(3));
    ];
     
    % Risolvere il sistema di equazioni differenziali
    [t, z] = ode45(ode, tspan, z0);
     
    % Estrazione delle soluzioni per z2, z3, z4
    z2 = z(:,1);
    z3 = z(:,2);
    z4 = z(:,3);
     
    % Calcolare le derivate dM2/dz2, dM2/dz3, dM2/dz4
    dM2_dz2 = (-k * z3 + c * z4) .* (z2 + (3*g/(2*m)) * z3 - (g/(2*m)) * z4);
    dM2_dz3 = (a * z2 - b * z3 + d * z4) .* (z2 + (3*g/(2*m)) * z3 - (g/(2*m)) * z4);
    dM2_dz4 = (a * z2 - b * z3 - e * z4) .* (z2 + (3*g/(2*m)) * z3 - (g/(2*m)) * z4);
     
end