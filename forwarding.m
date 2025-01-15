function forwarding(h_x, dx)

    syms x1 x2 x3 x4
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
    M = -x4;
    dM = -1;
    v = -(dV-(x3-M)*dM);

    % Cambio coordinate
    z4 = x4;
    z3 = x3-M;
    % abbiamo sostituito le x con le z in v
    v = -(z4+z3);

    % Nelle nuove coordinate
    dz4 = -z4+v;
    dz3 = z4 + 1;% dz3 = dx3 - dM;

end