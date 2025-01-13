function forwarding(h_x, x_dot)

    % Definizione delle variabili simboliche
    syms x h(x) F(x) G(x) m(x)
     
    % Equazione: h(x) - m(x) * F(x) = 0
    % Risolviamo per m(x)
    eq1 = h(x) - m(x) * F(x);
    m_solution = solve(eq1 == 0, m(x)); % m(x) = h(x) / F(x)
     
    % Condizione: m(0) * G(0) ≠ 0
    m_at_0 = subs(m_solution, x, 0); % Calcola m(0)
    G_at_0 = subs(G(x), x, 0); % Calcola G(0)
    condition = m_at_0 * G_at_0; % Calcola il prodotto m(0) * G(0)
     
    % Verifica della condizione
    if condition == 0
        error('La condizione m(0) * G(0) ≠ 0 non è soddisfatta!');
    else
        disp('La condizione m(0) * G(0) ≠ 0 è soddisfatta.');
    end
     
    % Visualizzazione della soluzione
    disp('Soluzione per m(x):');
    disp(m_solution);
     
    disp('Condizione m(0) * G(0):');
    disp(condition);

    m1 = h_x * x1_dot;
    m2 = h_x * x2_dot;
    m3 = h_x * x3_dot;
    m4 = h_x * x4_dot;
    m = [m1; m2; m3; m4];

end