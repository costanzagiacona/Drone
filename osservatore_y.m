function dXX = osservatore_y(tt,XX)

    global c1 c2 k omegad1 d1 xtheta xtheta_hat
    % Parametri
    g = 9.81;       % m/(s^2),
    m = 0.45;       % kg, peso del drone
    l = 0.23;       % m, Distanza dell'elica dal centro di massa
    kk = 7.5e-7;     % Coefficiente di portanza delle eliche
    b = 3e-6;       % Coefficiente di resistenza aereodinamica
    p = 1;

    %%% Inerzie
    % I_M = 3.357e-5;     % kg*m^2, Inertia Moment of the rotor
    Ir = 6e-5;     % kg*m^2, Inertia of Gyroscopic Effect
    Ixx = 5e-3;    % kg*m^2,
    Iyy = 5e-3;    % kg*m^2,
    Izz = 8e-3;    % kg*m^2,
    I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

    dXX = zeros(4,1);
    %(X = [x1,x2,x1_hat,x2_hat]) ; z1 = x1_hat,z2 = x2_hat
    x1 = XX(1);
    x2 = XX(2);
    x1_hat = XX(3);
    x2_hat = XX(4);
    
    % dinamica del processo
    % Pitch (su e giù)     
    x1_dot_z = x2;
    x2_dot_z = 1/m*(-g*xtheta);
    
    x_dot_z = [x1_dot_z; x2_dot_z];

    % uscita con rumore
    y = x1 + d1*sin(omegad1*tt);

    % dinamica dell'osservatore 
    y_hat = x1_hat;
    x1_dot_hat = x2_hat + c1*k*(y-y_hat);
    x2_dot_hat = 1/m*(-g*xtheta_hat) + c2*k^2*(y-y_hat);
  
    dXX = [x1_dot_z; x2_dot_z; x1_dot_hat; x2_dot_hat];

end