function dX = osservatore(t,X)

global c1 c2 k omegad1 d1

mi = 0.8;

dX = zeros(4,1);
%(X = [x1,x2,x1_hat,x2_hat]) ; z1 = x1_hat,z2 = x2_hat
x1 = X(1);
x2 = X(2);
x1_hat = X(3);
x2_hat = X(4);

%dinamica del processo
dx1 = x2;
dx2 = mi*(1-x1^2)*x2 - x1; 
y = x1 + d1*sin(omegad1*t);

%dinamica dell'osservatore 
y_hat = x1_hat; %x1_hat
dx1_hat = x2_hat  + c1*k*(y-y_hat) ;
dx2_hat =   mi*(1-x1_hat^2)*x2_hat - x1_hat    + c2*k^2*(y-y_hat); 

dX = [dx1;dx2;dx1_hat;dx2_hat];