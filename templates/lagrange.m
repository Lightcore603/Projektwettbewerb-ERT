%Skript zum Berechnen der Lagrangeschen Gleichungen 2. Art
clear 
clc
%Variablen 
syms l ms ml g phi dphi ddphi x dx ddx F k kr k_phi dphi 'real'
%verallgemeinterte Koordinaten x und phi
%nicht konservative Kräfte:
Q_x = -kr*dx+F;
Q_phi = -k_phi*dphi; %fuer die Aufgabe: k_phi = 0

%Potentielle Energie:
U_pendel = -ml*g*l*cos(phi);

%Kinetische Energie:
T_pendel = 1/2*ml*dx^2+ml*dx*dphi*l*cos(phi)+1/2*dphi^2*ml*l^2;
T_schlitten = 1/2*ms*dx^2;

%Lagrange Funktion:
L = T_pendel+T_schlitten-U_pendel;

%Ableitungen
L_dphi = diff (L,dphi);
L_phi = diff(L,phi);

L_dx = diff(L,dx);
L_x = diff(L,x);

syms x(t) dphi(t) dx(t) phi(t)
L_dphi = subs(L_dphi,phi);
L_dphi = subs(L_dphi,[dx,dphi],[dx(t),dphi(t)]);
dL_dphi = subs(diff(L_dphi,t),[diff(phi(t),t),diff(dphi(t),t),diff(x(t),t),diff(dx(t),t)],[dphi,ddphi,dx,ddx]);

L_dx = subs(L_dx,phi);
L_dx = subs(L_dx,[dx,dphi],[dx(t),dphi(t)]);
dL_dx = subs(diff(L_dx,t),[diff(phi(t),t),diff(dphi(t),t),diff(x(t),t),diff(dx(t),t)],[dphi,ddphi,dx,ddx]);

syms x_n dphi_n dx_n phi_n
x(t) = x_n; phi(t) = phi_n; dphi(t) = dphi_n; dx(t) = dx_n;
dL_dphi = subs(dL_dphi);
dL_dx = subs(dL_dx);

syms x phi dx dphi
x_n = x; phi_n = phi; dx_n = dx; dphi_n = dphi;
dL_dphi = subs(dL_dphi);
dL_dx = subs(dL_dx);

%Equations of motion:
eqns = [dL_dphi-L_phi == Q_phi; dL_dx-L_x == Q_x];
save('pendulum_model.mat','eqns')