%% Kapitel 4.4 - Zustandsrueckfuehrung

clear; clc; close all;

ml = 1;

A = [ 0,      1, 0,      0;
    -9.81*(1 + ml/0.8), 0, 0, 0.009;
    0,      0, 0,      1;
    12.26*ml, 0, 0, -0.009 ];

B = [0; -30.49; 0; 30.49];

Cy = [1 0 0 0;
    0 0 1 0];

Cx = [0 0 1 0];

% Analyse
disp('Eigenwerte der offenen Strecke:')
disp(eig(A))

disp('Rang Steuerbarkeit:')
disp(rank(ctrb(A,B)))

disp('Rang Beobachtbarkeit mit y=[phi;x]:')
disp(rank(obsv(A,Cy)))

% Zustandsrueckfuehrung
pK = [-1.8 -2.1 -2.4 -2.7];
K = place(A, B, pK);

disp('K = ')
disp(K)

disp('Eigenwerte A-BK:')
disp(eig(A-B*K))

% Vorfilter fuer x-Ausgang
V = -1 / (Cx * ((A - B*K) \ B));

disp('V = ')
disp(V)

% Geschlossener Kreis von x_soll nach z
Acl = A - B*K;
Bcl = B*V;

sys_x = ss(Acl, Bcl, Cx, 0);

figure;
step(2.45*sys_x, 20);
grid on;
title('Lineares Modell: x auf 2.45 m');

% Stellmoment fuer Sprung testen
t = 0:0.001:20;
r = 2.45 * ones(size(t));
[y,t,z] = lsim(ss(Acl,Bcl,eye(4),zeros(4,1)), r, t);

M = zeros(size(t));
for i = 1:length(t)
    M(i) = -K*z(i,:).' + V*r(i);
end

figure;
plot(t,M);
grid on;
xlabel('t [s]');
ylabel('M [Nm]');
title('Stellmoment M');
yline(0.4,'--');
yline(-0.4,'--');

figure;
subplot(2,1,1);
plot(t,z(:,3)); grid on;
ylabel('x [m]');
title('Zustandsantwort');

subplot(2,1,2);
plot(t,z(:,1)); grid on;
ylabel('\phi [rad]');
xlabel('t [s]');


%% Beobachterentwurf

pObs = [-7 -8 -9 -10];

H = place(A', Cy', pObs)';

Fobs = A - H*Cy;
E1 = H(:,1);
E2 = H(:,2);
Lobs = B;

disp('H = ')
disp(H)

disp('Fobs = ')
disp(Fobs)

disp('E1 = ')
disp(E1)

disp('E2 = ')
disp(E2)

disp('Lobs = ')
disp(Lobs)

disp('Eigenwerte Beobachterfehlerdynamik:')
disp(eig(A - H*Cy))

% Fuer Simulink State-Space Beobachter:
Aobs = Fobs;
Bobs = [E1 E2 Lobs];
Cobs = eye(4);
Dobs = zeros(4,3);




