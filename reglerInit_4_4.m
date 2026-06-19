%% reglerInit.m
% Kapitel 4.4 - Zustandsrueckfuehrung mit Beobachter und Sollwertfilter



%% Nominelles lineares Modell fuer ml = 1 kg

ml_nom = 1;

A = [ 0, 1, 0, 0;
     -9.81*(1 + ml_nom/0.8), 0, 0, 0.009;
      0, 0, 0, 1;
      12.26*ml_nom, 0, 0, -0.009 ];

B = [0; -30.49; 0; 30.49];

C_t = [1 0 0 0;
       0 0 1 0];

Cx = [0 0 1 0];

%% Stellgroessen- und Motorparameter

Mmax_total = 0.4;
Pmax = 3.56;
uMax = 100;

%% Zustandsrueckfuehrung

% Vorsichtiger Startwert
% Wenn das zu langsam ist: [-1.6 -1.8 -2.0 -2.2]
% Wenn Constraints verletzt werden: [-1.2 -1.4 -1.6 -1.8]
pK = [-20 -20.1 -20.2 -20.3];

K = place(A, B, pK);

% Statischer Vorfilter fuer Form M = -K*xhat + V*x_soll
% Bei Fehlervektor [phi_hat; phidot_hat; x_hat-x_ref; xdot_hat]
% ist V effektiv gleich K(3) und muss nicht separat verwendet werden.
V = -1 / (Cx * ((A - B*K) \ B));

%% Beobachter

% Beobachter nicht zu schnell machen, sonst wird der Regler hektisch.
%pObs = [-30 -31 -32 -33];
pObs= 3*pK

L_t = place(A', C_t', pObs)';

%% Variablen fuer manuellen Beobachter

A_t = A;
B_t = B;
xhat0 = [0; 0; 0; 0];

%% Optional: Beobachter als State-Space-Block

Aobs = A - L_t*C_t;
Bobs = [L_t(:,1), L_t(:,2), B];
Cobs = eye(4);
Dobs = zeros(4,3);

%% Dynamischer Sollwertfilter / Vorfilter

% Konservativer Startwert:
wRef = 0.45;
zetaRef = 1.0;

numRef = wRef^2;
denRef = [1, 2*zetaRef*wRef, wRef^2];

% Alternative, falls es zu langsam ist:
% wRef = 0.55;
% numRef = wRef^2;
% denRef = [1, 2*zetaRef*wRef, wRef^2];

%% Kontrolle

disp('--- ReglerInit Kapitel 4.4 mit Vorfilter ---')

disp('K = ')
disp(K)

disp('V = ')
disp(V)

disp('K(3) = ')
disp(K(3))

disp('L_t = ')
disp(L_t)

disp('numRef = ')
disp(numRef)

disp('denRef = ')
disp(denRef)

disp('Eigenwerte A-BK:')
disp(eig(A - B*K))

disp('Eigenwerte A-LC:')
disp(eig(A - L_t*C_t))

disp('Rang Steuerbarkeit:')
disp(rank(ctrb(A,B)))

disp('Rang Beobachtbarkeit:')
disp(rank(obsv(A,C_t)))