

%% Parameter
lim_Pos = 2.45; %Limit für Wagenposition
lim_rate = 1.3; % Limit für Rampe

xhat0 = [0, 0, 0, 0];

%% reglerInit.m
disp('Initialisiere blockbasierten LQI-Regler...');

%% Nominelle Masse fuer ersten robusten Entwurf
ml_nom = 1.0;

%% Systemmatrizen
A = [0, 1, 0, 0;
    -9.81*(1 + ml_nom/0.8), 0, 0, 0.009;
    0, 0, 0, 1;
    12.26*ml_nom, 0, 0, -0.009];

B = [0; -30.49; 0; 30.49];

C = [1 0 0 0;
    0 0 1 0];

C_x = [0 0 1 0];

%% LQI-Entwurf
A_aug = [A, zeros(4,1);
    C_x, 0];

B_aug = [B;
    0];

Q_lqi = diag([500, 40, 900, 20, 35]);
R_lqi = 70;

K_lqi = lqr(A_aug, B_aug, Q_lqi, R_lqi);
K_lqi_noI = K_lqi;
K_lqi_noI(5) = 0;

%% Beobachter
%obs_poles = [-25 -28 -31 -34];
obs_poles = [-35 -38 -41 -44];
L_obs = place(A', C', obs_poles)';

%% Beobachter-State-Space-Block
A_obs = A - L_obs*C;
B_obs = [B, L_obs];       % Eingang: [M_sat; phi; x]
C_obs = eye(4);
D_obs = zeros(4,3);

%% Sättigungen und Motorparameter
M_max_total = 0.4;
u_max = 100;
Pmax = 3.56;

%% Referenzformer
v_ref_max = 0.8;

disp('Blockbasierter LQI-Regler initialisiert.');
disp('K_lqi = ');
disp(K_lqi);
disp('L_obs = ');
disp(L_obs);