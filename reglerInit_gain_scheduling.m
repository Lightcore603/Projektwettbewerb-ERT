

%% Parameter
lim_Pos = 2.45; %Limit für Wagenposition
lim_rate = 1.3; % Limit für Rampe

xhat0 = [0, 0, 0, 0];

%% reglerInit.m
disp('Initialisiere blockbasierten LQI-Regler...');

%% Nominelle Masse fuer ersten robusten Entwurf
ml_nom = 3.0;

%% Systemmatrizen
A = [0, 1, 0, 0;
    -9.81*(1 + ml_nom/0.8), 0, 0, 0.009;
    0, 0, 0, 1;
    12.26*ml_nom, 0, 0, -0.009];

B = [0; -30.49; 0; 30.49];

C = [1 0 0 0;
    0 0 1 0];

C_x = [0 0 1 0];

%% ------------------------------------------------------------------------
% Gain Scheduling: K(m_l)
% -------------------------------------------------------------------------

disp('Initialisiere Gain-Scheduling fuer LQR/LQI...');

mass_grid = 1:0.5:5;
n_mass = numel(mass_grid);

B = [0; -30.49; 0; 30.49];

C = [1 0 0 0;
     0 0 1 0];

C_x = [0 0 1 0];

% Deine aktuell gute Beobachterpol-Wahl, ggf. leicht gespreizt
obs_poles = [-25 -28 -31 -34];

% LQI-Gewichte
% Da eta momentan deaktiviert ist, ist Q(5,5) erstmal nicht wichtig.
Q_lqi = diag([2000, 100, 80, 120, 1]);

% Falls dein aktueller Regler mit anderem R gut läuft, nimm deinen Wert.
R_lqi = 2000;

K_schedule = zeros(n_mass, 5);

% Optional schon fuer spaeter:
L_schedule = zeros(4, 2, n_mass);
Aobs_schedule = zeros(4, 4, n_mass);
Bobs_schedule = zeros(4, 3, n_mass);

for i = 1:n_mass
    ml_i = mass_grid(i);

    A_i = [0, 1, 0, 0;
          -9.81*(1 + ml_i/0.8), 0, 0, 0.009;
           0, 0, 0, 1;
           12.26*ml_i, 0, 0, -0.009];

    % Augmentiertes System fuer LQI
    A_aug_i = [A_i, zeros(4,1);
               C_x, 0];

    B_aug_i = [B;
               0];

    K_i = lqr(A_aug_i, B_aug_i, Q_lqi, R_lqi);

    % Integrator erstmal deaktiviert lassen
    K_i(5) = 0;

    K_schedule(i,:) = K_i;

    % Beobachterdaten fuer spaetere Stufe vorbereiten
    L_i = place(A_i', C', obs_poles)';

    L_schedule(:,:,i) = L_i;
    Aobs_schedule(:,:,i) = A_i - L_i*C;
    Bobs_schedule(:,:,i) = [B, L_i];
end

C_obs = eye(4);
D_obs = zeros(4,3);

disp('mass_grid = ');
disp(mass_grid);

disp('K_schedule = ');
disp(K_schedule);

%% Beobachter
obs_poles = [-25 -28 -31 -34];
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