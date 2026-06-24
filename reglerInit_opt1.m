

%% Parameter
lim_Pos = 2.45;     % Limit fuer Wagenposition
lim_rate = 1.3;     % kann erstmal bleiben, wird aber vom neuen Referenzformer ersetzt

xhat0 = [0; 0; 0; 0];

%% Abtastzeit fuer diskreten Referenzformer
Ts_ref = 0.01;   % 10 ms

%% Mechanische Parameter
m_s = 0.8;
g = 9.81;
r_wheel = 0.041;
k_r = 7.2e-3;

%% Kontinuierlicher Referenzfilter 3. Ordnung
omega_ref = 2.0;   % Startwert: sicher
% omega_ref = 1.30; % etwas schneller
% omega_ref = 1.50; % aggressiver

A_traj = [0, 1, 0;
          0, 0, 1;
         -omega_ref^3, -3*omega_ref^2, -3*omega_ref];

B_traj = [0;
          0;
          omega_ref^3];

C_traj = eye(3);

D_traj = zeros(3,1);

xtraj0 = [0; 0; 0];   % [x_ref(0); v_ref(0); a_ref(0)]
%% Momentenvorsteuerung
ff_gain = 1;
M_ff_max = 0.34;

%% reglerInit.m
disp('Initialisiere blockbasierten LQI-Regler...');

%% Nominelle Masse fuer ersten robusten Entwurf
ml_nom = 2.0;

%% Systemmatrizen
A = [0, 1, 0, 0;
    -9.81*(1 + ml_nom/0.8), 0, 0, 0.009;
    0, 0, 0, 1;
    12.26*ml_nom, 0, 0, -0.009];

B = [0; -30.49; 0; 30.49];

C = [1 0 0 0;
    0 0 1 0];


%% LQR-Entwurf
% Zustand: [phi; phi_dot; x; x_dot]
%Q_lqr = diag([700, 70, 900, 35]);
Q_lqr = diag([750, 95, 2200, 105]);
R_lqr = 32;



%% Beobachterpole
%obs_poles = [-25 -28 -31 -34];
obs_poles = [-35 -38 -41 -44];
L_obs = place(A', C', obs_poles)';

%% Gain Scheduling Raster
% 17 Punkte ist ein guter Kompromiss: fein genug, aber noch simpel.
ml_grid = linspace(1, 5, 17);     % 1.00, 1.25, ..., 5.00
N_ml = numel(ml_grid);

K_grid = zeros(N_ml, 4);          % jede Zeile: K_lqr fuer eine Masse
L_grid = zeros(N_ml, 8);          % jede Zeile: reshape(L_obs,1,8)
A_grid = zeros(N_ml, 16);         % jede Zeile: reshape(A,1,16)

for i = 1:N_ml
    ml_i = ml_grid(i);

    A_i = [0, 1, 0, 0;
        -9.81*(1 + ml_i/0.8), 0, 0, 0.009;
        0, 0, 0, 1;
        12.26*ml_i, 0, 0, -0.009];

    K_i = lqr(A_i, B, Q_lqr, R_lqr);

    L_i = place(A_i', C', obs_poles)';

    A_grid(i,:) = reshape(A_i, 1, 16);
    K_grid(i,:) = K_i;
    L_grid(i,:) = reshape(L_i, 1, 8);
end

%% Nominelle Werte fuer Anzeige / Fallback
K_lqr = lqr(A, B, Q_lqr, R_lqr);
L_obs = place(A', C', obs_poles)';

%% Sättigungen und Motorparameter
M_max_total = 0.4;
u_max = 100;
Pmax = 3.56;


disp('Blockbasierter LQ-Regler initialisiert.');
disp('K_lqr = ');
disp(K_lqr);
disp('L_obs = ');
disp(L_obs);