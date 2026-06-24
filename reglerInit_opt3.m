

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

%% Adaptiver kontinuierlicher Referenzfilter
% Grundidee:
% große Wege: moderat
% mittlere Wege: schneller
% kleine Wege: sehr schnell, damit kurze Referenzsprünge nicht unnötig Zeit kosten

omega_ref_large  = 2.0;   % große Sprünge
omega_ref_medium = 3.3;   % mittlere Sprünge
omega_ref_small  = 3.3;   % kleine Sprünge

d_large  = 1.30;          % ab hier große Sprünge
d_small  = 0.45;          % darunter kleine Sprünge

% Sicherheitsreduktion bei Pendelausschlag
phi_slow_1 = deg2rad(28); % ab hier etwas vorsichtiger
phi_slow_2 = deg2rad(36); % ab hier deutlich vorsichtiger

% Begrenzungen für Trajektorie
a_ref_lim = 3.8;          % maximale Referenzbeschleunigung
j_ref_lim = 20.0;         % maximale Referenzjerk

% Initialzustand des Referenzfilters
x_ref0 = 0;
v_ref0 = 0;
a_ref0 = 0;

%% Partielle Pendelreferenz
gamma_phi = 0.30;         % 0 = aus, 1 = volle -a/g-Referenz
phi_ref_lim = deg2rad(18);
phidot_ref_lim = 0.8;     % rad/s

% Vorzeichen bei Bedarf testen:
% Wenn Pendel stärker aufschaukelt, gamma_phi = -0.20 testen.
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


%% LQR-Entwurf Drive/Capture
% Zustand: [phi; phi_dot; x; x_dot]
%Q_lqr = diag([700, 70, 900, 35]);
Q_lqr = diag([750, 95, 2200, 105]);
R_lqr = 32;

d_capture_start = 0.10;   % ab hier beginnt weicher Übergang in Capture
d_capture_full  = 0.03;   % ab hier vollständig Capture

% Drive: aggressiv fahren
Q_drive = diag([750, 95, 2200, 105]);
R_drive = 1;

% Capture: nahe am Ziel schneller beruhigen/einfangen
Q_capture = diag([1100, 150, 2200, 150]);
R_capture = 1;

%% Beobachterpole
%obs_poles = [-25 -28 -31 -34];
obs_poles = [-35 -38 -41 -44];

%% Gain Scheduling Raster
% 17 Punkte ist ein guter Kompromiss: fein genug, aber noch simpel.
ml_grid = linspace(1, 5, 17);     % 1.00, 1.25, ..., 5.00
N_ml = numel(ml_grid);

K_drive_grid   = zeros(N_ml, 4);
K_capture_grid = zeros(N_ml, 4);  % jede Zeile: K_lqr fuer eine Masse

L_grid = zeros(N_ml, 8);          % jede Zeile: reshape(L_obs,1,8)
A_grid = zeros(N_ml, 16);         % jede Zeile: reshape(A,1,16)

for i = 1:N_ml
    ml_i = ml_grid(i);

    A_i = [0, 1, 0, 0;
        -9.81*(1 + ml_i/0.8), 0, 0, 0.009;
        0, 0, 0, 1;
        12.26*ml_i, 0, 0, -0.009];

    K_drive_i   = lqr(A_i, B, Q_drive, R_drive);
    K_capture_i = lqr(A_i, B, Q_capture, R_capture);

    L_i = place(A_i', C', obs_poles)';

    A_grid(i,:) = reshape(A_i, 1, 16);

    K_drive_grid(i,:)   = K_drive_i;
    K_capture_grid(i,:) = K_capture_i;

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