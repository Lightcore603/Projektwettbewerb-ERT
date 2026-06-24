

%% Parameter
lim_Pos = 2.45; %Limit für Wagenposition
lim_rate = 1.3; % Limit für Rampe

xhat0 = [0; 0; 0; 0];


K_index = 1:5;
K_index_const = 1:5;
K_ml_ones = ones(1,5);

%% reglerInit.m
disp('Initialisiere blockbasierten LQI-Regler...');

%% Nominelle Masse fuer ersten robusten Entwurf
ml_nom = 2.0;


%% Gain Scheduling Tabellen fuer Regler und Beobachter

mass_grid = 1:0.25:5;
n_mass = numel(mass_grid);

B = [0; -30.49; 0; 30.49];

C = [1 0 0 0;
    0 0 1 0];

C_x = [0 0 1 0];

Q_lqi = diag([2000, 100, 80, 120, 1]);
R_lqi = 2000;

obs_poles = [-25 -28 -31 -34];

K_schedule = zeros(n_mass, 5);
Aobs_schedule = zeros(n_mass, 16);
Lobs_schedule = zeros(n_mass, 8);

for i = 1:n_mass
    ml_i = mass_grid(i);

    A_i = [0, 1, 0, 0;
        -9.81*(1 + ml_i/0.8), 0, 0, 0.009;
        0, 0, 0, 1;
        12.26*ml_i, 0, 0, -0.009];

    %% LQI / LQR Gain
    A_aug_i = [A_i, zeros(4,1);
        C_x, 0];

    B_aug_i = [B;
        0];

    K_i = lqr(A_aug_i, B_aug_i, Q_lqi, R_lqi);

    % Integrator deaktiviert lassen
    K_i(5) = 0;

    K_schedule(i,:) = K_i;

    %% Beobachter
    L_i = place(A_i', C', obs_poles)';

    %Aobs_i = A_i - L_i*C;
    %Bobs_i = [B, L_i];

    % Als Vektoren speichern, damit Lookup Table sie ausgeben kann
    Aobs_schedule(i,:) = A_i(:).';
    Lobs_schedule(i,:) = L_i(:).';
end

disp('Gain Scheduling Tabellen initialisiert.');

%% Sättigungen und Motorparameter
M_max_total = 0.4;
u_max = 100;
Pmax = 3.56;

%% Referenzformer
v_ref_max = 0.8;

disp('K_schedule = ');
disp(K_schedule);

disp('Lobs_schedule = ');
disp(Lobs_schedule);