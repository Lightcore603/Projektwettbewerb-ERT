
s = tf('s');

%% Parameter
ml = 1;

%% Strecke fuer aktive Schwingungsdaempfung
G_phi = -(1/0.041)*s / ...
    (0.8*s^3 + 0.0072*s^2 + 9.81*(0.8 + ml)*s + 0.071);
G_phi = minreal(G_phi);

sisotool(G_phi)

%% Neuer Winkelregler: PI + Lead
K_phi = -1.2*(1 + 4/s)*((s/4 + 1)/(s/20 + 1));

K_phi = minreal(K_phi);