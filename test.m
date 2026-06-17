s = tf('s');

ml = 1;
D = 0.8*s^3 + 0.0072*s^2 + 9.81*(0.8+ml)*s + 0.071;

Gphi = (-s/D) * (1/0.041);
Gx   = ((s^2 + 9.81)/(s*D)) * (1/0.041);

bode(Gx)
grid on

figure
nyquist(Gx)
grid on
%%
sisotool(Gx)
%%

Kx = 11106*(s + 0.023)/(s + 2937);

T_x = feedback(Kx*Gx,1);

step(T_x)
grid on