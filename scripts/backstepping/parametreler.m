clear all; clc;

g   = 9.81;
L   = 0.225;
m   = 0.468;
Jr  = 3.357e-5;

Ixx = 4.856e-3;
Iyy = 4.856e-3;
Izz = 8.801e-3;

a1     = (Iyy - Izz) / Ixx;
a2     = Jr / Ixx;
b1     = L / Ixx;
b1_inv = 1 / b1;

a3     = (Izz - Ixx) / Iyy;
a4     = Jr / Iyy;
b2     = L / Iyy;
b2_inv = 1 / b2;

a5     = (Ixx - Iyy) / Izz;
b3     = 1 / Izz;
b3_inv = 1 / b3;

P_roll  = 6.0;
K1 = sqrt(P_roll);
K2 = sqrt(P_roll);

P_pitch = 5.0;
K3 = sqrt(P_pitch);
K4 = sqrt(P_pitch);

P_yaw = 0.3;
K5 = sqrt(P_yaw);
K6 = sqrt(P_yaw);

P_z = 0.3437;
K7 = sqrt(P_z);
K8 = sqrt(P_z);

P_pos = 0.7;
K9  = sqrt(P_pos);
K10 = sqrt(P_pos);
K11 = sqrt(P_pos);
K12 = sqrt(P_pos);

fprintf('K1: %.4f  K2: %.4f\n', K1, K2);
fprintf('K3: %.4f  K4: %.4f\n', K3, K4);
fprintf('K5: %.4f  K6: %.4f\n', K5, K6);
fprintf('K7: %.4f  K8: %.4f\n', K7, K8);
fprintf('K9..12:   %.4f\n',     K11);