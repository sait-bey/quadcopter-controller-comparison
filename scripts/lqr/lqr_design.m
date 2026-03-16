clear; clc;

m   = 0.468;
Az  = 0.2;
Ar  = 1e-3;
Ixx = 3.357e-5;
Iyy = 4.856e-3;
Izz = 8.801e-3;
g   = 9.81;

A = zeros(8, 8);
B = zeros(8, 4);

A(1, 5) = 1;
A(2, 6) = 1;
A(3, 7) = 1;
A(4, 8) = 1;

A(5, 5) = -Az/m;
A(6, 6) = -Ar/Ixx;
A(7, 7) = -Ar/Iyy;
A(8, 8) = -Ar/Izz;

B(5, 1) = 1/m;
B(6, 2) = 1/Ixx;
B(7, 3) = 1/Iyy;
B(8, 4) = 1/Izz;

Q = diag([5, 10, 10, 5, 5, 5, 5, 5]);
R = diag([10, 1, 1, 100]);

[K_lqr, S, P] = lqr(A, B, Q, R);

fprintf('LQR Gain Matrix K (4x8):\n');
disp(K_lqr);