clear; clc; close all;

m   = 0.468;
Az  = 0.2;
Ar  = 1e-3;
Ixx = 3.357e-5;
Iyy = 4.856e-3;
Izz = 8.801e-3;

G_Z     = tf(1/m,   [1, Az/m,   0]);
G_Phi   = tf(1/Ixx, [1, Ar/Ixx, 0]);
G_Theta = tf(1/Iyy, [1, Ar/Iyy, 0]);
G_Psi   = tf(1/Izz, [1, Ar/Izz, 0]);

Kp_Z     = 0.0309;
Kp_Phi   = 0.011;
Kp_Theta = 7.52e-5;
Kp_Psi   = 4.11e-5;

TF_List    = {G_Z, G_Phi, G_Theta, G_Psi};
Kp_List    = {Kp_Z, Kp_Phi, Kp_Theta, Kp_Psi};
Axis_Names = {'Z (Altitude)', 'Phi (Roll)', 'Theta (Pitch)', 'Psi (Yaw)'};
Time_Limits = {20, 1, 100, 150};
Tp_Ratio    = {1/5, 1/2, 1/20, 1/20};

G_open_comp_List    = cell(4, 1);
Kp_comp_List        = zeros(4, 1);
zc_List             = zeros(4, 1);
T_p_new_plot_List   = zeros(4, 1);

fprintf('*** PD Controller Design Results ***\n');

for i = 1:4
    G_s        = TF_List{i};
    axis_name  = Axis_Names{i};
    Kp_val     = Kp_List{i};
    t_limit    = Time_Limits{i};
    ratio      = Tp_Ratio{i};

    sys_cl_kp = feedback(Kp_val * G_s, 1);
    poles     = pole(sys_cl_kp);

    positive_imag_poles = poles(imag(poles) > 1e-6);

    if isempty(positive_imag_poles)
        dominant_pole = [];
    else
        [~, idx]      = max(real(positive_imag_poles));
        dominant_pole = positive_imag_poles(idx);
    end

    if isempty(dominant_pole)
        fprintf('%s: No oscillatory dominant pole found. Skipping PD design.\n', axis_name);
        continue;
    end

    w_d_uncomp = imag(dominant_pole);
    T_p_uncomp = pi / w_d_uncomp;
    T_p_new    = T_p_uncomp * ratio;
    w_d_new    = pi / T_p_new;
    zeta       = abs(real(dominant_pole)) / abs(dominant_pole);
    theta      = acos(zeta);

    if theta < 1e-4 || all(real(poles) >= 0)
        zeta_target = 0.7;
        theta       = acos(zeta_target);
        sigma_new   = w_d_new / tan(theta);
        P_d_new     = -sigma_new + 1i*w_d_new;
    else
        sigma_new = w_d_new / tan(theta);
        P_d_new   = -sigma_new + 1i*w_d_new;
    end

    [p_open, z_open] = pzmap(G_s);
    angle_poles      = sum(angle(P_d_new - p_open)) * 180/pi;
    angle_zeros      = sum(angle(P_d_new - z_open)) * 180/pi;
    angle_required   = 180 - angle_poles + angle_zeros;

    tan_angle = tand(angle_required);
    if abs(tan_angle) < 1e-4
        zc = sigma_new - w_d_new / tand(1e-3);
    else
        zc = w_d_new / tan_angle + sigma_new;
    end
    zc_val = abs(zc);

    G_PD         = tf([1 zc_val], 1);
    G_open_comp  = G_PD * G_s;
    K_pd         = 1 / abs(evalfr(G_open_comp, P_d_new));

    G_open_comp_List{i}  = G_open_comp;
    Kp_comp_List(i)      = K_pd;
    zc_List(i)           = zc_val;
    T_p_new_plot_List(i) = T_p_new;

    fprintf('Axis: %s (Ratio: 1/%.0f)\n', axis_name, 1/ratio);
    fprintf('  Uncompensated dominant pole: %.4f %+.4fj  (Tp=%.2fs)\n', real(dominant_pole), imag(dominant_pole), T_p_uncomp);
    fprintf('  Target dominant pole:        %.4f %+.4fj  (Tp=%.2fs)\n', real(P_d_new), imag(P_d_new), T_p_new);
    fprintf('  PD compensator zero (zc):    -%.4f\n', zc_val);
    fprintf('  New loop gain (Kp_comp):     %.4g\n', K_pd);
    fprintf('-----------------------------------------------------\n');
end

G_open_PID_List = cell(4, 1);
Kp_PID_List     = zeros(4, 1);

fprintf('\n*** PID Controller Design Results ***\n');

for i = 1:4
    if isempty(G_open_comp_List{i})
        continue;
    end

    G_open_PD  = G_open_comp_List{i};
    zi_val     = 0.01;
    pi_val     = 0;
    G_PI       = tf([1 zi_val], [1 pi_val]);
    G_open_PID = G_open_PD * G_PI;

    T_p_new  = T_p_new_plot_List(i);
    w_d_new  = pi / T_p_new;

    sys_cl_temp    = feedback(Kp_comp_List(i) * G_open_PD, 1);
    poles_temp     = pole(sys_cl_temp);
    P_d_new_complex = poles_temp(imag(poles_temp) > 1e-6);

    if isempty(P_d_new_complex)
        P_d_new = [];
    else
        [~, idx] = min(abs(imag(P_d_new_complex) - w_d_new));
        P_d_new  = P_d_new_complex(idx);
    end

    if isempty(P_d_new) || abs(imag(P_d_new)) < 1e-6
        K_pid = Kp_comp_List(i);
    else
        K_pid = 1 / abs(evalfr(G_open_PID, P_d_new));
    end

    G_open_PID_List{i} = G_open_PID;
    Kp_PID_List(i)     = K_pid;

    fprintf('Axis: %s\n', Axis_Names{i});
    fprintf('  PI zero/pole added: -%.2f / 0\n', zi_val);
    fprintf('  PID loop gain (Kp_PID): %.4g\n', K_pid);
    fprintf('-----------------------------------------------------\n');
end

figure('Name', 'Figure 1: Uncompensated System', 'Position', [50 50 1000 700]);
for i = 1:4
    G_s       = TF_List{i};
    axis_name = Axis_Names{i};
    t_limit   = Time_Limits{i};
    Kp_val    = Kp_List{i};

    subplot(4, 2, 2*i - 1);
    rlocus(G_s);
    title(sprintf('%d. %s - Uncompensated Root Locus', i, axis_name));
    grid on;
    if i == 2
        xlim([-30 1]);
    end

    subplot(4, 2, 2*i);
    sys_cl_kp = feedback(Kp_val * G_s, 1);
    step(sys_cl_kp, t_limit);
    ylim([0 1.2]);
    title(sprintf('%d. %s - Uncompensated Step Response (K=%.4g)', i, axis_name, Kp_val));
    xlabel('Time (s)');
    ylabel('Amplitude');
    grid on;

    if any(real(pole(sys_cl_kp)) >= 0)
        text(t_limit*0.1, 1.2, '!!! UNSTABLE !!!', 'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');
    end
end
fprintf('\nFigure 1: Uncompensated system analysis complete.\n');

figure('Name', 'Figure 2: PD Compensated System', 'Position', [1050 50 1000 700]);
for i = 1:4
    G_open_comp  = G_open_comp_List{i};
    axis_name    = Axis_Names{i};
    Kp_comp      = Kp_comp_List(i);
    zc_val       = zc_List(i);
    T_p_new_plot = T_p_new_plot_List(i);

    if isempty(G_open_comp) || Kp_comp == 0
        subplot(4, 2, 2*i - 1); title(sprintf('%d. %s - PD design failed', i, axis_name));
        subplot(4, 2, 2*i);     title(sprintf('%d. %s - PD design failed', i, axis_name));
        continue;
    end

    t_limit_comp = max(T_p_new_plot * 2.5, 1);

    subplot(4, 2, 2*i - 1);
    rlocus(G_open_comp);
    title(sprintf('%d. %s - PD Root Locus (z_c=-%.4f)', i, axis_name, zc_val));
    grid on;

    sys_cl_comp_temp = feedback(Kp_comp * G_open_comp, 1);
    P_d_new_all      = pole(sys_cl_comp_temp);
    P_d_new_complex  = P_d_new_all(imag(P_d_new_all) > 1e-6);

    if ~isempty(P_d_new_complex)
        w_d_target = pi / T_p_new_plot;
        [~, idx]   = min(abs(imag(P_d_new_complex) - w_d_target));
        P_d_new    = P_d_new_complex(idx);
        hold on;
        plot(real(P_d_new),  imag(P_d_new),  'r*', 'MarkerSize', 8);
        plot(real(P_d_new), -imag(P_d_new),  'r*', 'MarkerSize', 8);
        text(real(P_d_new)+0.05, imag(P_d_new), 'Pd', 'Color', 'red', 'FontSize', 10);
        hold off;
    end

    if i == 2
        xlim([-30 1]);
    end

    subplot(4, 2, 2*i);
    sys_cl_comp = feedback(Kp_comp * G_open_comp, 1);
    step(sys_cl_comp, t_limit_comp);
    ylim([0 1.2]);
    title(sprintf('%d. %s - PD Step Response (K=%.4g)', i, axis_name, Kp_comp));
    xlabel('Time (s)');
    ylabel('Amplitude');
    grid on;

    if any(real(pole(sys_cl_comp)) >= 0)
        text(t_limit_comp*0.1, 1.2, '!!! UNSTABLE !!!', 'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');
    end
end
fprintf('Figure 2: PD compensated system analysis complete.\n');

figure('Name', 'Figure 3: PID Compensated System', 'Position', [50 100 1000 700]);
for i = 1:4
    G_open_PID   = G_open_PID_List{i};
    axis_name    = Axis_Names{i};
    Kp_PID       = Kp_PID_List(i);
    zc_val       = zc_List(i);
    T_p_new_plot = T_p_new_plot_List(i);

    if isempty(G_open_PID) || Kp_PID == 0
        subplot(4, 2, 2*i - 1); title(sprintf('%d. %s - PID design failed', i, axis_name));
        subplot(4, 2, 2*i);     title(sprintf('%d. %s - PID design failed', i, axis_name));
        continue;
    end

    t_limit_pid = max(T_p_new_plot * 2.5, 1);

    subplot(4, 2, 2*i - 1);
    rlocus(G_open_PID);
    title(sprintf('%d. %s - PID Root Locus (z_c=-%.4f, z_i=-0.01, p_i=0)', i, axis_name, zc_val));
    grid on;

    sys_cl_comp_temp = feedback(Kp_PID * G_open_PID, 1);
    poles_pid_all    = pole(sys_cl_comp_temp);
    poles_pid_complex = poles_pid_all(imag(poles_pid_all) > 1e-6);

    if ~isempty(poles_pid_complex)
        w_d_target  = pi / T_p_new_plot;
        [~, idx]    = min(abs(imag(poles_pid_complex) - w_d_target));
        P_d_new_pid = poles_pid_complex(idx);
        hold on;
        plot(real(P_d_new_pid),  imag(P_d_new_pid),  'r*', 'MarkerSize', 8);
        plot(real(P_d_new_pid), -imag(P_d_new_pid),  'r*', 'MarkerSize', 8);
        hold off;
    end

    subplot(4, 2, 2*i);
    sys_cl_pid = feedback(Kp_PID * G_open_PID, 1);
    step(sys_cl_pid, t_limit_pid);
    ylim([0 1.2]);
    title(sprintf('%d. %s - PID Step Response (K=%.4g)', i, axis_name, Kp_PID));
    xlabel('Time (s)');
    ylabel('Amplitude');
    grid on;

    if any(real(pole(sys_cl_pid)) >= 0)
        text(t_limit_pid*0.1, 1.2, '!!! UNSTABLE !!!', 'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');
    end
end
fprintf('Figure 3: PID compensated system analysis complete.\n');