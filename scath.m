% Timeseries verilerini çek
x_ref_vec     = x_ref.Data(:,1);
y_ref_vec     = y_ref.Data(:,1);
w_ref_vec     = w_ref.Data(:,1);
x_actual_vec  = x_actual.Data(:,1);
y_actual_vec  = y_actual.Data(:,1);
w_actual_vec     = y_actual.Data(:,1);
t_vec         = x_ref.Time; % Tüm time seriesler aynı zaman vektörüne sahip olmalı
x_vel_ref_vec = x_vel_ref.Data(:,1);
y_vel_ref_vec = y_vel_ref.Data(:,1);
w_vel_ref_vec = w_wel_ref.Data(:,1);  % HATALI: "w_wel_ref" yanlış, düzelttim.
x_vel_act_vec = x_vel_act.Data(:);
y_vel_act_vec = y_vel_act.Data(:);
w_vel_act_vec = w_wel_act.Data(:);  % HATALI: "w_wel_act" yanlış, düzelttim.

% e_x ve e_y hesapla
e_x = x_ref_vec - x_actual_vec;
e_y = y_ref_vec - y_actual_vec;
e_w = w_ref_vec - w_actual_vec;

N = length(t_vec);
skip = round(N/500);

figure;

% --- ANİMASYON (1. subplot) ---
subplot(3,2,1);
hold on; axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
title('Animasyon: Referans Yörünge Üzerinde Gerçek ve Referans Araba');
plot(x_ref_vec, y_ref_vec, 'k--', 'LineWidth', 2); % referans yörünge
ref_car = plot(NaN, NaN, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
actual_trace = plot(NaN, NaN, 'b-', 'LineWidth', 2);
act_car = plot(NaN, NaN, 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
legend({'Referans Yörünge', 'Referans', 'Gerçek İz', 'Gerçek'}, 'Location', 'eastoutside');
xlim([-10 10]); ylim([-10 10]);

% --- e_x(t) (2. subplot) ---
subplot(3,2,3);
hold on; grid on;
yline(0, '--k', 'LineWidth', 1); % Sıfır referans çizgisi
xlabel('Zaman (s)'); ylabel('e_x (m)');
title('Hata (e_x) Zaman Grafiği');
ex_line = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
ylim([-1.5 1.5]);

% --- e_y(t) (3. subplot) ---
subplot(3,2,5);
hold on; grid on;
yline(0, '--k', 'LineWidth', 1); % Sıfır referans çizgisi
xlabel('Zaman (s)'); ylabel('e_y (m)');
title('Hata (e_y) Zaman Grafiği');
ey_line = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
ylim([-1.5 1.5]);

% --- x velocity (4. subplot) ---
subplot(3,2,2);
hold on; grid on;
xv_ref_line = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
xv_act_line = plot(NaN, NaN, 'b-',  'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('x\_vel (m/s)');
title('x ekseni hızı');
legend({'Referans', 'Gerçek'}, 'Location', 'best');
ylim([-1.5 1.5]);

% --- y velocity (5. subplot) ---
subplot(3,2,4);
hold on; grid on;
yv_ref_line = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
yv_act_line = plot(NaN, NaN, 'b-',  'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('y\_vel (m/s)');
title('y ekseni hızı');
legend({'Referans', 'Gerçek'}, 'Location', 'best');
ylim([-1.5 1.5]);

% --- w velocity (6. subplot) ---
subplot(3,2,6);
hold on; grid on;
wv_ref_line = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
wv_act_line = plot(NaN, NaN, 'b-',  'LineWidth', 1.5);
xlabel('Zaman (s)');
ylabel('\omega (rad/s)');
title('Açısal hız');
legend({'Referans', 'Gerçek'}, 'Location', 'best');
ylim([-3 3]);

for i = 1:skip:N
    % --- Animasyon (1. subplot) ---
    subplot(3,2,1);
    set(ref_car,      'XData', x_ref_vec(i),   'YData', y_ref_vec(i));
    set(actual_trace, 'XData', x_actual_vec(1:i), 'YData', y_actual_vec(1:i));
    set(act_car,      'XData', x_actual_vec(i),   'YData', y_actual_vec(i));

    % --- e_x(t) (2. subplot) ---
    subplot(3,2,3);
    set(ex_line, 'XData', t_vec(1:i), 'YData', e_x(1:i));
    
    % --- e_y(t) (3. subplot) ---
    subplot(3,2,5);
    set(ey_line, 'XData', t_vec(1:i), 'YData', e_w(1:i));


    % --- x velocity (4. subplot) ---
    subplot(3,2,2);
    set(xv_ref_line, 'XData', t_vec(1:i), 'YData', x_vel_ref_vec(1:i));
    set(xv_act_line, 'XData', t_vec(1:i), 'YData', x_vel_act_vec(1:i));

    % --- y velocity (5. subplot) ---
    subplot(3,2,4);
    set(yv_ref_line, 'XData', t_vec(1:i), 'YData', y_vel_ref_vec(1:i));
    set(yv_act_line, 'XData', t_vec(1:i), 'YData', y_vel_act_vec(1:i));

    % --- w velocity (6. subplot) ---
    subplot(3,2,6);
    set(wv_ref_line, 'XData', t_vec(1:i), 'YData', w_vel_ref_vec(1:i));
    set(wv_act_line, 'XData', t_vec(1:i), 'YData', w_vel_act_vec(1:i));

    drawnow;
end
