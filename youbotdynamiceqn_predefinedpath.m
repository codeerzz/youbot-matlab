clc;
run('Forward.m');
%%
ti=0; tf=10;
n=200;
dt=(tf-ti)/n;

%t=[ti:dt:tf]';
T_total=10;
m=20;
I_z=0.4606;
L=0.23;
l=0.167;
Ll=L+l;
r=0.05;
I_w=0.0002;
I_w1=0.0002;
I_w2=0.0002;
I_w3=0.0002;
I_w4=0.0002;
phi=pi/4;
phi_dot=1;
initX=0;
initY=0;
initAngle=0;

%Kontrol kazanç katsayıları

%Kp=[0.5 0.5 0.5];
%Kd=[0.5 0.5 0.5];
Kpx=1.5;
Kpy=1.5;
Kpz=2.5;
Kdx=1.5;
Kdy=1.5;
Kdz=2.5;

% Gövdeye ait kütle matrisi (Mr)
M_r = [m  0  0;
       0  m  0;
       0  0  I_z]; % Mr: robot gövdesine ait atalet matrisi


% Teker atalet matrisi (Mw)
M_w = diag([I_w1, I_w2, I_w3, I_w4]); 

% Köşegen bir matris (diagonal matrix)
M = [M_r, zeros(3,4);  % Üst blok: Mr ve 0 matris
     zeros(4,3), M_w]; % Alt blok: 0 matris ve Mw


% R matrisi tanımı
R = (1/r) * [  1, -1, -(L+l);
               1,  1,  (L+l);
               1,  1, -(L+l);
               1, -1,  (L+l)];

% R matrisi transpoz
R_T=transpose(R);

sim youbot_predefinedpath_2103

%% 
% 1) timeseries -> double dizilere dönüştür [Nx1]
xPlotRef = squeeze(x_ref.Data);      
yPlotRef = squeeze(y_ref.Data);      
xPlotAct = squeeze(x_actual.Data);   
yPlotAct = squeeze(y_actual.Data);   
    

% 2) X-Y Grafiği oluştur
figure('Name','X-Y Grafiği','NumberTitle','off');
plot(xPlotRef, yPlotRef, 'b-', 'LineWidth', 2); 
hold on;
plot(xPlotAct, yPlotAct, 'r--', 'LineWidth', 2);

% 3) Grafik ayarları
axis equal;             
grid on;               
xlabel('X (m)');
ylabel('Y (m)');
title('Konum (X-Y Grafiği)');
legend('x_{ref}, y_{ref}','x_{actual}, y_{actual}');
hold off;

%%
%Hata grafikleri 

% 1) timeseries -> double
ex = squeeze(e_x.Data);
ey = squeeze(e_y.Data);
eth = squeeze(e_theta.Data);
evx = squeeze(e_Vx.Data);
evy = squeeze(e_Vy.Data);
ethd = squeeze(e_thetad.Data);

t = e_x.Time;  % Zaman vektörü 

% 2) Plot

figure;
subplot(2,1,1);
plot(t, ex, t, ey, t, eth, 'LineWidth', 2);
legend('e_x','e_y','e_\theta','Location','best');
grid on;
title('Position Errors');
xlabel('Time (s)');
ylabel('Error');

subplot(2,1,2);
plot(t, evx, t, evy, t, ethd, 'LineWidth', 2);
legend('e_{Vx}','e_{Vy}','e_{\theta d}','Location','best');
grid on;
title('Velocity Errors');
xlabel('Time (s)');
ylabel('Error');


