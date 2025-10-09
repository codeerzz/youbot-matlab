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
Kpx=10;
Kpy=10;
Kpz=1;
Kdx=1.5;
Kdy=1.5;
Kdz=0.001;

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