clc; clear all;

%%
sigma1 = 0.095552102645907;
sigma2 = -4.188591525534890;
sigma3 = -9.560074503602351e+02;
sigma4 = 0.948578347399056;
sigma5 = 0.183951237289056;
sigma6 = 80.615844666612460;

%%
%sigma1 = -sigma1;
%sigma2 = -sigma2;
%sigma3 = -sigma3;
sigma4 = -sigma4;
sigma5 = -sigma5;
sigma6 = -sigma6;
%%
A21 = -1/sigma1;
A22 = -sigma2/sigma1;
A44 = -sigma5/sigma4;
A43 = sigma6/sigma4;
A42 = -sigma2/sigma1/sigma4;
A41 = -1/sigma1/sigma4;


B20 = sigma3/sigma1;
B40 = sigma3/sigma1/sigma4;

x0 = [-2;
      0;
      0.1;
      0];   

A = [0  1   0   0;      %x
    A21 A22 0   0;      %x_dot
    0   0   0   1;      %theta
    A41 A42 A43 A44];   %theta dot

eig(A)

B = [0;
    B20;
    0;
    B40];

S = [B A*B A^2*B A^3*B]; 
R_controllable = rank(S);

Q = [2 0 0 0;
     0 0.11 0 0;
     0 0 0.1 0;
     0 0 0 2];
 R = 0.01;
 
 K = lqr(A,B,Q,R);
 
 C = [0 0 1 0;
      1 0 0 0];
 D = zeros(size(C,1),size(B,2));


%%  Augment system with disturbances and noise
Vd = .1*eye(4); % disturbance covariance
Vn = 1;  %noise covarince(scalar);

BF = [B Vd 0*B]; %augment inputs with disturbance and noise
sysC = ss(A,BF,eye(4),zeros(4,size(BF,2))); % system with full state output, disturbance, no noise

%%  Building Kalman filter
%[Kf,P,E] = lqe(A,Vd,C,Vd,Vn); %design Kalman filter
Kf = (lqr(A',C',Vd,Vn))'; %same as above design with 'LQR' code

sysKF = ss(A-Kf*C,[B,Kf],eye(4),0*[B Kf]);
%%
R_observable = rank([C;
       C*A;
       C*A^2;
       C*A^3]); %Yes!! it is observable

 sys = ss((A - B*K),B,C,D);
 t = 0:0.005:10;
 
 [y,t,x] = initial(sys,x0,t);
 plot(t,y); grid on;
 legend('theta','x');