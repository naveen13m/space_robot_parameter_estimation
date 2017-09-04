% ReDySim sys_ode module contains ODE of system under study
% Contributors of the native code: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
% Code is modified to meet the requirements of this project

function dy=sys_ode(t, y, tf, tr_par, num_intervals_each_joint)

%Model parameter
[n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz,...
    Icxy, Icyz, Iczx]=inputs();

q=y(1:6);
dq=y(6+1:2*6);
% disp(t);

% Trajectories
[th_d, dth_d, ddth_d]=trajectory(t, n, tf, tr_par, num_intervals_each_joint);

% % C+Tug- TERM USING INVERSE DYNAMIC ALGORITHM
th=[0;th_d];
dth=[0;dth_d];
ddth=[0;ddth_d];
[tu_th, ddq] = invdyn_float(q, dq, th, dth,ddth, n,alp,a,b,bt,dx,dy,dz,al,alt, m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx);

%Accelerations
dy=zeros(2*6+1,1);
dy(1:6)=dq;
dy(6+1:2*6)=ddq;

% derive of joint energy
dy(2*6+1)=-tu_th'*dth(2:n);
%%%%%%%%%%%%%%%%%%% hopon.m ends %%%%%%%%%%%%%%%%%%%%
