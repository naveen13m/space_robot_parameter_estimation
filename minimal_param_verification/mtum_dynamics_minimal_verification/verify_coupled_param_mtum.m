% Verifies that the coupled parameters are the minimal parameters by
% constructing the momentum model with two sets of decoupled parameters
% which give the same coupled parameters
clear all; close all; clc;

qr=zeros(6, 1); % Fix an arbitrary pose
thr=[0, pi/6, pi/3]; % Fix arbitrary joint angles
ane=[1; 0; 0];

% Case-1
[n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz, Icxy, Icyz, Iczx]=inputs_case1_3_link();
% [n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz, Icxy, Icyz, Iczx]=inputs_try_set_1();
ee=n;
[Ib_1, Im_1, Ibm_1, Jbe_1, Jme_1, GJM_1, Jg2_1] = Jacobian_vom(qr,thr,n,alp,a,b,bt,dx,dy,dz,m,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, ee, ane);
Ib_1_tilde = Ib_1(4 : 6, 4 : 6) - (Ib_1(4 : 6, 1 : 3) * Ib_1(4 : 6, 1 : 3)) / Ib_1(1, 1);
Ibm_1_tilde = Ibm_1(4 : 6, :) - (Ib_1(4 : 6, 1 : 3) * Ibm_1(1 : 3, :)) / Ib_1(1, 1);
prod_term_1 = inv(Ib_1) * Ibm_1;

% Case-2
[n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz, Icxy, Icyz, Iczx]=inputs_case2_3_link();
% [n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz, Icxy, Icyz, Iczx]=inputs_try_set_2();
[Ib_2, Im_2, Ibm_2, Jbe_2, Jme_2, GJM_2, Jg2_2] = Jacobian_vom(qr,thr,n,alp,a,b,bt,dx,dy,dz,m,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, ee, ane);
Ib_2_tilde = Ib_2(4 : 6, 4 : 6) - (Ib_2(4 : 6, 1 : 3) * Ib_2(4 : 6, 1 : 3)) / Ib_2(1, 1);
Ibm_2_tilde = Ibm_2(4 : 6, :) - (Ib_2(4 : 6, 1 : 3) * Ibm_2(1 : 3, :)) / Ib_2(1, 1);
prod_term_2 = inv(Ib_2) * Ibm_2;

% Matrices of interest
Ib_e_tilde = Ib_2_tilde - Ib_1_tilde
Ibm_e_tilde = Ibm_2_tilde - Ibm_1_tilde
prod_term_e = prod_term_2 - prod_term_1


% xhd=GJM*thetad;
% xbd=pinv(Jbe)*(xhd-Jme*thetad);
% res=[xbd; 0; thetad];