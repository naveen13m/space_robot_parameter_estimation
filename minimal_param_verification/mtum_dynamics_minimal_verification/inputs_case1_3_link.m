function [n, nq, alp, a, b, bt, dx, dy, dz, al, alt, m, g,  Icxx, Icyy, Iczz, Icxy, Icyz, Iczx]=inputs_case1_3_link()

load('sols.mat');
x = actual_params;
%NO. OF LINKS
n=3;
red_params = 1;

nq=0;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; 0];
a=[0; 0.5; 1];
b=[0; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2];

%Link Length
al=[1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1];

%ENTER VECTOR dm
dx=[  0      x(7)  x(11)];
dy=[  0       0       0     ];
dz=[  0       0       0     ];


%MASS
m=[x(2); x(6); x(10)];
% g=[0 ; -9.81];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
% Icxx(1)=(1/12)*0.01*0.01;   Icyy(1)=(1/12)*m(1)*(al(1)*al(1)+0.1*0.1);  Iczz(1)=(1/12)*m(1)*al(1)*al(1);
% Icxx(2)=(1/12)*0.01*0.01;   Icyy(2)=(1/12)*m(2)*al(2)*al(2);  Iczz(2)=(1/12)*m(2)*al(2)*al(2);
% Icxx(3)=(1/12)*0.01*0.01;   Icyy(3)=(1/12)*m(3)*al(3)*al(3);  Iczz(3)=(1/12)*m(3)*al(3)*al(3);
% Icxx(4)=(1/12)*0.01*0.01;   Icyy(4)=(1/12)*m(4)*al(4)*al(4);  Iczz(4)=(1/12)*m(4)*al(4)*al(4);

Icxx(1)=x(1);  Icyy(1)=x(1); Iczz(1)=x(1);
Icxx(2)=x(5);   Icyy(2)=x(5);  Iczz(2)=x(5);
Icxx(3)=x(9);   Icyy(3)=x(9);  Iczz(3)=x(9);

if red_params == 1
    M = sum(m);
    m = m / M;
    Icxx = Icxx/M;
    Icyy = Icyy/M;
    Iczz = Iczz/M;
end