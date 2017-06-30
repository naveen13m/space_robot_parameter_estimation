% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 7 Link Manipulator
%NO. OF LINK
n=8;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; pi/2; pi/2; 0; 0; pi/2; pi/2];
a=[  0; 0;    0;   0 ;     0.87;   0.63; 0;    0.55];
b=[  0; 0.35; 0.2; 0.275; -0.275; -0.36; 0.16; 0.2];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3 4 5 6 7];

%Link Length
al=[0.7; 0.35; 0.2; 0.87; 0.63; 0.36; 0.55; 0.532];
% %Distance from origin to link tip in term of link length
alt=[0.35; 0.35; 0.2; 0.87; 0.63; 0.36; 0.55; 0.532];

%ENTER VECTOR dm
ax=[ 0   a(3)              a(4)              a(5)              a(6)              a(7)              a(8)              0.532];
ay=[ 0  -b(3)*sin(alp(3)) -b(4)*sin(alp(4)) -b(5)*sin(alp(5)) -b(6)*sin(alp(6)) -b(7)*sin(alp(7)) -b(8)*sin(alp(8))  0];
az=[ 0  b(3)*cos(alp(3))  b(4)*cos(alp(4))  b(5)*cos(alp(5))  b(6)*cos(alp(6))  b(7)*cos(alp(7))  b(8)*cos(alp(8))  0];
dx=ax/2;
dy=ay/2;
dz=az/2;

%MASS
m=[1000; 35.01; 30; 22.69; 21.38; 16.75; 26.17; 26.17];
% g=[0 ; -9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=1200;  Icyy(1)=1200; Iczz(1)=1200;
Icxx(2)=1.218;   Icyy(2)=0.5132;  Iczz(2)=1.331;
Icxx(3)=2.10;   Icyy(3)=1.378;  Iczz(3)=2.359;
Icxx(4)=0.102;   Icyy(4)=3.378;  Iczz(4)=3.359;
Icxx(5)=0.4327;  Icyy(5)=2.266; Iczz(5)=1.911;
Icxx(6)=0.3837;   Icyy(6)=0.3936;  Iczz(6)=0.07271;
Icxx(7)=0.5727;   Icyy(7)=0.5987;  Iczz(7)=0.1288;
Icxx(8)=0.5727;   Icyy(8)=0.5987;  Iczz(8)=0.1288;