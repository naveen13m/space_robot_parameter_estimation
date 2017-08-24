% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 7 Link Manipulator
%NO. OF LINK
n=4;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; pi/3; pi/4; -pi/6];
a=[  0; 0.5; 1; 1];
b=[  0; 0; 0.5; 1];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3];

%Link Length
al=[1; 1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1; 1];

%ENTER VECTOR dm
ax=[ 0  1 1 1];
ay=[ 0  0 0 0];
az=[ 0  0 0 0];
dx=ax/2;
dy=ay/2;
dz=az/2;

%MASS
m=[1000; 50; 30; 100];
% g=[0 ; -9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=1200;  Icyy(1)=1200; Iczz(1)=1200;
Icxx(2)=10;   Icyy(2)=1;  Iczz(2)=10;
Icxx(3)=10;   Icyy(3)=120;  Iczz(3)=10;
Icxx(4)=50;   Icyy(4)=50;  Iczz(4)=50;