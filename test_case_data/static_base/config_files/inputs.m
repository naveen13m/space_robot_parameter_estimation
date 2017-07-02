% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 3 Link Manipulator
%NO. OF LINKS
n=4;

nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; pi/2; pi/2; pi/2];
a=[0; 0.5; 1; 1];
b=[0; 0.5; 1; 1];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3];

%Link Length
al=[1; 1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1; 1];

%ENTER VECTOR dm
ax=[ 0   a(3)              a(4)             1];
ay=[ 0  -b(3)*sin(alp(3)) -b(4)*sin(alp(4)) 0];
az=[ 0  b(3)*cos(alp(3))  b(4)*cos(alp(4))  0];
dx=ax/2;
dy=ay/2;
dz=az/2;

%MASS
m=[500000; 1; 1; 1];
% g=[0 ; -9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
% Icxx(1)=(1/12)*0.01*0.01;   Icyy(1)=(1/12)*m(1)*(al(1)*al(1)+0.1*0.1);  Iczz(1)=(1/12)*m(1)*al(1)*al(1);
% Icxx(2)=(1/12)*0.01*0.01;   Icyy(2)=(1/12)*m(2)*al(2)*al(2);  Iczz(2)=(1/12)*m(2)*al(2)*al(2);
% Icxx(3)=(1/12)*0.01*0.01;   Icyy(3)=(1/12)*m(3)*al(3)*al(3);  Iczz(3)=(1/12)*m(3)*al(3)*al(3);
% Icxx(4)=(1/12)*0.01*0.01;   Icyy(4)=(1/12)*m(4)*al(4)*al(4);  Iczz(4)=(1/12)*m(4)*al(4)*al(4);

Icxx(1)=830000.61;  Icyy(1)=830000.61; Iczz(1)=830000.61;
Icxx(2)=0.01;   Icyy(2)=0.01;  Iczz(2)=0.01;
Icxx(3)=0.01;   Icyy(3)=0.01;  Iczz(3)=0.01;
Icxx(4)=0.01;   Icyy(4)=0.01;  Iczz(4)=0.01;
