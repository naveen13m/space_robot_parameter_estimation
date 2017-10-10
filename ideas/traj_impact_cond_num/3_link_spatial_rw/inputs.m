% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx add_rw]=inputs()

% 4-Link spatial system
%NO. OF LINK
add_rw = 0;
n=3;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; pi/2];
a=[  0; 0.5; 1];
b=[  0; 0;   0];
%Parent array bt and corrosponding vectors
bt=[0 1 2];

%Link Length
al=[1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1];

%ENTER VECTOR dm
ax=[ 0  1 1];
ay=[ 0  0 0];
az=[ 0  0 0];
dx=ax/2;
dy=ay/2;
dz=az/2;

%MASS
m=[500; 10; 10];
% g=[0 ; -9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=83.61;  Icyy(1)=83.61; Iczz(1)=83.61;
Icxx(2)=0.1;   Icyy(2)=2.05;  Iczz(2)=2.05;
Icxx(3)=0.1;   Icyy(3)=1.5;  Iczz(3)=1.5;

% Kinematic and dynamic parameters of MRW
if add_rw
	n = n + 4;
	alp = [alp; 0; pi/2; pi/2; pi/2];
	a = [a; -0.3; -0.3; -0.3; 0];
	b = [b; 0; 0; 0; 0];
	bt = [bt, 1 1 1 6];
	al = [al; 0.3; 0.3; 0; 0.3];
	alt = [alt; 0.15; 0.15; 0; 0.15];
	dx = [dx, 0, 0, 0, 0];
	dy = [dy, 0, 0, 0, 0];
	dz = [dz, 0, 0, 0, 0];
	m = [m; 20; 20; 0; 20];
	Icxx = [Icxx; 1; 1; 0; 1];
	Icyy = [Icyy; 1; 1; 0; 1];
	Iczz = [Iczz; 2; 2; 0; 2];
	Icxy = [Icxy; 0; 0; 0; 0];
	Icyz = [Icyz; 0; 0; 0; 0];
	Iczx = [Iczx; 0; 0; 0; 0];
end