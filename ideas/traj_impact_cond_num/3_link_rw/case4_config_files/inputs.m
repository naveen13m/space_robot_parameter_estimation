% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx add_rw]=inputs()

% 3 Link Manipulator
%NO. OF LINKS
add_rw = 1;
n=3;

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
dx=[  0    0.5    0.5 ];
dy=[  0       0       0     ];
dz=[  0       0       0     ];


%MASS
m=[1000; 50; 30];
% g=[0 ; -9.81];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
% Icxx(1)=(1/12)*0.01*0.01;   Icyy(1)=(1/12)*m(1)*(al(1)*al(1)+0.1*0.1);  Iczz(1)=(1/12)*m(1)*al(1)*al(1);
% Icxx(2)=(1/12)*0.01*0.01;   Icyy(2)=(1/12)*m(2)*al(2)*al(2);  Iczz(2)=(1/12)*m(2)*al(2)*al(2);
% Icxx(3)=(1/12)*0.01*0.01;   Icyy(3)=(1/12)*m(3)*al(3)*al(3);  Iczz(3)=(1/12)*m(3)*al(3)*al(3);
% Icxx(4)=(1/12)*0.01*0.01;   Icyy(4)=(1/12)*m(4)*al(4)*al(4);  Iczz(4)=(1/12)*m(4)*al(4)*al(4);

Icxx(1)=500;  Icyy(1)=500; Iczz(1)=500;
Icxx(2)=20;   Icyy(2)=20;  Iczz(2)=20;
Icxx(3)=10;   Icyy(3)=10;  Iczz(3)=10;

% Kinematic and dynamic parameters of MRW
if add_rw
	n = n + 1;
	alp(n) = 0;
	a(n) = -0.5;
	b(n) = 0;
	bt(n) = 1;
	al(n) = 0.5;
	alt(n) = 0.25;
	dx(n) = 0;
	dy(n) = 0;
	dz(n) = 0;
	m(n) = 20;
	Icxx(n) = 10;
	Icyy(n) = 10;
	Iczz(n) = 10;
	Icxy(n) = 0;
	Icyz(n) = 0;
	Iczx(n) = 0;
end

% % NTUA Verification params
% % Set-2a
% m=[503.899639; 109.7057987; 90.15216512];
% dx=[  0    0.7695643366  0.6453894074];
% Icxx(1)=1205.894768;  Icyy(1)=1205.894768; Iczz(1)=1205.894768;
% Icxx(2)=44.43355118;   Icyy(2)=44.43355118;  Iczz(2)=44.43355118;
% Icxx(3)=40.17615466;   Icyy(3)=40.17615466;  Iczz(3)=40.17615466;

% % Set-2b
% m=[159.7134012; 266.8389805; 345.1412672];
% dx=[  0    0.9699719946  0.5832108791];
% Icxx(1)=1210.005403;  Icyy(1)=1210.005403; Iczz(1)=1210.005403;
% Icxx(2)=32.75090948;   Icyy(2)=32.75090948;  Iczz(2)=32.75090948;
% Icxx(3)=8.027031368;   Icyy(3)=8.027031368;  Iczz(3)=8.027031368;
