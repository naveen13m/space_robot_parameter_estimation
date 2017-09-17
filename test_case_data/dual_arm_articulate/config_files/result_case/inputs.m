function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 7-link system
%NO. OF LINKS
add_rw = 1;
n=7;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; pi/2; 0; 0; pi/2; 0];
a=[0; 0.5; 1; 1; -0.5; 1; 1];
b=[0; 0.5; 0; 0; 0.5; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3 1 5 6];

%Link Length
al=[1; 1; 1; 1; 1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1; 1; 1; 1; 1];

%ENTER VECTOR dm
% dx=[  0   0.6    0.4    0.7   0.55   0.45   0.35];
% dy=[  0   0.05  -0.04   0.4   0.04   0.05  -0.05];
% dz=[  0  -0.07  -0.05   0.3  -0.04   0.05  -0.05];
dx=[  0   0.6    0.4    0.7   0.55   0.45   0.35];
dy=[  0   0      0      0.4     0      0      0];
dz=[  0   0      0      0.3     0      0      0];

%MASS
m=[2000; 50; 50; 50; 50; 50; 50];
% g=[0 ; 9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=1200;  Icyy(1)=1200; Iczz(1)=1200;
Icxx(2)=2.18;   Icyy(2)=1.89;  Iczz(2)=20.51;
Icxx(3)=1.15;   Icyy(3)=1.68;  Iczz(3)=18.67;
Icxx(4)=24.45;   Icyy(4)=28.56;  Iczz(4)=35.53;
Icxx(5)=1.85;   Icyy(5)=1.62;  Iczz(5)=17.05;
Icxx(6)=1.55;   Icyy(6)=1.84;  Iczz(6)=14.28;
Icxx(7)=1.24;   Icyy(7)=1.45;  Iczz(7)=10.77;

Icxy(1)=35.52;  Icyz(1)=40.45; Iczx(1)=45.71;
Icxy(2)=1.9;   Icyz(2)=1.65;  Iczx(2)=2.5;
Icxy(3)=1.61;   Icyz(3)=1.75;  Iczx(3)=1.5;
Icxy(4)=6.78;   Icyz(4)=9.1;  Iczx(4)=10.23;
Icxy(5)=1.5;   Icyz(5)=1.35;  Iczx(5)=3.21;
Icxy(6)=1.23;   Icyz(6)=1.55;  Iczx(6)=1.27;
Icxy(7)=1.1;   Icyz(7)=1.52;  Iczx(7)=1.67;

if add_rw
	n = n + 1;
	alp(n) = 0;
	a(n) = -0.3;
	b(n) = 0;
	bt(n) = 1;
	al(n) = 0.3;
	alt(n) = 0.15;
	dx(n) = 0;
	dy(n) = 0;
	dz(n) = 0;
	m(n) = 20;
	Icxx(n) = 0.2;
	Icyy(n) = 0.2;
	Iczz(n) = 10;
	Icxy(n) = 0;
	Icyz(n) = 0;
	Iczx(n) = 0;
end

end