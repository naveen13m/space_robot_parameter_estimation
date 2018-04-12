function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 7-link system
%NO. OF LINKS
add_rw = 0;
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
dx=[  0   0.4    0.7    0.5   0.4   0.3   0.5];
dy=[  0   0      0      0.2   0     0     0.2];
dz=[  0   0      0      0.1   0   0       0.2];
% dx=[  0   0.6    0.4    0.7   0.55   0.45   0.35];
% dy=[  0   0      0      0.4     0      0      0];
% dz=[  0   0      0      0.3     0      0      0];

%MASS
m=[1500; 30; 20; 50; 40; 25; 30];
% g=[0 ; 9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=1000;  Icyy(1)=1000; Iczz(1)=1000;
Icxx(2)=2;   Icyy(2)=1;  Iczz(2)=10;
Icxx(3)=1;   Icyy(3)=1;  Iczz(3)=28;
Icxx(4)=14;   Icyy(4)=20;  Iczz(4)=20;
Icxx(5)=1;   Icyy(5)=2;  Iczz(5)=9;
Icxx(6)=1;   Icyy(6)=1.5;  Iczz(6)=19;
Icxx(7)=6;   Icyy(7)=20;  Iczz(7)=35;

Icxy(1)=25;  Icyz(1)=20.45; Iczx(1)=65.71;
Icxy(2)=0.9;   Icyz(2)=2.65;  Iczx(2)=2.9;
Icxy(3)=1.2;   Icyz(3)=2.75;  Iczx(3)=2.5;
Icxy(4)=4.6;   Icyz(4)=3.1;  Iczx(4)=1.23;
Icxy(5)=1;   Icyz(5)=1.25;  Iczx(5)=6.71;
Icxy(6)=1.5;   Icyz(6)=0.55;  Iczx(6)=4.27;
Icxy(7)=15;   Icyz(7)=18.52;  Iczx(7)=4.67;

if add_rw
	n = n + 4;
	alp = [alp; 0; pi/2; pi/2; pi/2];
	a = [a; -0.3; -0.3; -0.3; 0];
	b = [b; 0; 0; 0; 0];
	bt = [bt, 1 1 1 10];
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

end