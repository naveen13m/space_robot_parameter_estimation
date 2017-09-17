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
dx=[  0    al(2)/2  al(3)/2  al(4)/2  al(5)/2  al(6)/2  al(7)/2 ];
dy=[  0       0       0       0           0       0       0];
dz=[  0       0       0       0           0       0       0];

%MASS
m=[500; 25; 15; 10; 20; 17; 12];
% g=[0 ; 9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=83.61;  Icyy(1)=83.61; Iczz(1)=83.61;
Icxx(2)=0.05;   Icyy(2)=0.05;  Iczz(2)=2.5;
Icxx(3)=0.03;   Icyy(3)=0.03;  Iczz(3)=1.5;
Icxx(4)=0.02;   Icyy(4)=0.02;  Iczz(4)=1;
Icxx(5)=0.05;   Icyy(5)=0.05;  Iczz(5)=3;
Icxx(6)=0.03;   Icyy(6)=0.03;  Iczz(6)=1.2;
Icxx(7)=0.02;   Icyy(7)=0.02;  Iczz(7)=1;

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