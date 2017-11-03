function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

% 7 Link Manipulator
%NO. OF LINKS
add_rw = 1;
n=5;
nq=0;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; 0; 0; 0];
a=[0; 0.198; 0.14827; -0.198; 0.14889];
b=[0; 0; 0; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2 1 4];

%Link Length
al=[0.396; 0.14827; 0.09656; 0.14889; 0.09622];
% %Distance from origin to link tip in term of link length
alt=[0.198; 1; 1; 1; 1];

%ENTER VECTOR dm
dx=[  0  0.0772 0.04729 0.06513 0.04849];
dy=[  0       0       0       0           0  ];
dz=[  0       0       0       0           0  ];

%MASS
m=[7; 0.323; 0.233; 0.323 ; 0.233];
g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=0;   Icyy(1)=0;  Iczz(1)=0.03;
Icxx(2)=0;   Icyy(2)=0;  Iczz(2)=6.53387e-4;
Icxx(3)=0;   Icyy(3)=0;  Iczz(3)=2.22926e-4;
Icxx(4)=0;   Icyy(4)=0;  Iczz(4)=6.53387e-4;
Icxx(5)=0;   Icyy(5)=0;  Iczz(5)=2.22926e-4;

if add_rw
	n = n + 1;
	alp = [alp; 0];
	a = [a; 0];
	b = [b; 0];
	bt = [bt, 1];
	al = [al; 0.1];
	alt = [alt; 0.05];
	dx = [dx, 0];
	dy = [dy, 0];
	dz = [dz, 0];
	m = [m; 0.63];
	Icxx = [Icxx; 0];
	Icyy = [Icyy; 0];
	Iczz = [Iczz; 3.90233e-4];
	Icxy = [Icxy; 0];
	Icyz = [Icyz; 0];
	Iczx = [Iczx; 0];
end

end