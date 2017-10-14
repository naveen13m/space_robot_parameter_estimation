function [n nq alp a b bt dx dy dz al alt m g  Iaxx Iayy Iazz Iaxy Iayz Iazx]=inputs_sym()

% 7-link system
syms ax0 ax1  ax2  ax3  ax4   ax5  ax6  ay0  ay1  ay2  ay3  ay4   ay5  ay6 ...
    az0 az1  az2  az3  az4   az5  az6 m0  m1  m2  m3  m4  m5 m6 ...
    Iaxx0  Iayy0 Iazz0  Iaxx1  Iayy1  Iazz1  Iaxx2  Iayy2  Iazz2 ...
    Iaxx3  Iayy3  Iazz3 Iaxx4  Iayy4  Iazz4  Iaxx5  Iayy5  Iazz5 ...
    Iaxx6  Iayy6  Iazz6 Iaxy0  Iayz0 Iazx0 Iaxy1   Iayz1  Iazx1 ...
    Iaxy2   Iayz2  Iazx2 Iaxy3  Iayz3  Iazx3 Iaxy4   Iayz4  Iazx4 ...
    Iaxy5   Iayz5  Iazx5 Iaxy6  Iayz6  Iazx6 al

%NO. OF LINKS
add_rw = 0;
n=7;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
alp=[0; 0; al; 0; 0; al; 0];
a=[0; 0.5; 1; 1; -0.5; 1; 1];
b=[0; 0.5; 0; 0; 0.5; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3 1 5 6];

%Link Length
al=[1; 1; 1; 1; 1; 1; 1];
% %Distance from origin to link tip in term of link length
alt=[0.5; 1; 1; 1; 1; 1; 1];

%ENTER VECTOR dm
dx=[  ax0   ax1  ax2  ax3  ax4   ax5  ax6];
dy=[  ay0   ay1  ay2  ay3  ay4   ay5  ay6];
dz=[  az0   az1  az2  az3  az4   az5  az6];
% dx=[  0   0.6    0.4    0.7   0.55   0.45   0.35];
% dy=[  0   0      0      0.4     0      0      0];
% dz=[  0   0      0      0.3     0      0      0];

%MASS
m=[m0; m1; m2; m3; m4; m5; m6];
% g=[0 ; 9.81; 0];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Iaxx=sym(zeros(n,1));Iayy=sym(zeros(n,1));Iazz=sym(zeros(n,1)); % Initialization 
Iaxy=sym(zeros(n,1));Iayz=sym(zeros(n,1));Iazx=sym(zeros(n,1)); % Initialization 
Iaxx(1)=Iaxx0;  Iayy(1)=Iayy0; Iazz(1)=Iazz0;
Iaxx(2)=Iaxx1;   Iayy(2)=Iayy1;  Iazz(2)=Iazz1;
Iaxx(3)=Iaxx2;   Iayy(3)=Iayy2;  Iazz(3)=Iazz2;
Iaxx(4)=Iaxx3;   Iayy(4)=Iayy3;  Iazz(4)=Iazz3;
Iaxx(5)=Iaxx4;   Iayy(5)=Iayy4;  Iazz(5)=Iazz4;
Iaxx(6)=Iaxx5;   Iayy(6)=Iayy5;  Iazz(6)=Iazz5;
Iaxx(7)=Iaxx6;   Iayy(7)=Iayy6;  Iazz(7)=Iazz6;

Iaxy(1)=Iaxy0;  Iayz(1)=Iayz0; Iazx(1)=Iazx0;
Iaxy(2)=Iaxy1;   Iayz(2)=Iayz1;  Iazx(2)=Iazx1;
Iaxy(3)=Iaxy2;   Iayz(3)=Iayz2;  Iazx(3)=Iazx2;
Iaxy(4)=Iaxy3;   Iayz(4)=Iayz3;  Iazx(4)=Iazx3;
Iaxy(5)=Iaxy4;   Iayz(5)=Iayz4;  Iazx(5)=Iazx4;
Iaxy(6)=Iaxy5;   Iayz(6)=Iayz5;  Iazx(6)=Iazx5;
Iaxy(7)=Iaxy6;   Iayz(7)=Iayz6;  Iazx(7)=Iazx6;

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
	Iaxx = [Iaxx; 1; 1; 0; 1];
	Iayy = [Iayy; 1; 1; 0; 1];
	Iazz = [Iazz; 2; 2; 0; 2];
	Iaxy = [Iaxy; 0; 0; 0; 0];
	Iayz = [Iayz; 0; 0; 0; 0];
	Iazx = [Iazx; 0; 0; 0; 0];
end

end