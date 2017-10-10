function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs()

load('sols.mat');
x_ac = actual_params;
x = alt_params;
%NO. OF LINKS
n=3;
red_params = 0;

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
dx=[  0      0.268631851403449  0.339092387372478];
dy=[  0       0       0     ];
dz=[  0       0       0     ];


%MASS
m=[1000; 51.273813858150240; 73.726220361668340];
% g=[0 ; -9.81];
 g=[0 ; 0; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
% Icxx(1)=(1/12)*0.01*0.01;   Icyy(1)=(1/12)*m(1)*(al(1)*al(1)+0.1*0.1);  Iczz(1)=(1/12)*m(1)*al(1)*al(1);
% Icxx(2)=(1/12)*0.01*0.01;   Icyy(2)=(1/12)*m(2)*al(2)*al(2);  Iczz(2)=(1/12)*m(2)*al(2)*al(2);
% Icxx(3)=(1/12)*0.01*0.01;   Icyy(3)=(1/12)*m(3)*al(3)*al(3);  Iczz(3)=(1/12)*m(3)*al(3)*al(3);
% Icxx(4)=(1/12)*0.01*0.01;   Icyy(4)=(1/12)*m(4)*al(4)*al(4);  Iczz(4)=(1/12)*m(4)*al(4)*al(4);

% Icxx(1)=x_ac(1);  Icyy(1)=x_ac(1); Iczz(1)=x(1);
% Icxx(2)=x_ac(5);   Icyy(2)=x_ac(5);  Iczz(2)=x(5);
% Icxx(3)=x_ac(9);   Icyy(3)=x_ac(9);  Iczz(3)=x(9);
Icxx(1)=1200;  Icyy(1)=1200; Iczz(1)=1200;
Icxx(2)=21.3237031256132;   Icyy(2)=21.323703125613214;  Iczz(2)=21.323703125613214;
Icxx(3)=14.022688901859365;   Icyy(3)=14.022688901859365;  Iczz(3)=14.022688901859365;

end