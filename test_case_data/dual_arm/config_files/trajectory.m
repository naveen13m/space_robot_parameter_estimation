% ReDySim trajectory module. The desired indpendent joint trejectories are 
% enterd here
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [th_d dth_d ddth_d]=trajectory(t, n, tf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1: Joint level trejectory: Cycloidal motion
thin=[0.3803;   -0.6198;    1.2867;    2.4411;    1.7062;   -1.0058];%60
thf=[-0.2530;   -1.8050;   -0.0364;    3.3946;    1.8050;    0.0364];
Tp=tf;
for i=1:n-1
    thi(i,1)=thin(i)+((thf(i)-thin(i))/Tp)*(t-(Tp/(2*pi))*sin((2*pi/Tp)*t));
    dthi(i,1)=((thf(i)-thin(i))/Tp)*(1-cos((2*pi/Tp)*t));
    ddthi(i,1)=(2*pi*(thf(i)-thin(i))/(Tp*Tp))*sin((2*pi/Tp)*t);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

th_d=thi;
dth_d=dthi;
ddth_d=ddthi;