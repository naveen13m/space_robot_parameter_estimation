% ReDySim trajectory module. The desired indpendent joint trejectories are 
% enterd here
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [th_d dth_d ddth_d]=trajectory(t, n, tf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, add_rw] = inputs();
% 1: Joint level trejectory: Cycloidal motion
thin=[pi/6, 0].';
thf=[-pi/6, -pi/3].';

Tp=tf;
if add_rw
	thin = [thin; 0;      0     ; pi/2; 0];
	thf =  [thf;  12 * pi; -12 * pi; pi/2; 12 * pi];
end

for i=1:n-1
%     if ((i <= 2) || (i == 5))
        thi(i,1)=thin(i)+((thf(i)-thin(i))/Tp)*(t-(Tp/(2*pi))*sin((2*pi/Tp)*t));
        dthi(i,1)=((thf(i)-thin(i))/Tp)*(1-cos((2*pi/Tp)*t));
        ddthi(i,1)=(2*pi*(thf(i)-thin(i))/(Tp*Tp))*sin((2*pi/Tp)*t);
%     else
%         Tp = 2;
%         if (t <= 1)
%             thi(i,1)=thin(i)+((thf(i)-thin(i))/Tp)*(t-(Tp/(2*pi))*sin((2*pi/Tp)*t));
%             dthi(i,1)=((thf(i)-thin(i))/Tp)*(1-cos((2*pi/Tp)*t));
%             ddthi(i,1)=(2*pi*(thf(i)-thin(i))/(Tp*Tp))*sin((2*pi/Tp)*t);
%         elseif ((t > 1) && (t < 19))
%             tuse = t - 1;
%             thi(i,1)=((thf(i)-thin(i))/Tp) + 2*((thf(i)-thin(i))/Tp)*tuse;
%             dthi(i,1)= 2*((thf(i)-thin(i))/Tp);
%             ddthi(i,1)=0;
%         else
%             tuse = t - 18;
%             thi(i,1)= 2*((thf(i)-thin(i))/Tp)*18 + ((thf(i)-thin(i))/Tp)*(tuse-(Tp/(2*pi))*sin((2*pi/Tp)*tuse));
%             dthi(i,1)=((thf(i)-thin(i))/Tp)*(1-cos((2*pi/Tp)*tuse));
%             ddthi(i,1)=(2*pi*(thf(i)-thin(i))/(Tp*Tp))*sin((2*pi/Tp)*tuse);
%         end
%     end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

th_d=thi;
dth_d=dthi;
ddth_d=ddthi;

end