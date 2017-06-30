% ReDySim animate module. This module animates the system under study
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi
function [] = animate()
tskip=0.05;
disp('------------------------------------------------------------------');
disp('Animating the simulation data');
load timevar.dat;
load statevar.dat;
T=timevar;
Y=statevar;
s=0;
j=1;
[n nq alp a b bt dx dy dz al alt]=inputs();
for i=2:length(T)
    if T(i)>s
        time(j)=T(i);
        q=Y(i,1:6);
        th=Y(i,6:6+n-1);
        nqn=6+n;
        dq=Y(i,nqn:nqn+6-1)';
        dth=Y(i,nqn+6-1:2*(n+6-1))';
        [tt vc  scf vcf sof stf sbf]=for_kine(q,th, dq, dth, n, alp, a, b, bt, dx, dy, dz, al, alt);
        BX(j,:)=[sbf(1,1:8) sbf(1,1)];
        BY(j,:)=[sbf(2,1:8) sbf(2,1)];
        BZ(j,:)=[sbf(3,1:8) sbf(2,1)];
        L1X(j,:)=[sof(1,2:n) stf(1,n)];
        L1Y(j,:)=[sof(2,2:n) stf(2,n)];
        L1Z(j,:)=[sof(3,2:n) stf(3,n)];
        j=j+1;
        s=s+tskip;
    else
        continue
    end
end
xc=0;yc=-2;
nst=sum(al)- al(1);
[qq]=initials();
xmin=qq(1)-1*nst;
xmax=qq(1)+1*nst;
ymin=qq(2)-1*nst;
ymax=qq(2)+1*nst;
zmin=qq(3)-1*nst;
zmax=qq(3)+1*nst;

figure('Name','Animation Window','NumberTitle','off');
for i=1:length(time)
    t=time(i);
    t=num2str(t);
    plot3(L1X(i,:),L1Y(i,:),L1Z(i,:),'linewidth',3);
    hold on;
    fill3(BX(i,1:4),BY(i,1:4),BZ(i,1:4),[4 4 4 4],BX(i,5:8),BY(i,5:8),BZ(i,5:8),[4 4 4 4]...
        ,BX(i,[2,3,6,7]),BY(i,[2,3,6,7]),BZ(i,[2,3,6,7]),[4 4 4 4], ...
        BX(i,[3,4,5,6]),BY(i,[3,4,5,6]),BZ(i,[3,4,5,6]),[4 4 4 4]);
    axis([xmin xmax ymin ymax zmin zmax ]);
    set (gca,'fontsize',10,'fontweight','normal','fontname','times new romans','linewidth',0.5,'Box', 'off','TickDir','out' );
    xlabel('X (m)','fontweight','n','fontsize',10);
    ylabel('Y (m)','fontweight','n','fontsize',10);
    zlabel('Z (m)','fontweight','n','fontsize',10);
    title(['Current time t=',t],'fontweight','normal','fontsize',10);
    if nq==0
    view(0,90)
    end
    hold off
    grid on;
    drawnow;
end

