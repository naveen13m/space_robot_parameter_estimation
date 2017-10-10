% ReDySim initials module. The initial conditions are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [y0, ti, tf, incr, rtol, atol, vel_combi_mat]=initials()

%Simulation time
ti=0;
tf=20;

%Inverse kinematics for obtaining initial configuration
[num_links, not_planar] = inputs();
is_planar = 1 - not_planar;
if is_planar
    num_rw_joints = 1;
else
    num_rw_joints = 4;
end

%Base motions
q=[0; 0; 0; 0; 0; 0];
dq=[0; 0; 0; 0; 0; 0];

% Actuator energy
acten=0;

%Vecotor of all the initial State Variable
y0=[q; dq; acten];

%INTERATION TOLERANCES
incr=0.1;
rtol=1e-5;         %relative tolerance in integration 
atol=1e-7;         %absolute tolerances in integration 

% num_arm_joints = num_links - 1 - num_rw_joints;
num_arm_joints = num_links - 1;
vel_combi_mat = generate_vel_combi_mat(num_arm_joints, is_planar);

end