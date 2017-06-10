function [y0, t_initial, t_final, incr, rtol, atol]=initials()

%Simulation time
t_initial=0;
t_final=30;

%Inverse kinematics for obtaining initial configuration
[n]=inputs();

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