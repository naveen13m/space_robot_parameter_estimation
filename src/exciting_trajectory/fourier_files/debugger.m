clear all; close all; clc

% Verify trajectory.m
tr_par =  [0.246453055349776,0.157079632679490,0.930004965785531,1.11937146194537,-0.739777383079138,-0.253614141027095,-0.0417806438674086,-1.48515460692936,0.856654445118829,0.845845229883876,-0.635009759642293,-0.370971489805016,0.914746938452375,0.314185957061356,-0.329502563409821,0.0781272168195961,0.0244486685385692,-0.0496306462297603,0.0424769050258065,-0.918719682706590,0.0525960746998900,-0.0425473654988772,-0.0609509148334886,0.00300513410078804];
       
num_joints = 2;
t_diff = 0.1;
total_time = 0 : t_diff : 20;
num_instants = length(total_time);
fourier_value = zeros(num_joints, num_instants); 
dfourier_value = zeros(num_joints, num_instants); 
ddfourier_value = zeros(num_joints, num_instants); 

for time_index = 1 : 1 : num_instants
    [fourier_value(1 : num_joints, time_index), dfourier_value(1 : num_joints, time_index), ... 
        ddfourier_value(1 : num_joints, time_index)] = ...
        trajectory(total_time(time_index), num_joints + 1, tf, tr_par);
end

subplot(1, 3, 1);
plot(total_time, fourier_value(1, :), total_time, fourier_value(2, :)) 
legend('jt-1','jt-2');

subplot(1, 3, 2);
plot(total_time, dfourier_value(1, :), total_time, dfourier_value(2, :));
legend('jt-1','jt-2');

subplot(1, 3, 3);
plot(total_time, ddfourier_value(1, :), total_time, ddfourier_value(2, :));
legend('jt-1','jt-2');
