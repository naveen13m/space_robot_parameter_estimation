function [min_jt_angle, max_jt_angle, min_jt_vel, max_jt_vel, ...
                                   min_jt_acc, max_jt_acc] = joint_limits()
    min_jt_angle = [-95; -50; 85; -50] * pi/180;
    max_jt_angle = [95; 50; 275; 50] * pi/180;
%     min_jt_angle = [-90; -90; 90; -90] * pi/180;
%     max_jt_angle = [90; 90; 270; 90] * pi/180;
    min_jt_vel = [-300; -300; -300; -300] * pi/180;
    max_jt_vel = [300; 300; 300; 300] * pi/180;
    min_jt_acc = [-1000; -1000; -1000; -1000] * pi/180;
    max_jt_acc = [1000; 1000; 1000; 1000] * pi/180;
end

