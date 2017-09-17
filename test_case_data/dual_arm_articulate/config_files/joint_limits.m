function [min_jt_angle, max_jt_angle, min_jt_vel, max_jt_vel, ...
                                   min_jt_acc, max_jt_acc] = joint_limits()
    min_jt_angle = [-90; -110; -110; 90; -110; -110; 0] * pi/180;
    max_jt_angle = [90; 110; 110; 270; 110; 110; 5400] * pi/180;
    min_jt_vel = [-25; -25; -25; -25; -25; -25; 0] * pi/180;
    max_jt_vel = [25; 25; 25; 25; 25; 25; 30] * pi/180;
    min_jt_acc = [-15; -15; -15; -15; -15; -15; -30] * pi/180;
    max_jt_acc = [15; 15; 15; 15; 15; 15; 30] * pi/180;
end

