function [min_jt_angle, max_jt_angle, min_jt_velocity, max_jt_velocity, ...
                                   min_jt_acc, max_jt_acc] = joint_limits()
    min_jt_angle = [-90; -110] * pi/180;
    max_jt_angle = [90; 110] * pi/180;
    min_jt_velocity = [-25; -25] * pi/180;
    max_jt_velocity = [25; 25] * pi/180;
    min_jt_acc = [-15; -15] * pi/180;
    max_jt_acc = [15; 15] * pi/180;
end

