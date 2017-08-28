function [min_jt_angle, max_jt_angle, min_jt_speed, max_jt_speed, ...
                                   min_jt_acc, max_jt_acc] = joint_limits()
    min_jt_angle = [-pi/2;  -2 * pi/3];
    max_jt_angle = [pi/2;  2 * pi/3];
    min_jt_speed = [-pi/2;  -2 * pi/3];
    max_jt_speed = [pi/2;  2 * pi/3];
    min_jt_acc = [-pi/2;  -2 * pi/3];
    max_jt_acc = [pi/2;  2 * pi/3];
end

