function [min_jt_angle, max_jt_angle, min_jt_vel, max_jt_vel, ...
                                   min_jt_acc, max_jt_acc] = joint_limits()
    min_jt_angle = [-90; -45; 90; -45] * pi/180;
    max_jt_angle = [90; 45; 270; 45] * pi/180;
%     min_jt_angle = [-90; -90; 90; -90] * pi/180;
%     max_jt_angle = [90; 90; 270; 90] * pi/180;
    min_jt_vel = [-60; -60; -60; -60] * pi/180;
    max_jt_vel = [60; 60; 60; 60] * pi/180;
    min_jt_acc = [-360; -360; -360; -360] * pi/180;
    max_jt_acc = [360; 360; 360; 360] * pi/180;
end

