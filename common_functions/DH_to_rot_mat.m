% Computes the rotation matrix from the modified DH parameters
% Inputs: Joint twist (1 X 1)
%         Joint angle (1 X 1)
% Ouputs: Rotation matrix (3 X 3)
function rot_mat = DH_to_rot_mat(joint_twist, joint_angle)
    c_ja = cos(joint_angle); s_ja = sin(joint_angle);
    c_jt = cos(joint_twist); s_jt = sin(joint_twist);
    rot_mat = [c_ja         -s_ja             0; ...
               s_ja * c_jt   c_ja * c_jt  -s_jt; ... 
               s_ja * s_jt   c_ja * s_jt   c_jt];
end