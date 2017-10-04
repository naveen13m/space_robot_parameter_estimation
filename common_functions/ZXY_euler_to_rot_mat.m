% Computes the rotation matrix from ZXY euler angles
% Inputs: phi, theta, psi are ZYX euler angles respectively (Each 1 X 1)
% Ouputs: Rotation matrix (3 X 3)
function rot_mat = ZXY_euler_to_rot_mat(phi, theta, psi)
    r11 = cos(phi) * cos(psi) - sin(phi) * sin(psi) * sin(theta);
    r12 = -cos(theta) * sin(phi);
    r13 = cos(phi) * sin(psi) + cos(psi) * sin(phi) * sin(theta);
    r21 = cos(psi) * sin(phi) + cos(phi) * sin(psi) * sin(theta);
    r22 = cos(phi) * cos(theta);
    r23 = sin(phi) * sin(psi) - cos(phi) * cos(psi) * sin(theta) ;
    r31 = -cos(theta) * sin(psi);
    r32 =  sin(theta);
    r33 = cos(psi) * cos(theta);
    rot_mat = [r11  r12  r13
               r21  r22  r23
               r31  r32  r33];
end