function rot_mat = ZXY_euler_to_rot_mat(phi, theta, psi)
    rot_mat = [cos(phi) * cos(psi) - sin(phi) * sin(psi) * sin(theta)   -cos(theta) * sin(phi)   cos(phi) * sin(psi) + cos(psi) * sin(phi) * sin(theta)
               cos(psi) * sin(phi) + cos(phi) * sin(psi) * sin(theta)    cos(phi) * cos(theta)   sin(phi) * sin(psi) - cos(phi) * cos(psi) * sin(theta) 
              -cos(theta) * sin(psi)                                     sin(theta)              cos(psi) * cos(theta)                          ];
end