% Computes the inertia of a rigid body using parallel axis theorem in 3D
% Inputs: Inertia about the CoM (3 X 3)
%         Mass of the body (1 X 1)    
%         Position vector of the new frame in the CoM frame (3, 1)
% Outputs: Inertia of the body in the new frame (3 X 3)
function I_new = move_inertia_axis(I_com, m, pos_vec)
    I_new = I_com + m * (pos_vec.' * pos_vec * eye(3) - pos_vec * pos_vec.');
end