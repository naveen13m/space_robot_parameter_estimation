% Constructs the parameters of a the reaction wheel present on the physical
% robot
% Outputs: Reaction wheel parameters as a structure
function rw_params = make_real_reaction_wheel()
    rw_params.Ixx = 0; rw_params.Iyy = 0; rw_params.Izz = 3.90233e-4;
%     rw_params.Izz = 3.90233e-3;
    rw_params.Ixy = 0; rw_params.Iyz = 0; rw_params.Izx = 0;
    rw_params.m = 0.63;
    rw_params.x_com = 0; rw_params.y_com = 0; rw_params.z_com = 0;
end