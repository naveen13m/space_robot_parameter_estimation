% Constructs the parameters of a reaction wheel
% Outputs: Reaction wheel parameters as a structure
function rw_params = make_reaction_wheel
    rw_params.Ixx = 1; rw_params.Iyy = 1; rw_params.Izz = 2;
    rw_params.Ixy = 0; rw_params.Iyz = 0; rw_params.Izx = 0;
    rw_params.m = 20;
    rw_params.x_com = 0; rw_params.y_com = 0; rw_params.z_com = 0;
end