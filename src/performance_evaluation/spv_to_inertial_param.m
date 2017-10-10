% After obtaining one of the SPVs of the desired MPV, this function
% converts such an SPV to the inertial parameters i.e. into values needed 
% for the inputs file of ReDySim.
function input_param = spv_to_inertial_param(std_param_vec)
   num_links = length(std_param_vec) / 10;
   input_param = zeros(10 * num_links, 1);
   for curr_link = 1 : num_links
       curr_param = std_param_vec(10 * (curr_link - 1) + 1 : 10 * curr_link);
       curr_param(8 : 10) = curr_param(8 : 10) / curr_param(7);
       x_d = curr_param(8 : 10);
       curr_param(1 : 6) = curr_param(1 : 6) - ...
           curr_param(7) * inertia_mat_to_vec(x_d.' * x_d * eye(3) - x_d * x_d.');                     
       input_param(10 * (curr_link - 1) + 1 : 10 * curr_link) = curr_param;
   end
end