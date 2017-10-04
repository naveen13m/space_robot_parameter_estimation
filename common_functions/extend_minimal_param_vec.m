% Extends the MPV to the size of SPV by appending zeros
function extended_minimal_param_vec = extend_minimal_param_vec(...
                                  minimal_param_vec, num_links, is_planar)
    if is_planar
        extended_minimal_param_vec = zeros(10 * num_links, 1);
        extended_minimal_param_vec(1 : 10) = ...
        [0; 0; minimal_param_vec(1); 0; 0; 0; 1; minimal_param_vec(2 : 3); 0];
        for curr_link = 2 : num_links
            Izz_index = 3 * curr_link - 2;
            max_index = 3 * curr_link - 1;
            may_index = 3 * curr_link;
            extended_minimal_param_vec(10 * curr_link - 9 : 10 * curr_link) = ...
                [0; 0; minimal_param_vec(Izz_index); 0; 0; 0; 0; ...
                 minimal_param_vec(max_index); minimal_param_vec(may_index); 0];
        end
    else
        extended_minimal_param_vec = zeros(10 * num_links, 1);
        extended_minimal_param_vec(1 : 10) = ...
            [minimal_param_vec(1 : 6); 1; minimal_param_vec(7 : 9)];
        for curr_link = 2 : num_links
            Ixx_index = 9 + 7 * curr_link - 13;
            Izz_index = 9 + 7 * curr_link - 12;
            Ixz_index = 9 + 7 * curr_link - 9;
            max_index = 9 + 7 * curr_link - 8;
            may_index = 9 + 7 * curr_link - 7;
            extended_minimal_param_vec(10 * curr_link - 9 : 10 * curr_link) = ...
                [minimal_param_vec(Ixx_index); 0; ...
                minimal_param_vec(Izz_index : Ixz_index); 0; 
                minimal_param_vec(max_index); minimal_param_vec(may_index); 0];
        end
    end
end