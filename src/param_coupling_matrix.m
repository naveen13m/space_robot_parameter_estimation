function [coupled_param_vec, param_coupling_mat] = param_coupling_matrix()
    close all; clc;
    
    % Experiment setup
    num_links = 3;
    
    % Dynamic parameters
    mass = nd_sym('m', [1, num_links]);
    mass_link_com_prod = nd_sym('ma', [3, num_links]);
    inertia = nd_sym('I', [6, num_links]);
    
    % Kinematic parameters
    joint_twist = nd_sym('al', [1, num_links]);
    base_link_length = nd_sym('bl', [3, 1]);
    arm_link_length = nd_sym('l', [3, num_links - 1]);
    joint_twist(1) = 0;    
    
    link_length = [base_link_length(:, 1), arm_link_length];
    [coupled_param_vec, inst_param_coupling_mat] = reduced_parameter(mass, mass_link_com_prod, inertia, ...
                                     joint_twist, link_length, num_links);
    row_index = (0 : 1 : (num_links * 10) - 3 * (num_links - 1)).';
    col_index = 1 : 1 : num_links * 10;    
    param_coupling_mat = simplify([row_index, [col_index; inst_param_coupling_mat]]);
end

function [coupled_param_vec, reg_mat] = reduced_parameter(mass, mass_link_com_prod, inertia, ...
                                                joint_twist, link_length, num_links)
    % Construct decoupled dynamic parameter vector
    decoupled_param_vec = sym(zeros(10 * num_links, 1));
    for curr_link_index = 1 : num_links
        decoupled_param_vec(10 * curr_link_index - 9 : 10 * curr_link_index, :) = ...
            [inertia(:, curr_link_index)
             mass(curr_link_index);
             mass_link_com_prod(:, curr_link_index)];
    end
    coupled_param_vec = decoupled_param_vec;
    
    for curr_link_index = num_links : -1 : 2
        prev_link = curr_link_index - 1;
        prev_link_length = link_length(:, prev_link);
        curr_link_length = link_length(:, curr_link_index);
        s_al = sin(joint_twist(curr_link_index));
        c_al = cos(joint_twist(curr_link_index));
        % Backward iteration for mass
        coupled_param_vec(10 * prev_link - 3 : 10 * prev_link, :) = ...
            coupled_param_vec(10 * prev_link - 3 : 10 * prev_link, :) + ...
            coupled_param_vec(10 * curr_link_index - 3) * ...
                [1;
                 prev_link_length];
        % Backward iteration for z-com coordinate
        lx = curr_link_length(1); ly = curr_link_length(2); lz = curr_link_length(3); 
        coupled_param_vec(10 * prev_link - 9 : 10 * prev_link, :) = ...
            coupled_param_vec(10 * prev_link - 9 : 10 * prev_link, :) + ...
            coupled_param_vec(10 * curr_link_index, :) * ...
                [2 * (lz * c_al - ly * s_al);
                 2 * lz * c_al;
                -2 * lz * s_al;
                 lx * s_al;
                 lz * s_al - ly * c_al;
                -lx * c_al;
                 0;
                 0;
                -s_al;
                 c_al];         
        % Backward iteration for Iyy
        coupled_param_vec([10 * prev_link - 9 : 10 * prev_link - 4, 10 * curr_link_index - 9], :) = ...
            coupled_param_vec([10 * prev_link - 9 : 10 * prev_link - 4, 10 * curr_link_index - 9], :) + ...
            coupled_param_vec(10 * curr_link_index - 8) * ...
                [1;
                 c_al^2;
                 s_al^2;
                 0
                 c_al * s_al;
                 0
                -1];
    end
    coupled_param_vec([17 : 10 : end, 20 : 10 : end, 12 : 10 : end], :) = [];
    coupled_param_vec = expand(coupled_param_vec);
    reg_mat = compute_param_coupling_mat(coupled_param_vec, decoupled_param_vec);
end

function param_coupling_mat = compute_param_coupling_mat(red_param_vec, dyn_param_vec)
    num_cols = length(dyn_param_vec);
    num_rows = length(red_param_vec);
    param_coupling_mat = sym(zeros(num_rows, num_cols));
    for curr_equ = 1 : num_rows
        for curr_param = 1 : num_cols
            all_coeffs = coeffs(red_param_vec(curr_equ, :), dyn_param_vec(curr_param));
            num_coeffs = length(all_coeffs);
            if num_coeffs == 1
                if all_coeffs ~= 1
                    dyn_param_coeff = sym(0);
                else
                    dyn_param_coeff = all_coeffs;
                end
            else
                dyn_param_coeff = all_coeffs(end);
            end
            param_coupling_mat(curr_equ, curr_param) = dyn_param_coeff;
        end
    end
end