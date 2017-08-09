function [coupled_param_vec, param_coupling_mat] = param_coupling_matrix()
    close all; clc;
    
    % Experiment setup
    num_links = 3;
    
    % Dynamic parameters
    mass = nd_sym('m', [1, num_links]);
    link_com = nd_sym('a', [3, num_links]);
    inertia = nd_sym('I', [6, num_links]);
    
    % Kinematic parameters
    joint_twist = nd_sym('al', [1, num_links]);
    base_link_length = nd_sym('bl', [3, 1]);
    arm_link_length = nd_sym('l', [3, num_links - 1]);
    joint_twist(1) = 0;    
    
    link_length = [base_link_length(:, 1), arm_link_length];
    [coupled_param_vec, inst_param_coupling_mat] = reduced_parameter(mass, link_com, inertia, ...
                                     joint_twist, link_length, num_links);
    row_index = (0 : 1 : (num_links * 10) - 3 * (num_links - 1)).';
    col_index = 1 : 1 : num_links * 10;    
    param_coupling_mat = simplify([row_index, [col_index; inst_param_coupling_mat]]);
end

function [coupled_param_vec, reg_mat] = reduced_parameter(mass, link_com, inertia, ...
                                                joint_twist, link_length, num_links)
    % Construct decoupled dynamic parameter vector
    % Simple link parameters
    decoupled_param_vec = sym(zeros(10 * num_links + 10 * (num_links - 1), 1));
    param_vec = sym(zeros(10 * num_links, 1));
    for curr_link_index = 1 : num_links
        decoupled_param_vec(20 * curr_link_index - 19 : 20 * curr_link_index - 10, :) = ...
            [inertia(:, curr_link_index)
             mass(curr_link_index);
             mass(curr_link_index) * link_com(:, curr_link_index)];
        param_vec(10 * curr_link_index - 9 : 10 * curr_link_index, :) = ...
            [inertia(:, curr_link_index)
             mass(curr_link_index);
             link_com(:, curr_link_index)];
    end
    % Compound link parameter
    num_cpd_links = num_links - 1;
    cpd_mass = zeros(1, num_cpd_links);
    cpd_com = zeros(0, num_cpd_links);
    cpd_length = zeros(3, num_cpd_links);
    cpd_inertia = zeros(6, num_cpd_links);
    for curr_cpd_index = 1 : num_cpd_links
        input_mass = mass(curr_cpd_index, curr_cpd_index + 1);
        input_com = link_com(:, [curr_cpd_index, curr_cpd_index + 1]);
        input_length = link_length(:, [curr_cpd_index, curr_cpd_index + 1]);
        input_inertia = inertia(:, [curr_cpd_index, curr_cpd_index + 1]);
        [cpd_mass(curr_cpd_index), cpd_com(:, curr_cpd_index), ...
            cpd_length(:, curr_cpd_index), cpd_inertia(:, curr_cpd_index)] = ...
            compute_cpd_link_params(input_mass, input_com, input_length, ...
            input_inertia, input_joint_angle, input_joint_twist);
    end
    coupled_param_vec = decoupled_param_vec;
    
    % Backward iteration of link parameters
    for curr_link_index = num_links : -1 : 2
        prev_link = curr_link_index - 1;
        prev_link_length = link_length(:, prev_link);
        s_al = sin(joint_twist(curr_link_index));
        c_al = cos(joint_twist(curr_link_index));
        % Backward iteration for mass
        coupled_param_vec(10 * prev_link - 3 : 10 * prev_link, :) = ...
            coupled_param_vec(10 * prev_link - 3 : 10 * prev_link, :) + ...
            coupled_param_vec(10 * curr_link_index - 3) * ...
                [1;
                 prev_link_length];
        % Backward iteration for z-com coordinate
        lx = prev_link_length(1); ly = prev_link_length(2); lz = prev_link_length(3); 
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
    reg_mat = compute_param_coupling_mat(coupled_param_vec, param_vec, mass, link_com);
end

function param_coupling_mat = compute_param_coupling_mat(coupled_param_vec, param_vec, mass, link_com)
    num_cols = length(param_vec);
    num_rows = length(coupled_param_vec);
    param_coupling_mat = sym(zeros(num_rows, num_cols));
    for curr_equ_index = 1 : num_rows
        for curr_param_index = 1 : num_cols
            equation = coupled_param_vec(curr_equ_index, :);
            parameter = param_vec(curr_param_index);
            all_coeffs = coeffs(equation, parameter);
            num_coeffs = length(all_coeffs);
            is_mass_index = mod(curr_param_index, 10) == 7; 
            is_com_index = (mod(curr_param_index, 10) == 8) || ...
            (mod(curr_param_index, 10) == 9) || (mod(curr_param_index, 10) == 0);
            param_link_index = fix((curr_param_index - 1) / 10) + 1;
            if is_mass_index
                all_coeffs = simplify(subs(all_coeffs, link_com(:, param_link_index), [0; 0; 0]));
            elseif is_com_index
                all_coeffs = all_coeffs / mass(param_link_index);
            end
            if num_coeffs == 1
                if all_coeffs ~= 1
                    dyn_param_coeff = sym(0);
                else
                    dyn_param_coeff = all_coeffs;
                end
            else
                dyn_param_coeff = all_coeffs(end);
            end
            param_coupling_mat(curr_equ_index, curr_param_index) = dyn_param_coeff;
        end
    end
end

function [cpd_mass, cpd_com, cpd_length, cpd_inertia] = ...
        compute_cpd_link_params(mass, link_com, link_length, inertia, ...
        joint_angle, joint_twist)
    cpd_mass = sum(mass);
    rot_mat_link_prev_link_frame = DH_to_rot_mat(joint_twist, joint_angle);
    cpd_com = m(1) * link_com(:, 1) + m(2) * (link_length(:, 1) + ...
        rot_mat_link_prev_link_frame * link_com(:, 2));
    cpd_length = link_length(:, 1) + rot_mat_link_prev_link_frame * link_com(:, 2);
    cpd_inertia  = inertia(:, 1) + ...
        inertia_transformation_matrix(joint_twist(1), joint_angle(1)) * inertia(:, 2) - ...
        - mass(2) * inertia_mat_to_vec(vec_to_mat(link_length(:, 2)));
end

function mat = inertia_transformation_matrix(al, th)
    cth = cos(th); sth = sin(th); cal = cos(al); sal = sin(al);
    c2th = cos(th); s2th = sin(th); c2al = cos(al); s2al = sin(al);
    mat = [cth^2               sth^2               0            -s2th^2               0             0;
           cal^2 * sth^2       cal^2 * cth^2       sal ^2        cal^2 * s2th        -s2al * cth   -s2al * s2th;
           sal^2 * sth^2       sal^2 * cth^2       0             sal^2 * s2th         s2al * cth    s2al * sth;
           cal * cth * sth    -cal * cth * sth     cal^2         cal * c2th           sal * sth    -sal * cth; 
           cal * sal * sth^2   cal * sal * cth^2  -sal * cal     cal * sal * c2th     c2al * cth    c2al * sth;
           sal * cth * sth    -sal * cth * sth     0             sal * c2th          -cal * sth     cal * cth];
end