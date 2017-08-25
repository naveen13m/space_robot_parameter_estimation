function [Ka1_error, Ka2_error, Ka3_error] = compute_coeff_error(case_name, global_kin_mat)
    curr_dir = pwd;
    cd(strcat('./kinematic_structural_data/', case_name));
    [num_links, ~, joint_twist_angle, link_length_DH, joint_offset] = inputs();
    cd(curr_dir);
    
    z10nl = zeros(10, num_links - 1);
    z11nl = zeros(11, num_links - 1);
    
    K12_comp = z10nl;
    K13_comp = z10nl;
    K21_comp = z11nl;
    K22_comp = z10nl;
    K23_comp = z10nl;
    
    Ka1_comp = z11nl;
    Ka2_comp = z10nl;
    Ka3_comp = z10nl;
    
    Ka1_act = z11nl;
    Ka2_act = z10nl;
    Ka3_act = z10nl;
        
    for curr_link_index = 2 : num_links
        s_al = sin(joint_twist_angle(curr_link_index));
        c_al = cos(joint_twist_angle(curr_link_index));
        piLpi = [link_length_DH(curr_link_index);
              -joint_offset(curr_link_index) * s_al;
               joint_offset(curr_link_index) * c_al];
        lx = piLpi(1); ly = piLpi(2); lz = piLpi(3);
        K12_comp(:, curr_link_index - 1) = [zeros(6, 1); 1; piLpi];
        K13_comp(:, curr_link_index - 1) = [zeros(8, 1); -s_al; c_al];
        K21_comp(:, curr_link_index - 1) = [1; c_al^2; s_al^2; 0; c_al * s_al; 0; 0; 0; 0; 0; -1];
        K22_comp(:, curr_link_index - 1) = [inertia_mat_to_vec(-vec_to_mat(piLpi)^2); 1; piLpi];
        K23_comp(:, curr_link_index - 1) = [2 * lz * c_al - 2 * ly * s_al;
                                            2 * lz * c_al; 
                                           -2 * ly * s_al;
                                            lx * s_al;
                                            lz * s_al - ly * c_al;
                                           -lx * c_al;
                                            0;
                                            0;
                                           -s_al;
                                            c_al];
        act_coeffs_mat = rref(global_kin_mat);
        Ka1_comp(:, curr_link_index - 1) = K21_comp(:, curr_link_index - 1); 
        Ka2_comp(:, curr_link_index - 1) = club_ld_coeffs(K12_comp(:, curr_link_index - 1), ...
                                        K22_comp(:, curr_link_index - 1));
        Ka3_comp(:, curr_link_index - 1) = club_ld_coeffs(K13_comp(:, curr_link_index - 1), ...
                                        K23_comp(:, curr_link_index - 1));
        
        Ka1_act(:, curr_link_index - 1) = elim_negligible_coeffs(act_coeffs_mat(1 : 11, 12));
        Ka2_act(:, curr_link_index - 1) = elim_negligible_coeffs(act_coeffs_mat(1 : 10, 17));
        Ka3_act(:, curr_link_index - 1) = elim_negligible_coeffs(act_coeffs_mat(1 : 10, 20));
        global_kin_mat(:, 1 : 10) = [];
    end
    Ka1_error = Ka1_act - Ka1_comp;
    Ka2_error = Ka2_act - Ka2_comp;
    Ka3_error = Ka3_act - Ka3_comp;
end

function eff_ld_coeffs = club_ld_coeffs(set1, set2)
    lnt = length(set1);
    eff_ld_coeffs = zeros(lnt, 1);
    for coeff_idx = 1 : lnt
        if set1(coeff_idx) == 0  
            eff_ld_coeffs(coeff_idx) = set2(coeff_idx);
        elseif set2(coeff_idx) == 0
            eff_ld_coeffs(coeff_idx) = set1(coeff_idx);
        elseif set1(coeff_idx) == 0  || set1(coeff_idx) == 0 
            eff_ld_coeffs(coeff_idx) = 0;
        else
            eff_ld_coeffs(coeff_idx) =  set1(coeff_idx);
        end
    end     
end

function coeffs = elim_negligible_coeffs(coeffs)
    lnt = length(coeffs);
    eps = 10e-7;
    for coeff_idx = 1 : lnt
        if abs(coeffs(coeff_idx)) < eps
            coeffs(coeff_idx) = 0;
        end
    end
end