% This file generates a parameter coupling matrix in symbolic or double
% formats given the inputs of the system. 
clear all; close all; clc;

%% Config parameters 
% Set the inputs file
base_sensor_base_frame_position_base_frame = [-0.2; -0.3; -0.4];

%% Common for all robots
[n nq alp a b bt dx dy dz al alt m g  Iaxx Iayy Iazz Iaxy Iayz Iazx] = ...
                                                              inputs_sym();
std_param_vec_sym = construct_std_param_vector_sym(Iaxx, Iayy, Iazz, Iaxy,...
                                            Iayz, Iazx, m, dx, dy, dz, n);
minimal_param_vec = compute_minimal_param_vector(std_param_vec_sym, n, nq, ...
                alp, a, b, bt, base_sensor_base_frame_position_base_frame);
link_com = [dx; dy; dz];

%% Symbolic to numeric variables
syms al                            
minimal_param_vec = subs(minimal_param_vec, al, pi/2);

%% Parameter coupling matrix computation
pcm = compute_param_coupling_mat(minimal_param_vec, std_param_vec_sym, m, link_com);
pcm = double(pcm);
save param_coupling_mat.mat pcm

                           

