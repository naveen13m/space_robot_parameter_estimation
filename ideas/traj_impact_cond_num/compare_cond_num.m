clear all; close all; clc;

load('reg_mat_case1.mat');
load('reg_mat_case2.mat');

cond_case1 = cond(reg_mat_case1)
cond_case2 = cond(reg_mat_case2)
cond_case12 = cond([reg_mat_case1; reg_mat_case2])