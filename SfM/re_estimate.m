function [R_est_1, t_est_1] = re_estimate
load ray1_select.mat
load ray2_select.mat
load points_3d_est.mat
load cross_point_effctive.mat

function val = angle_constraint(x, xs, ray)
