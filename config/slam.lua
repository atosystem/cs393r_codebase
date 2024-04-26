-- criteria for adding new node --------------------------------
min_trans_diff_between_nodes = 1;
-- min_trans_diff_between_nodes = M_PI / 6.0;
min_angle_diff_between_nodes = 3.14 / 6.0;

-- Motion Model --------------------------------
motion_model_trans_err_from_trans = 0.4;
motion_model_trans_err_from_rot = 0.4;
motion_model_rot_err_from_trans = 0.4;
motion_model_rot_err_from_rot = 0.4;


-- PoseGraph Parameters --------------------------------
considerOdomConstraint = false;
new_node_x_std = 1.0;
new_node_y_std = 1.0;
new_node_theta_std = 1.0;
non_successive_scan_constraints = true
max_factors_per_node = 15
maximum_node_dis_scan_comparison = 5.0
initial_node_global_x = -26
initial_node_global_y = 8
initial_node_global_theta = 0.0
