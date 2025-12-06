rosbag record --tcpnodelay \
/drone_0_ego_planner_node/goal_point \
/ego_planner_node/global_list \
/drone_0_ego_planner_node/optimal_list \
/ego_planner_node/a_star_list \
/drone_0_ego_planner_node/init_list \
/drone_0_odom_visualization/path \
/drone_0_ego_planner_node/grid_map/occupancy_inflate \
/drone_0_odom_visualization/robot \
/vins_fusion/path \
/vins_fusion/odometry \
/vins_fusion/imu_propagate \
/vins_fusion/extrinsic \
/vins_fusion/image_track \
/detect_result_image/compressed \
/mavros/imu/data_raw \
/move_base_simple/goal \
/position_cmd
# /camera/infra1/image_rect_raw \
# /camera/infra2/image_rect_raw \
# /camera/depth/image_rect_raw \
