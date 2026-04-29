# DockMatchConfidence.msg
#
# Published by mowgli_lidar_docking/dock_scan_match at 10 Hz alongside
# /dock_match/pose. Downstream consumers (FineDock BT, dock_yaw_to_set_pose,
# RecordDockApproachPose, GUI) MUST gate on `trusted` before using the pose.
#
# Trust threshold (defaults, overridable via dock_scan_match params):
#   trusted = (inlier_ratio >= min_inlier_ratio) AND (rmse_m <= max_rmse_m)
#
# Topic:  /dock_match/confidence  (rclcpp::SensorDataQoS)
# Pose companion topic: /dock_match/pose (PoseWithCovarianceStamped, reliable depth=1)

std_msgs/Header header
float32 inlier_ratio    # [0.0, 1.0] - fraction of registered points within max_correspondence_distance
float32 rmse_m          # metres - sqrt(mean(squared_residuals)) over inliers
bool    trusted         # convenience: precomputed gate (saves consumer-side recomputation)
