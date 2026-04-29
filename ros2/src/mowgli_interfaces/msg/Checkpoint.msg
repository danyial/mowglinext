# Per-area resume state. One Checkpoint per working area, persisted as
# <areas_dir>/coverage_<area_index>.kv (key=value text, atomic write per D-05/D-06).
uint32 area_index
uint32 current_outline_index
uint32 current_swath_index
uint8 swath_direction
uint32 last_completed_swath_index
uint32 next_open_swath_index
float64 last_mow_angle_deg           # normalized to [0, 180)
geometry_msgs/Pose last_swath_endpoint

uint8 SWATH_DIRECTION_FORWARD=0
uint8 SWATH_DIRECTION_REVERSE=1
