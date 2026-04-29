# One waypoint in a coverage plan.
# Sparse plan: one pose per outline vertex / swath start / swath end / dock waypoint.
geometry_msgs/PoseStamped pose       # in map frame (header.frame_id = "map")
uint32 sequence_id                   # 0-based plan index, monotonic
uint32 area_index                    # MapArea index this waypoint belongs to;
                                     # UINT32_MAX (4294967295) for non-area
                                     # waypoints (UNDOCK/RETURN_TO_DOCK/
                                     # DOCK_APPROACH/DOCKING/start-pose TRANSIT).
                                     # Used by FollowCoveragePlan to stamp the
                                     # canonical Checkpoint key (R-9/R-11 fix).
float32 speed                        # m/s; matches mowing_speed/transit_speed/undock_speed
bool blade_enabled                   # true ONLY for OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON

# Segment type (controls blade + speed + Nav2 sub-action dispatch in FollowCoveragePlan)
uint8 segment_type
uint8 SEGMENT_UNDOCK=0
uint8 SEGMENT_TRANSIT=1
uint8 SEGMENT_OUTLINE_WORKING_AREA=2
uint8 SEGMENT_OUTLINE_OBSTACLE=3
uint8 SEGMENT_MOWING_BOUSTROPHEDON=4
uint8 SEGMENT_RETURN_TO_DOCK=5
uint8 SEGMENT_DOCK_APPROACH=6
uint8 SEGMENT_DOCKING=7
