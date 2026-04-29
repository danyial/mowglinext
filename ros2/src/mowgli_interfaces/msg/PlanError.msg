# Structured error for failed plans. Populated only when PlanCoverage.action result.success=false.
uint8 error_code

uint8 failed_validation_point        # 1-10 per SPEC §Validierung; 0 = "other / not validation-driven"
uint32[] affected_area_indices       # indices into MapArea[] from GetAllAreas
geometry_msgs/Polygon[] affected_polygons  # polygons that triggered the failure
string human_readable                # operator-facing English description

uint8 ERROR_NO_AREAS=1
uint8 ERROR_AREA_TOO_NARROW=2
uint8 ERROR_OBSTACLE_BLOCKS_AREA=3
uint8 ERROR_DOCK_OUTSIDE_AREAS=4
uint8 ERROR_FOOTPRINT_VIOLATION=5
uint8 ERROR_OBSTACLE_OFFSET_FAILED=6
uint8 ERROR_RESUME_CHECKPOINT_INVALID=7
uint8 ERROR_INTERNAL=255
