# Narrow-area strategy enum values (per SPEC R-13). Declaring them here
# lets all consumers reference the symbolic names instead of duplicating
# magic 0/1/2 across packages.
uint8 NARROW_AREA_SKIP = 0
uint8 NARROW_AREA_OUTLINE_ONLY = 1
uint8 NARROW_AREA_SPECIAL_PATTERN = 2

string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
bool is_navigation_area
uint8 narrow_area_strategy           # one of NARROW_AREA_* above
