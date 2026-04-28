# Live-tunable planning parameters for map_server_node.
#
# Published from the GUI to /map_server_node/planning_params_in (one-way,
# best-effort) when the operator saves the Mowing-Pattern card. Service-based
# variants of this update kept hitting foxglove_bridge typesupport bugs on
# rmw_cyclonedds; topic-based publish goes through cleanly.
#
# Sentinels mean "leave the field alone": <0 for non-negative fields,
# >180 for the angle (since -1 already means "auto"). The handler casts
# outline_passes back to int internally.
float64 outline_passes        # >=0 (cast to int by handler); -1 = unchanged
float64 outline_offset        # >=0; <0 = unchanged
float64 outline_overlap       # >=0; <0 = unchanged
float64 path_spacing          # >0;  <=0 = unchanged
float64 mow_angle_offset_deg  # any; >180 = unchanged (sentinel kept distinct from auto=-1)
float64 headland_width        # >=0; <0 = unchanged
