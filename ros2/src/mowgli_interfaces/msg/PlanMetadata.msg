# Plan-level metadata returned by PlanCoverage.action result.
float64 mow_angle_used_deg           # actual angle used; if goal=-1, this is the auto-rotated value
uint32 outline_passes_used           # configured outline_passes value applied
float64 path_spacing_used            # tool_width - strip_overlap (m)
uint32[] processed_area_indices      # areas planned successfully
uint32[] skipped_area_indices        # areas omitted from plan
string[] skip_reasons                # parallel to skipped_area_indices, same length
string[] warnings                    # non-fatal issues (e.g., narrow segment converted via OUTLINE_ONLY)
mowgli_interfaces/Checkpoint checkpoint_seed   # initial checkpoint embedded in plan
