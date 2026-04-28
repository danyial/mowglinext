package api

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sort"
	"sync"
	"time"

	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
	"gopkg.in/yaml.v3"
)

func SettingsRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider, rosProvider types.IRosProvider) {
	GetSettings(r, dbProvider)
	PostSettings(r, dbProvider)
	GetSettingsSchema(r, dbProvider)
	GetSettingsYAML(r, dbProvider)
	PostSettingsYAML(r, dbProvider, rosProvider)
	GetSettingsStatus(r, dbProvider)
	PostSettingsStatus(r, dbProvider)
}

// asFloat64 normalizes the menagerie of numeric types that can come back
// from JSON unmarshalling and YAML decoding into a single float64.
func asFloat64(v any) (float64, bool) {
	switch x := v.(type) {
	case float64:
		return x, true
	case float32:
		return float64(x), true
	case int:
		return float64(x), true
	case int64:
		return float64(x), true
	case json.Number:
		f, err := x.Float64()
		return f, err == nil
	}
	return 0, false
}

// readToolWidth reads the operator-facing blade width from the flat
// settings map. The flat map is expected to have schema defaults
// merged in by extractDefaults, so tool_width is always populated when
// this is called from a settings handler. Returns 0 if the value is
// genuinely unset or non-numeric — the caller's downstream math will
// then surface the misconfiguration rather than silently substitute a
// hardcoded default.
func readToolWidth(flat map[string]any) float64 {
	v, ok := flat["tool_width"]
	if !ok {
		return 0
	}
	f, _ := asFloat64(v)
	return f
}

// injectStripOverlap mutates flat in place to expose the GUI-facing
// strip_overlap field while hiding the legacy yaml fields. Called
// from GET handlers so the form sees only the new shape.
//
// When the yaml has a path_spacing value (either from a pre-#60
// install or from a recent save), strip_overlap is back-calculated
// from it: strip_overlap = tool_width - path_spacing. When yaml
// has no path_spacing, the schema-default for strip_overlap (already
// merged into flat) is left untouched.
//
// path_spacing and headland_width are stripped from the response
// either way — the form has no UI for them anymore.
func injectStripOverlap(flat map[string]any) {
	defer delete(flat, "path_spacing")
	defer delete(flat, "headland_width")
	ps, ok := flat["path_spacing"]
	if !ok {
		return
	}
	psFloat, ok := asFloat64(ps)
	if !ok {
		return
	}
	tw := readToolWidth(flat)
	overlap := tw - psFloat
	if overlap < 0 {
		overlap = 0
	}
	flat["strip_overlap"] = overlap
}

// applyMowingPatternConversion is the inverse of injectStripOverlap.
// Called on POST handlers before yaml-write and before the live-tune
// publish, so the on-disk yaml + the wire message keep using the
// legacy field name (path_spacing) that the C++ planner already
// reads. headland_width is left untouched — if the existing yaml has
// a value for it from a pre-#60 install, the planner-side fallback
// continues to honour it, but new saves never write or rewrite it.
//
//   path_spacing = toolWidth - strip_overlap
//
// outline_overlap stays in the yaml as the operator-set Strip-Outline
// Overlap value (semantics changed by #60: it's now metres of
// strip-to-outline blade overlap, decoupled from the inter-outline
// pass step).
//
// toolWidth is passed explicitly because the request payload from
// the GUI usually only contains the changed fields (so reading
// tool_width directly from the payload would return 0). Caller
// resolves it from the merged-with-defaults existing map.
func applyMowingPatternConversion(flat map[string]any, toolWidth float64) {
	so, ok := flat["strip_overlap"]
	if !ok {
		return
	}
	soFloat, _ := asFloat64(so)
	flat["path_spacing"] = toolWidth - soFloat
	delete(flat, "strip_overlap")
}

// liveTuneMapServer pushes the live-tunable subset of the settings payload
// to /map_server_node/planning_params_in (topic). We use a topic rather
// than rcl_interfaces/srv/SetParameters or our own SetPlanningParams
// service because foxglove_bridge can't relay either through rmw_cyclonedds:
// service requests round-trip through "rosidl_typesupport_cpp not supported
// by this library, rmw_serialize: invalid data size" no matter how flat the
// schema is. Topic publish goes through cleanly because the bridge already
// has working JSON↔CDR mappings advertised for every other topic.
//
// Sentinels (<0 for non-negative fields, >180 for the angle since -1 means
// "auto") leave the existing value alone, so we only push fields the
// operator actually changed and never clobber server-side overrides.
func liveTuneMapServer(ctx context.Context, rosProvider types.IRosProvider, payload map[string]any) {
	if rosProvider == nil {
		return
	}

	// Sentinel defaults — every field starts as "unchanged". Each branch
	// below overwrites the sentinel only if the GUI sent a value for that
	// key, so the server-side handler knows which fields to mutate.
	req := mowgli.SetPlanningParamsReq{
		OutlinePasses:     -1.0,
		OutlineOffset:     -1.0,
		OutlineOverlap:    -1.0,
		PathSpacing:       -1.0,
		MowAngleOffsetDeg: 999.0, // distinct from the GUI's own -1 = auto sentinel
		HeadlandWidth:     -1.0,
	}
	anyChange := false

	asFloat := func(v any) (float64, bool) {
		switch x := v.(type) {
		case float64:
			return x, true
		case float32:
			return float64(x), true
		case int:
			return float64(x), true
		case int64:
			return float64(x), true
		case json.Number:
			f, err := x.Float64()
			return f, err == nil
		}
		return 0, false
	}

	if v, ok := payload["outline_passes"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.OutlinePasses = f
			anyChange = true
		}
	}
	if v, ok := payload["outline_offset"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.OutlineOffset = f
			anyChange = true
		}
	}
	if v, ok := payload["outline_overlap"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.OutlineOverlap = f
			anyChange = true
		}
	}
	if v, ok := payload["path_spacing"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.PathSpacing = f
			anyChange = true
		}
	}
	if v, ok := payload["mow_angle_offset_deg"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.MowAngleOffsetDeg = f
			anyChange = true
		}
	}
	if v, ok := payload["headland_width"]; ok {
		if f, ok2 := asFloat(v); ok2 {
			req.HeadlandWidth = f
			anyChange = true
		}
	}
	if !anyChange {
		return
	}

	// Publish to /map_server_node/planning_params_in (topic) instead of
	// calling the SetPlanningParams service. foxglove_bridge can't relay our
	// service requests through rmw_cyclonedds (typesupport identifier
	// mismatch on rosidl_typesupport_cpp), but topic-pub/sub goes through
	// cleanly because the bridge already advertises a JSON↔CDR mapping for
	// every other topic in the system.
	wireMsg := mowgli.PlanningParams{
		OutlinePasses:     req.OutlinePasses,
		OutlineOffset:     req.OutlineOffset,
		OutlineOverlap:    req.OutlineOverlap,
		PathSpacing:       req.PathSpacing,
		MowAngleOffsetDeg: req.MowAngleOffsetDeg,
		HeadlandWidth:     req.HeadlandWidth,
	}
	if dbg, err := json.Marshal(wireMsg); err == nil {
		log.Printf("liveTuneMapServer: publishing planning_params_in = %s", string(dbg))
	}
	if err := rosProvider.Publish(
		"/map_server_node/planning_params_in",
		"mowgli_interfaces/msg/PlanningParams",
		&wireMsg,
	); err != nil {
		log.Printf("liveTuneMapServer: planning_params_in publish failed (yaml is still persisted): %v", err)
		return
	}
	log.Printf("liveTuneMapServer: planning_params_in published")
	_ = ctx // keep signature stable for callers passing request context
}

// GetSettingsStatus returns whether onboarding has been completed.
// Used by the frontend to decide whether to show the onboarding wizard.
//
// @Summary get settings onboarding status
// @Description returns whether onboarding has been completed
// @Tags settings
// @Produce json
// @Success 200 {object} SettingsStatusResponse
// @Router /settings/status [get]
func GetSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/status", func(c *gin.Context) {
		val, err := dbProvider.Get("onboarding.completed")
		completed := err == nil && string(val) == "true"
		c.JSON(200, gin.H{"onboarding_completed": completed})
	})
}

// PostSettingsStatus marks onboarding as completed.
//
// @Summary mark onboarding as completed
// @Description marks onboarding as completed so the wizard is not shown again
// @Tags settings
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/status [post]
func PostSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/status", func(c *gin.Context) {
		if err := dbProvider.Set("onboarding.completed", []byte("true")); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

func extractDefaults(schema map[string]any, defaults map[string]any) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				if def, hasDef := propMap["default"]; hasDef {
					defaults[key] = def
				}
				extractDefaults(propMap, defaults)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractDefaults(thenBlock, defaults)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractDefaults(elseBlock, defaults)
				}
			}
		}
	}
}

func extractAllKeys(schema map[string]any, keys map[string]bool) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				_, hasEnvVar := propMap["x-environment-variable"]
				if hasEnvVar || (propType != "" && propType != "object") {
					keys[key] = true
				}
				extractAllKeys(propMap, keys)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractAllKeys(thenBlock, keys)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractAllKeys(elseBlock, keys)
				}
			}
		}
	}
}

// derivedFloatKeys are yaml-on-disk numeric keys that the schema no
// longer references but still must be float-encoded so ROS2 can read
// them. Keep this list small — it is purely the bridge for fields
// that were renamed/hidden from the GUI but still feed the C++ side.
//   - path_spacing: derived from strip_overlap (#60).
//   - headland_width: legacy field preserved from pre-#60 yamls.
var derivedFloatKeys = []string{"path_spacing", "headland_width"}

// forceFloatYAML rewrites whole-number values for every schema "number"-typed
// key so they render as YAML floats (`0.0` instead of `0`). gopkg.in/yaml.v3
// drops the trailing zero on float64 round-trip, which ROS2's parameter
// loader then reads back as integer and InvalidParameterTypeException kills
// the node on the next startup. The fix is purely syntactic — values are
// untouched, only the on-disk representation changes.
func forceFloatYAML(in []byte, schema map[string]any) []byte {
	keyTypes := map[string]string{}
	if schema != nil {
		extractKeyTypes(schema, keyTypes)
	}
	for _, k := range derivedFloatKeys {
		keyTypes[k] = "number"
	}
	out := string(in)
	for key, t := range keyTypes {
		if t != "number" {
			continue
		}
		// Match `<indent><key>: <whole-int>` (no decimal point) at line end.
		re := regexp.MustCompile(`(?m)^(\s+)(` + regexp.QuoteMeta(key) + `):\s+(-?\d+)$`)
		out = re.ReplaceAllString(out, "$1$2: $3.0")
	}
	return []byte(out)
}

func extractKeyTypes(schema map[string]any, types_ map[string]string) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				if propType != "" && propType != "object" {
					types_[key] = propType
				}
				extractKeyTypes(propMap, types_)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractKeyTypes(thenBlock, types_)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractKeyTypes(elseBlock, types_)
				}
			}
		}
	}
}

// extractNodeMappings builds a map of paramKey -> ROS2 node name from
// x-yaml-node extensions in the schema. Keys without x-yaml-node default to "mowgli".
func extractNodeMappings(schema map[string]any) map[string]string {
	mappings := map[string]string{}
	allKeys := map[string]bool{}
	extractAllKeys(schema, allKeys)

	var walk func(s map[string]any)
	walk = func(s map[string]any) {
		if props, ok := s["properties"].(map[string]any); ok {
			for key, prop := range props {
				if propMap, ok := prop.(map[string]any); ok {
					if node, ok := propMap["x-yaml-node"].(string); ok {
						mappings[key] = node
					}
					walk(propMap)
				}
			}
		}
		if allOf, ok := s["allOf"].([]any); ok {
			for _, cond := range allOf {
				if condMap, ok := cond.(map[string]any); ok {
					if thenBlock, ok := condMap["then"].(map[string]any); ok {
						walk(thenBlock)
					}
					if elseBlock, ok := condMap["else"].(map[string]any); ok {
						walk(elseBlock)
					}
				}
			}
		}
	}
	walk(schema)

	// Default unmapped keys to "mowgli"
	for key := range allKeys {
		if _, exists := mappings[key]; !exists {
			mappings[key] = "mowgli"
		}
	}
	return mappings
}

// flattenROS2YAML reads nested ROS2 YAML (node: ros__parameters: {k: v}) and
// returns a flat map of all parameter key-value pairs across all nodes.
//
// Some keys legitimately appear in multiple nodes (datum_lat/datum_lon are
// declared on both `mowgli` and `navsat_to_absolute_pose` so each node loads
// its own copy from a single yaml). Two correctness rules apply:
//  1. Iterate node names in deterministic (sorted) order — Go map iteration
//     is randomized, so a naive last-wins merge produced non-deterministic
//     reads (the GUI's MapPage saw datum_lat=0 sometimes and 48.x other
//     times depending on hash seed).
//  2. Prefer non-zero values: if a key is present in two nodes with
//     different values (typically because nestToROS2YAML used to only
//     update one of them), the live value usually lives in the node that
//     was updated. Returning the zero would cause downstream consumers
//     (MapPage, MiniMap) to treat the datum as unset.
func flattenROS2YAML(yamlData map[string]any) map[string]any {
	flat := map[string]any{}

	nodeNames := make([]string, 0, len(yamlData))
	for n := range yamlData {
		nodeNames = append(nodeNames, n)
	}
	sort.Strings(nodeNames)

	for _, nodeName := range nodeNames {
		nodeMap, ok := yamlData[nodeName].(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for k, v := range rosParams {
			if existing, hadValue := flat[k]; hadValue && !isZeroish(v) && isZeroish(existing) {
				// Replace the zero with the meaningful value.
				flat[k] = v
			} else if !hadValue {
				flat[k] = v
			}
			// else: keep what we already have (either both non-zero,
			// or the existing value is non-zero and the new one is zero).
		}
	}
	return flat
}

// isZeroish returns true for values that look like "unset" — empty strings,
// zero numerics, nil, and the literal "0" / "0.0" strings YAML sometimes
// produces. Used by flattenROS2YAML to prefer meaningful values when a
// key appears in multiple nodes.
func isZeroish(v any) bool {
	if v == nil {
		return true
	}
	switch x := v.(type) {
	case float64:
		return x == 0
	case float32:
		return x == 0
	case int:
		return x == 0
	case int64:
		return x == 0
	case string:
		return x == "" || x == "0" || x == "0.0"
	}
	return false
}

// nestToROS2YAML takes a flat param map and node mappings, and produces
// nested ROS2 YAML structure. It preserves any existing YAML content that
// is not covered by the schema.
//
// When a key already exists in *multiple* nodes of the existing YAML
// (e.g. datum_lat lives on both `mowgli` and `navsat_to_absolute_pose`),
// the new value is written to *all* of those nodes. The previous behaviour
// updated only the schema-mapped node (mowgli by default), which left the
// other copy stale and triggered the dual-block bug: GET would re-read the
// stale zero from the un-updated block and overwrite the value the GUI just
// thought it had saved.
//
// For brand-new keys (not present anywhere in existingYAML) we still defer
// to the schema's x-yaml-node mapping, falling back to "mowgli".
func nestToROS2YAML(flat map[string]any, nodeMappings map[string]string, existingYAML map[string]any) map[string]any {
	result := map[string]any{}

	// Preserve existing top-level structure
	for nodeName, nodeData := range existingYAML {
		if nodeMap, ok := nodeData.(map[string]any); ok {
			cloned := map[string]any{}
			for k, v := range nodeMap {
				cloned[k] = v
			}
			result[nodeName] = cloned
		}
	}

	// Build a key → [nodes that already host it] index from existingYAML so
	// we can fan out shared keys (datum_lat etc.) to every place that
	// already declares them.
	keyHostNodes := map[string][]string{}
	for nodeName, nodeData := range existingYAML {
		nodeMap, ok := nodeData.(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for k := range rosParams {
			keyHostNodes[k] = append(keyHostNodes[k], nodeName)
		}
	}

	// Group flat params by node. Three cases:
	//   1. Schema declares an x-yaml-node target — write to that node only, and
	//      strip the key from any *other* node it currently lives in (migrate
	//      legacy entries that were written under the wrong namespace before
	//      the schema gained x-yaml-node hints).
	//   2. No schema mapping but the key already exists in existingYAML —
	//      fan out to every existing host so dual-block keys (e.g. datum_lat)
	//      stay in sync.
	//   3. Brand-new key with no mapping — fall back to "mowgli".
	nodeParams := map[string]map[string]any{}
	keysToStripFromNode := map[string]map[string]bool{}
	for key, value := range flat {
		schemaTarget, hasSchema := nodeMappings[key]
		hosts := keyHostNodes[key]

		var targets []string
		if hasSchema {
			targets = []string{schemaTarget}
			for _, host := range hosts {
				if host == schemaTarget {
					continue
				}
				if keysToStripFromNode[host] == nil {
					keysToStripFromNode[host] = map[string]bool{}
				}
				keysToStripFromNode[host][key] = true
			}
		} else if len(hosts) > 0 {
			targets = hosts
		} else {
			targets = []string{"mowgli"}
		}

		for _, n := range targets {
			if nodeParams[n] == nil {
				nodeParams[n] = map[string]any{}
			}
			nodeParams[n][key] = value
		}
	}

	// Strip keys that have been migrated to a different node so the legacy
	// entry doesn't shadow the new one on the next read.
	for nodeName, stripKeys := range keysToStripFromNode {
		nodeMap, ok := result[nodeName].(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for k := range stripKeys {
			delete(rosParams, k)
		}
		nodeMap["ros__parameters"] = rosParams
	}

	// Merge into result, preserving existing ros__parameters that aren't in flat
	for nodeName, params := range nodeParams {
		nodeMap, ok := result[nodeName].(map[string]any)
		if !ok {
			nodeMap = map[string]any{}
			result[nodeName] = nodeMap
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			rosParams = map[string]any{}
		}
		for k, v := range params {
			rosParams[k] = v
		}
		nodeMap["ros__parameters"] = rosParams
	}

	return result
}

func coerceValue(value string, schemaType string) any {
	switch schemaType {
	case "boolean":
		return strings.EqualFold(value, "true")
	case "number":
		if f, err := strconv.ParseFloat(value, 64); err == nil {
			return f
		}
	case "integer":
		if i, err := strconv.ParseInt(value, 10, 64); err == nil {
			return i
		}
	}
	return value
}

// applyMowgliOverlay modifies the settings schema to add Mowgli-specific
// options: adds "Mowgli" to the OM_MOWER enum, moves OM_HARDWARE_VERSION and
// OM_MOWER_ESC_TYPE behind conditional allOf rules, and adds OM_NO_COMMS for
// Mowgli boards.
func applyMowgliOverlay(schema map[string]any) map[string]any {
	props, ok := schema["properties"].(map[string]any)
	if !ok {
		return schema
	}
	hw, ok := props["important_settings"].(map[string]any)
	if !ok {
		return schema
	}
	hwProps, ok := hw["properties"].(map[string]any)
	if !ok {
		return schema
	}

	// Add "Mowgli" to OM_MOWER enum if not already present.
	if omMower, ok := hwProps["OM_MOWER"].(map[string]any); ok {
		if enum, ok := omMower["enum"].([]any); ok {
			found := false
			for _, v := range enum {
				if v == "Mowgli" {
					found = true
					break
				}
			}
			if !found {
				omMower["enum"] = append(enum, "Mowgli")
			}
		}
	}

	// Extract OM_HARDWARE_VERSION and OM_MOWER_ESC_TYPE from base properties.
	hwVersion := hwProps["OM_HARDWARE_VERSION"]
	escType := hwProps["OM_MOWER_ESC_TYPE"]
	delete(hwProps, "OM_HARDWARE_VERSION")
	delete(hwProps, "OM_MOWER_ESC_TYPE")

	// Build conditional allOf rules.
	nonMowgliThenProps := map[string]any{}
	if hwVersion != nil {
		nonMowgliThenProps["OM_HARDWARE_VERSION"] = hwVersion
	}
	if escType != nil {
		nonMowgliThenProps["OM_MOWER_ESC_TYPE"] = escType
	}

	nonMowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{
					"not": map[string]any{"const": "Mowgli"},
				},
			},
		},
		"then": map[string]any{
			"properties": nonMowgliThenProps,
		},
	}

	mowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{"const": "Mowgli"},
			},
		},
		"then": map[string]any{
			"properties": map[string]any{
				"OM_NO_COMMS": map[string]any{
					"type":    "boolean",
					"title":   "No Comms Mode",
					"default": true,
				},
			},
		},
	}

	hw["allOf"] = []any{nonMowgliCond, mowgliCond}

	return schema
}

func getSchema(dbProvider types.IDBProvider) (map[string]any, error) {
	schemaCacheMu.RLock()
	if schemaCache != nil && time.Since(schemaCacheTime) < schemaCacheTTL {
		cached := schemaCache
		schemaCacheMu.RUnlock()
		return cached, nil
	}
	schemaCacheMu.RUnlock()

	// Load local Mowgli schema (no upstream fetch)
	var schema map[string]any
	localFile, err := os.ReadFile("asserts/mower_config.schema.json")
	if err != nil {
		return nil, fmt.Errorf("failed to read local schema: %w", err)
	}
	if err := json.Unmarshal(localFile, &schema); err != nil {
		return nil, fmt.Errorf("invalid local schema JSON: %w", err)
	}

	schemaCacheMu.Lock()
	schemaCache = schema
	schemaCacheTime = time.Now()
	schemaCacheMu.Unlock()

	return schema, nil
}

// ============================================================================
// Legacy shell-config endpoints (kept for backward compatibility)
// ============================================================================

// PostSettings saves settings to the mower_config.sh file.
//
// @Summary saves the settings to the mower_config.sh file
// @Description saves the settings to the mower_config.sh file
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "settings key-value map"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [post]
func PostSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings", func(c *gin.Context) {
		var settingsPayload map[string]any
		err := c.BindJSON(&settingsPayload)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		mowerConfigFile, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		var settings = map[string]any{}
		configFileContent, err := os.ReadFile(string(mowerConfigFile))
		if err == nil {
			parse, err := godotenv.Parse(strings.NewReader(string(configFileContent)))
			if err == nil {
				for s, s2 := range parse {
					settings[s] = s2
				}
			}
		}

		schema, err := getSchema(dbProvider)
		defaults := map[string]any{}
		knownKeys := map[string]bool{}
		if err == nil {
			extractDefaults(schema, defaults)
			extractAllKeys(schema, knownKeys)
			for key, value := range defaults {
				if _, exists := settings[key]; !exists {
					settings[key] = value
				}
			}
		}

		customEnvVars := map[string]string{}
		for key, value := range settings {
			if !knownKeys[key] {
				if key != "custom_environment" {
					customEnvVars[key] = fmt.Sprintf("%v", value)
				}
				delete(settings, key)
			}
		}
		if customEnvObj, ok := settingsPayload["custom_environment"]; ok {
			customEnvVars = map[string]string{}
			if customEnv, ok := customEnvObj.(map[string]any); ok {
				for k, v := range customEnv {
					customEnvVars[k] = fmt.Sprintf("%v", v)
				}
			}
			delete(settingsPayload, "custom_environment")
		}
		for key, value := range settingsPayload {
			settings[key] = value
		}
		for k, v := range customEnvVars {
			settings[k] = v
		}

		var fileContent string
		for key, value := range settings {
			if key == "custom_environment" {
				continue
			}
			if value == true {
				value = "True"
			}
			if value == false {
				value = "False"
			}
			fileContent += "export " + key + "=" + fmt.Sprintf("%#v", value) + "\n"
		}
		if err = os.MkdirAll(filepath.Dir(string(mowerConfigFile)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		err = os.WriteFile(string(mowerConfigFile), []byte(fileContent), 0644)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// GetSettings returns a JSON object with the settings.
//
// @Summary returns a JSON object with the settings
// @Description returns a JSON object with the settings
// @Tags settings
// @Produce json
// @Success 200 {object} GetSettingsResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [get]
func GetSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings", func(c *gin.Context) {
		mowerConfigFilePath, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(mowerConfigFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				c.JSON(200, GetSettingsResponse{Settings: map[string]any{}})
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		settings, err := godotenv.Parse(strings.NewReader(string(file)))
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		schema, _ := getSchema(dbProvider)
		knownKeys := map[string]bool{}
		keyTypes := map[string]string{}
		if schema != nil {
			extractAllKeys(schema, knownKeys)
			extractKeyTypes(schema, keyTypes)
		}

		finalSettings := map[string]any{}
		customEnv := map[string]string{}
		for k, v := range settings {
			if knownKeys[k] || k == "custom_environment" {
				if t, ok := keyTypes[k]; ok {
					finalSettings[k] = coerceValue(v, t)
				} else {
					finalSettings[k] = v
				}
			} else {
				customEnv[k] = v
			}
		}
		if len(customEnv) > 0 {
			finalSettings["custom_environment"] = customEnv
		}

		c.JSON(200, GetSettingsResponse{Settings: finalSettings})
	})
}

// ============================================================================
// Schema endpoint
// ============================================================================

var (
	schemaCache     map[string]any
	schemaCacheMu   sync.RWMutex
	schemaCacheTime time.Time
	schemaCacheTTL  = 1 * time.Hour
)


// GetSettingsSchema returns the mower config JSON Schema.
//
// @Summary returns the mower config JSON Schema
// @Description returns the JSON Schema for mower configuration parameters
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/schema [get]
func GetSettingsSchema(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/schema", func(c *gin.Context) {
		schema, err := getSchema(dbProvider)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, schema)
	})
}

// ============================================================================
// YAML settings endpoints — primary for Mowgli ROS2
// ============================================================================

// GetSettingsYAML reads mowgli_robot.yaml, flattens the nested ROS2 YAML,
// and returns a flat key-value JSON object for the settings form.
//
// @Summary returns the current YAML mower configuration
// @Description returns the current YAML mower configuration values as a flat key-value map
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [get]
func GetSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/yaml", func(c *gin.Context) {
		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(configFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				// Return schema defaults when file doesn't exist
				schema, schemaErr := getSchema(dbProvider)
				if schemaErr != nil {
					c.JSON(200, map[string]any{})
					return
				}
				defaults := map[string]any{}
				extractDefaults(schema, defaults)
				c.JSON(200, defaults)
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		var yamlData map[string]any
		if err := yaml.Unmarshal(file, &yamlData); err != nil {
			c.JSON(500, ErrorResponse{Error: "invalid YAML: " + err.Error()})
			return
		}

		flat := flattenROS2YAML(yamlData)

		// Merge schema defaults for missing keys
		schema, err := getSchema(dbProvider)
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := flat[key]; !exists {
					flat[key] = value
				}
			}
		}

		// #60: project the legacy yaml shape (path_spacing, headland_width)
		// onto the new GUI-facing field (strip_overlap). Done after defaults
		// merge so tool_width is reliably populated.
		injectStripOverlap(flat)

		c.JSON(200, flat)
	})
}

// PostSettingsYAML receives flat key-value settings from the form,
// nests them into proper ROS2 YAML structure, and writes mowgli_robot.yaml.
//
// @Summary saves the mower configuration as YAML
// @Description saves the mower configuration as YAML to mowgli_robot.yaml
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "flat key-value settings map"
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [post]
func PostSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider, rosProvider types.IRosProvider) gin.IRoutes {
	return r.POST("/settings/yaml", func(c *gin.Context) {
		var payload map[string]any
		if err := c.BindJSON(&payload); err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}

		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		// Read existing YAML to preserve unknown parameters
		existingYAML := map[string]any{}
		file, err := os.ReadFile(string(configFilePath))
		if err == nil {
			_ = yaml.Unmarshal(file, &existingYAML)
		}

		// Flatten existing to get current values
		existing := flattenROS2YAML(existingYAML)

		// Merge schema defaults
		schema, err := getSchema(dbProvider)
		nodeMappings := map[string]string{}
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := existing[key]; !exists {
					existing[key] = value
				}
			}
			nodeMappings = extractNodeMappings(schema)
		}

		// Merge payload on top
		for key, value := range payload {
			existing[key] = value
		}

		// #60: convert the GUI-facing strip_overlap into the legacy
		// path_spacing yaml field that the C++ planner reads. tool_width
		// is resolved from `existing` (which has schema defaults merged)
		// because the request payload usually doesn't carry blade-width.
		// Both `existing` (for yaml write) and `payload` (for live-tune
		// republish) are converted so downstream code keeps seeing the
		// pre-#60 field name.
		toolWidth := readToolWidth(existing)
		applyMowingPatternConversion(existing, toolWidth)
		applyMowingPatternConversion(payload, toolWidth)

		// Debug: surface the resolved namespace for the live-tunable keys so
		// we can confirm x-yaml-node hints are honoured. Logged once per save.
		debugKeys := []string{"outline_passes", "outline_offset", "path_spacing", "mow_angle_offset_deg"}
		for _, k := range debugKeys {
			log.Printf("PostSettingsYAML: nodeMappings[%q] = %q (in payload: %v)", k, nodeMappings[k], payload[k])
		}

		// Nest back into ROS2 YAML structure
		nested := nestToROS2YAML(existing, nodeMappings, existingYAML)

		// Marshal with YAML comments header
		out, err := yaml.Marshal(nested)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: "failed to marshal YAML: " + err.Error()})
			return
		}

		// Force every schema-typed "number" key to render as a YAML float
		// even when the value is whole. gopkg.in/yaml.v3 serializes
		// float64(0) as "0", which ROS2's parameter loader then reads as
		// integer and the C++ node throws InvalidParameterTypeException
		// on startup ("setting it to {integer} is not allowed"). The same
		// node never starts again until someone manually adds the .0.
		out = forceFloatYAML(out, schema)

		header := "# Mowgli Robot Configuration — managed by mowglinext-gui\n" +
			"# This file is the single source of truth for robot parameters.\n" +
			"# Changes made here are picked up on container restart.\n\n"

		if err := os.MkdirAll(filepath.Dir(string(configFilePath)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if err := os.WriteFile(string(configFilePath), []byte(header+string(out)), 0644); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		log.Printf("Saved mowgli_robot.yaml (%d bytes)", len(out)+len(header))

		// Push the live-tunable subset to the running map_server_node so
		// outline / strip changes take effect without a container restart.
		// Errors are non-fatal — yaml is already on disk.
		liveTuneMapServer(c.Request.Context(), rosProvider, payload)

		c.JSON(200, OkResponse{})
	})
}
