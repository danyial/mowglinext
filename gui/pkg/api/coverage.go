package api

import (
	"context"
	"encoding/json"
	"net/http"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// PlanCoverageGoal mirrors mowgli_interfaces/action/PlanCoverage's Goal
// section. Action types are not currently emitted by generate_go_msgs.sh
// (only msg + srv), so this is hand-maintained alongside the .action file
// at ros2/src/mowgli_interfaces/action/PlanCoverage.action.
type PlanCoverageGoal struct {
	StartPose            geometry.PoseStamped `json:"start_pose"`
	DockPose             geometry.PoseStamped `json:"dock_pose"`
	MowAngleOffsetDeg    float32              `json:"mow_angle_offset_deg"`
	ResumeFromCheckpoint bool                 `json:"resume_from_checkpoint"`
}

// PlanCoverageResult mirrors mowgli_interfaces/action/PlanCoverage's Result.
type PlanCoverageResult struct {
	Success  bool                      `json:"success"`
	Plan     []mowgli.CoverageWaypoint `json:"plan"`
	Metadata mowgli.PlanMetadata       `json:"metadata"`
	Error    mowgli.PlanError          `json:"error"`
}

// PlanCoverageActionRoute exposes the PlanCoverage action over HTTP so
// the browser can drive it without speaking the foxglove WebSocket
// protocol directly. Body shape is PlanCoverageGoal; response shape is
// PlanCoverageResult on 200 and ErrorResponse on non-200.
//
// @Summary  Run the coverage planner
// @Tags     mowglinext
// @Accept   json
// @Produce  json
// @Param    goal body PlanCoverageGoal true "Action goal"
// @Success  200 {object} PlanCoverageResult
// @Failure  400 {object} ErrorResponse
// @Failure  500 {object} ErrorResponse
// @Router   /mowglinext/action/plan-coverage [post]
func PlanCoverageActionRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/action/plan-coverage", func(c *gin.Context) {
		// Planning typically completes in <10 s but allocate ample headroom
		// for first-run validators + resume-checkpoint reads.
		ctx, cancel := context.WithTimeout(c.Request.Context(), 60*time.Second)
		defer cancel()

		var goal PlanCoverageGoal
		if err := c.ShouldBindJSON(&goal); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid PlanCoverageGoal: " + err.Error()})
			return
		}

		raw, err := provider.CallAction(
			ctx,
			"/coverage_planner_node/plan_coverage",
			&goal,
			"mowgli_interfaces/action/PlanCoverage",
		)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

		var result PlanCoverageResult
		if err := json.Unmarshal(raw, &result); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "decode action result: " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, result)
	})
}
