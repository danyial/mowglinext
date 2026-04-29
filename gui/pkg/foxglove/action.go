package foxglove

import (
	"context"
	"crypto/rand"
	"encoding/json"
	"fmt"
)

// ROS2 action_msgs/msg/GoalStatus status enum.
const (
	ActionStatusUnknown   = 0
	ActionStatusAccepted  = 1
	ActionStatusExecuting = 2
	ActionStatusCanceling = 3
	ActionStatusSucceeded = 4
	ActionStatusCanceled  = 5
	ActionStatusAborted   = 6
)

// goalUUID is the JSON shape of unique_identifier_msgs/msg/UUID.
// Wire format is a 16-element uint8 array under field "uuid". `[]int` is
// used (rather than `[]byte`) because encoding/json default-encodes byte
// slices as base64 strings, which the foxglove CDR serializer does not
// expect for primitive uint8 arrays.
type goalUUID struct {
	UUID []int `json:"uuid"`
}

func newGoalUUID() goalUUID {
	var b [16]byte
	_, _ = rand.Read(b[:])
	arr := make([]int, 16)
	for i, v := range b {
		arr[i] = int(v)
	}
	return goalUUID{UUID: arr}
}

// CallAction invokes a ROS2 action via the synthetic services that every
// rclcpp_action server exposes (foxglove_bridge advertises them like any
// other service). It blocks until the result is available or ctx fires.
//
// Flow:
//  1. POST to `<action>/_action/send_goal` with `{goal_id, goal}`.
//     Expect `{accepted: true, ...}` back.
//  2. POST to `<action>/_action/get_result` with `{goal_id}`. The server
//     does not respond until the action reaches a terminal state, so
//     this call naturally blocks for the duration of the action.
//  3. Inspect the response status; return the embedded `result` payload
//     iff status == ActionStatusSucceeded.
//
// `actionType` is the .action package path (e.g.
// "mowgli_interfaces/action/PlanCoverage"). This function does NOT
// subscribe to the action's feedback or status topics — Preview Plan
// is a request/response flow from the GUI's perspective.
func (c *Client) CallAction(
	ctx context.Context,
	action string,
	goal interface{},
	actionType string,
) (json.RawMessage, error) {
	id := newGoalUUID()

	// --- 1. send_goal ----------------------------------------------------
	sendGoalReq := struct {
		GoalID goalUUID    `json:"goal_id"`
		Goal   interface{} `json:"goal"`
	}{GoalID: id, Goal: goal}

	rawSend, err := c.CallService(
		ctx,
		action+"/_action/send_goal",
		sendGoalReq,
		actionType+"_SendGoal",
	)
	if err != nil {
		return nil, fmt.Errorf("action send_goal: %w", err)
	}

	var sendResp struct {
		Accepted bool `json:"accepted"`
	}
	if err := json.Unmarshal(rawSend, &sendResp); err != nil {
		return nil, fmt.Errorf("action send_goal unmarshal: %w", err)
	}
	if !sendResp.Accepted {
		return nil, fmt.Errorf("action %s rejected goal", action)
	}

	// --- 2. get_result (blocks server-side until terminal status) --------
	getResultReq := struct {
		GoalID goalUUID `json:"goal_id"`
	}{GoalID: id}

	rawResult, err := c.CallService(
		ctx,
		action+"/_action/get_result",
		getResultReq,
		actionType+"_GetResult",
	)
	if err != nil {
		return nil, fmt.Errorf("action get_result: %w", err)
	}

	var resultEnv struct {
		Status int             `json:"status"`
		Result json.RawMessage `json:"result"`
	}
	if err := json.Unmarshal(rawResult, &resultEnv); err != nil {
		return nil, fmt.Errorf("action get_result unmarshal: %w", err)
	}

	switch resultEnv.Status {
	case ActionStatusSucceeded:
		return resultEnv.Result, nil
	case ActionStatusCanceled:
		return nil, fmt.Errorf("action %s canceled", action)
	case ActionStatusAborted:
		return nil, fmt.Errorf("action %s aborted by server", action)
	default:
		return nil, fmt.Errorf("action %s ended with non-success status %d", action, resultEnv.Status)
	}
}
