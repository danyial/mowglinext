// useCoveragePlan — rosbridge action client for the coverage planner.
//
// Replaces the legacy /api/mowglinext/preview-plan/* HTTP path (Plan 04).
// Calls /coverage_planner_node/plan_coverage (mowgli_interfaces/action/PlanCoverage)
// directly via the roslib websocket and converts the resulting CoverageWaypoint[]
// into a single GeoJSON FeatureCollection with `segment_type` properties for the
// MapPage Mapbox layers (D-11 colour palette).
//
// The hook owns:
// - Connection lifecycle to the rosbridge_server on ws://<host>:9090.
// - PlanCoverage goal -> result conversion to FeatureCollection.
// - Loading / Active / Idle state machine per UI-SPEC §Interaction Contract.
// - notification.error mapping for all 8 PlanError.error_code values per
//   UI-SPEC §Copywriting Contract.
//
// React JSX in MapPage auto-escapes the popup body strings produced from the
// FeatureCollection. The hook never returns HTML — only typed plain objects.
import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import type {Feature, FeatureCollection, LineString, Point, Position} from "geojson";
import {Action, Ros} from "roslib";
import {App} from "antd";
import {transpose} from "../utils/map.tsx";
import type {CoverageWaypoint, PlanError, PlanMetadata, PoseStamped} from "../types/ros.ts";

const ROSBRIDGE_PORT = 9090;
const ACTION_NAME = "/coverage_planner_node/plan_coverage";
const ACTION_TYPE = "mowgli_interfaces/action/PlanCoverage";

/**
 * Map CoverageWaypoint.segment_type uint8 constants (defined in
 * mowgli_interfaces/msg/CoverageWaypoint.msg) to the string enum used by the
 * Mapbox `match` color expression in MapPage. Must stay in lockstep with the
 * SEGMENT_* constants regenerated from the .msg file.
 */
const SEGMENT_TYPE_NAMES: Record<number, string> = {
    0: "UNDOCK",
    1: "TRANSIT",
    2: "OUTLINE_WORKING_AREA",
    3: "OUTLINE_OBSTACLE",
    4: "MOWING_BOUSTROPHEDON",
    5: "RETURN_TO_DOCK",
    6: "DOCK_APPROACH",
    7: "DOCKING",
};

/**
 * UI-SPEC §"Copywriting Contract" — exact body text per error_code.
 * Keys MUST match PlanErrorConstants in types/ros.ts.
 *   1 ERROR_NO_AREAS               -> "No mowing areas defined…"
 *   2 ERROR_AREA_TOO_NARROW        -> "One or more areas are too narrow for the robot to enter…"
 *   3 ERROR_OBSTACLE_BLOCKS_AREA   -> "An obstacle completely blocks a mowing area…"
 *   4 ERROR_DOCK_OUTSIDE_AREAS     -> "Dock position is outside all mowing and navigation areas…"
 *   5 ERROR_FOOTPRINT_VIOLATION    -> "Robot footprint does not fit safely…"
 *   6 ERROR_OBSTACLE_OFFSET_FAILED -> reuses INTERNAL copy (per plan: code-path mapping must exist)
 *   7 ERROR_RESUME_CHECKPOINT_INVALID -> "Saved progress checkpoint is corrupted…"
 *   255 ERROR_INTERNAL             -> "Internal planner error: ${human_readable}"
 */
function planErrorBody(error: PlanError | undefined): string {
    if (!error) {
        return "Internal planner error: unknown failure (no PlanError payload).";
    }
    const code = error.error_code ?? 255;
    const human = error.human_readable ?? "no detail";
    switch (code) {
        case 1:
            return "No mowing areas defined. Draw at least one mowing area on the map.";
        case 2:
            return "One or more areas are too narrow for the robot to enter. Check narrow-area strategy settings.";
        case 3:
            return "An obstacle completely blocks a mowing area. Review obstacle placement.";
        case 4:
            return "Dock position is outside all mowing and navigation areas. Reposition the dock.";
        case 5:
            return "Robot footprint does not fit safely in all planned paths. Check robot geometry settings.";
        case 6:
            // OBSTACLE_OFFSET_FAILED — share copy with INTERNAL until UX is refined,
            // but the dedicated code-path is explicitly preserved per plan instructions.
            return `Internal planner error: ${human}`;
        case 7:
            return "Saved progress checkpoint is corrupted. Clear checkpoints and restart from scratch.";
        case 255:
        default:
            return `Internal planner error: ${human}`;
    }
}

interface RequestPlanArgs {
    startPose?: PoseStamped;
    dockPose: PoseStamped;
    mowAngleOffsetDeg?: number;
    resumeFromCheckpoint?: boolean;
}

interface CoveragePlanProjection {
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
}

interface UseCoveragePlanResult {
    planGeoJson: FeatureCollection | null;
    isLoading: boolean;
    error: string | null;
    planMetadata: PlanMetadata | null;
    requestPlan: (args: RequestPlanArgs) => Promise<void>;
    clearPlan: () => void;
}

/**
 * Convert a sparse list of CoverageWaypoint poses into a Mapbox-ready
 * FeatureCollection per UI-SPEC §"GeoJSON Conversion Contract":
 * - one LineString feature per consecutive same-segment_type run of waypoints
 * - one Point feature for the first waypoint (point_type="endpoint")
 * - one Point feature for the last waypoint  (point_type="endpoint")
 * Coordinates use the canonical transpose(offsetX, offsetY, datum, y, x) helper
 * that all other plan/path features in MapPage already use.
 */
function waypointsToFeatureCollection(
    waypoints: CoverageWaypoint[],
    proj: CoveragePlanProjection,
): FeatureCollection {
    const features: Feature[] = [];
    if (waypoints.length === 0) {
        return {type: "FeatureCollection", features};
    }

    const coordOf = (wp: CoverageWaypoint): Position => {
        const x = wp.pose?.pose?.position?.x ?? 0;
        const y = wp.pose?.pose?.position?.y ?? 0;
        return transpose(proj.offsetX, proj.offsetY, proj.datum, y, x) as Position;
    };

    const segName = (wp: CoverageWaypoint): string => {
        const code = wp.segment_type ?? 1;
        const name = SEGMENT_TYPE_NAMES[code];
        if (!name) {
            console.warn(`useCoveragePlan: unknown segment_type=${code}, falling back to TRANSIT`);
            return "TRANSIT";
        }
        return name;
    };

    for (let i = 1; i < waypoints.length; i += 1) {
        const prev = waypoints[i - 1];
        const curr = waypoints[i];
        if ((prev.segment_type ?? 1) !== (curr.segment_type ?? 1)) {
            // Bridge the boundary with a TRANSIT line so the polyline has no
            // visible gap at segment-type boundaries; the Mapbox color match
            // expression renders it grey via the TRANSIT case.
            const lineFeature: Feature<LineString> = {
                type: "Feature",
                geometry: {
                    type: "LineString",
                    coordinates: [coordOf(prev), coordOf(curr)],
                },
                properties: {
                    segment_type: "TRANSIT",
                    blade_enabled: false,
                    speed: curr.speed ?? 0,
                },
            };
            features.push(lineFeature);
            continue;
        }
        const lineFeature: Feature<LineString> = {
            type: "Feature",
            geometry: {
                type: "LineString",
                coordinates: [coordOf(prev), coordOf(curr)],
            },
            properties: {
                segment_type: segName(curr),
                blade_enabled: curr.blade_enabled ?? false,
                speed: curr.speed ?? 0,
            },
        };
        features.push(lineFeature);
    }

    const first = waypoints[0];
    const last = waypoints[waypoints.length - 1];
    const startPoint: Feature<Point> = {
        type: "Feature",
        geometry: {type: "Point", coordinates: coordOf(first)},
        properties: {
            point_type: "endpoint",
            segment_type: segName(first),
            sequence_id: first.sequence_id ?? 0,
        },
    };
    const endPoint: Feature<Point> = {
        type: "Feature",
        geometry: {type: "Point", coordinates: coordOf(last)},
        properties: {
            point_type: "endpoint",
            segment_type: segName(last),
            sequence_id: last.sequence_id ?? waypoints.length - 1,
        },
    };
    features.push(startPoint, endPoint);

    return {type: "FeatureCollection", features};
}

/**
 * Resolve the rosbridge WebSocket URL. In dev we still hit the same host (the
 * developer is expected to expose port 9090 from the mowgli-ros2 container);
 * in prod we use the page host. https pages get wss://, http pages get ws://.
 */
function resolveRosbridgeUrl(): string {
    if (typeof window === "undefined") {
        return `ws://localhost:${ROSBRIDGE_PORT}`;
    }
    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    return `${proto}://${window.location.hostname}:${ROSBRIDGE_PORT}`;
}

export function useCoveragePlan(proj: CoveragePlanProjection): UseCoveragePlanResult {
    const {notification} = App.useApp();
    const [planGeoJson, setPlanGeoJson] = useState<FeatureCollection | null>(null);
    const [planMetadata, setPlanMetadata] = useState<PlanMetadata | null>(null);
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // The Ros object owns the websocket; create lazily on first request.
    const rosRef = useRef<Ros | null>(null);
    const inFlightRef = useRef<boolean>(false);
    const progressNotificationKeyRef = useRef<string | null>(null);

    const ensureRos = useCallback((): Promise<Ros> => {
        return new Promise((resolve, reject) => {
            if (rosRef.current && rosRef.current.isConnected) {
                resolve(rosRef.current);
                return;
            }
            const ros = rosRef.current ?? new Ros({url: resolveRosbridgeUrl()});
            rosRef.current = ros;
            if (ros.isConnected) {
                resolve(ros);
                return;
            }
            type RosLike = {
                on: (event: string, cb: (e: unknown) => void) => void;
                off?: (event: string, cb: (e: unknown) => void) => void;
                removeListener?: (event: string, cb: (e: unknown) => void) => void;
            };
            const rosEmitter = ros as unknown as RosLike;
            const detach = (event: string, cb: (e: unknown) => void) => {
                if (typeof rosEmitter.off === "function") {
                    rosEmitter.off(event, cb);
                } else if (typeof rosEmitter.removeListener === "function") {
                    rosEmitter.removeListener(event, cb);
                }
            };
            const onConnection = () => {
                detach("error", onError);
                resolve(ros);
            };
            const onError = (err: unknown) => {
                detach("connection", onConnection);
                reject(err instanceof Error ? err : new Error("rosbridge connection failed"));
            };
            rosEmitter.on("connection", onConnection);
            rosEmitter.on("error", onError);
            // Force open if not already attempted.
            try {
                ros.connect(resolveRosbridgeUrl());
            } catch (e) {
                reject(e instanceof Error ? e : new Error(String(e)));
            }
        });
    }, []);

    useEffect(() => {
        return () => {
            try {
                rosRef.current?.close();
            } catch {
                // ignore on teardown
            }
            rosRef.current = null;
        };
    }, []);

    const closeProgressNotification = useCallback(() => {
        if (progressNotificationKeyRef.current) {
            notification.destroy(progressNotificationKeyRef.current);
            progressNotificationKeyRef.current = null;
        }
    }, [notification]);

    const clearPlan = useCallback(() => {
        setPlanGeoJson(null);
        setPlanMetadata(null);
        setError(null);
        closeProgressNotification();
    }, [closeProgressNotification]);

    const requestPlan = useCallback(
        async (args: RequestPlanArgs) => {
            if (inFlightRef.current) {
                // Re-entrant clicks are dropped — UI-SPEC: AsyncButton's loading
                // prop disables the button during the in-flight request.
                return;
            }
            inFlightRef.current = true;
            setIsLoading(true);
            setError(null);

            const progressKey = `coverage-plan-progress-${Date.now()}`;
            progressNotificationKeyRef.current = progressKey;
            notification.open({
                key: progressKey,
                message: "Planning in progress…",
                description: "Generating coverage plan from the map server. This can take 5–30 s.",
                duration: 0,
            });

            let ros: Ros;
            try {
                ros = await ensureRos();
            } catch (e) {
                inFlightRef.current = false;
                setIsLoading(false);
                closeProgressNotification();
                const description = e instanceof Error ? e.message : String(e);
                setError(description);
                notification.error({
                    message: "Plan generation failed",
                    description: `Internal planner error: rosbridge connection failed (${description}).`,
                });
                return;
            }

            type PlanResult = {
                success?: boolean;
                plan?: CoverageWaypoint[];
                metadata?: PlanMetadata;
                error?: PlanError;
            };
            type PlanFeedback = {progress_percent?: number; phase?: string};
            type PlanGoal = {
                start_pose: PoseStamped;
                dock_pose: PoseStamped;
                mow_angle_offset_deg: number;
                resume_from_checkpoint: boolean;
            };

            const goal: PlanGoal = {
                start_pose: args.startPose ?? {
                    pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
                },
                dock_pose: args.dockPose,
                mow_angle_offset_deg: args.mowAngleOffsetDeg ?? -1.0,
                resume_from_checkpoint: args.resumeFromCheckpoint ?? false,
            };

            const actionClient = new Action<PlanGoal, PlanFeedback, PlanResult>({
                ros,
                name: ACTION_NAME,
                actionType: ACTION_TYPE,
            });

            return new Promise<void>((resolve) => {
                let finished = false;
                const finish = () => {
                    if (finished) return;
                    finished = true;
                    inFlightRef.current = false;
                    setIsLoading(false);
                    closeProgressNotification();
                    resolve();
                };

                try {
                    actionClient.sendGoal(
                        goal,
                        // result callback
                        (result: PlanResult) => {
                            if (result.success) {
                                const fc = waypointsToFeatureCollection(result.plan ?? [], proj);
                                setPlanGeoJson(fc);
                                setPlanMetadata(result.metadata ?? null);
                                setError(null);
                                const n = result.plan?.length ?? 0;
                                const m = result.metadata?.processed_area_indices?.length ?? 0;
                                notification.success({
                                    message: `Plan ready — ${n} waypoints, ${m} areas`,
                                });
                            } else {
                                const description = planErrorBody(result.error);
                                setPlanGeoJson(null);
                                setPlanMetadata(null);
                                setError(description);
                                notification.error({
                                    message: "Plan generation failed",
                                    description,
                                });
                            }
                            finish();
                        },
                        // feedback callback
                        (_feedback: PlanFeedback) => {
                            // Feedback is informational only; AsyncButton's spinner already covers the wait.
                        },
                        // failed callback
                        (failure: string) => {
                            const description = failure || "Action server unreachable.";
                            setPlanGeoJson(null);
                            setPlanMetadata(null);
                            setError(description);
                            notification.error({
                                message: "Plan generation failed",
                                description: `Internal planner error: ${description}`,
                            });
                            finish();
                        },
                    );
                } catch (e) {
                    const description = e instanceof Error ? e.message : String(e);
                    setError(description);
                    notification.error({
                        message: "Plan generation failed",
                        description: `Internal planner error: ${description}`,
                    });
                    finish();
                }
            });
        },
        [ensureRos, notification, closeProgressNotification, proj],
    );

    return useMemo(
        () => ({planGeoJson, isLoading, error, planMetadata, requestPlan, clearPlan}),
        [planGeoJson, isLoading, error, planMetadata, requestPlan, clearPlan],
    );
}
