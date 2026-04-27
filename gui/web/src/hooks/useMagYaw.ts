import {Imu} from "../types/ros.ts";

// Stub — danyial fork removed the magnetometer publisher pipeline during the
// 2026-04-27 robot_localization migration (Decision B). The hook signature is
// preserved so DiagnosticsPage and any other consumers compile and render
// inertly: the magnetometer card permanently shows "Stale" and "—", which is
// the honest state since /imu/mag_yaw is no longer published.
//
// TODO follow-up PR: strip the magnetometer card from DiagnosticsPage entirely
// (and the calibration_status.go mag struct, the run-mag-cal button, and any
// other dead refs surfaced by `grep mag_yaw`).
export const useMagYaw = (): { imu: Imu | null; lastMessageAt: number | null } => {
    return {imu: null, lastMessageAt: null};
};
