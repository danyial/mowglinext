import {useEffect, useRef, useState} from "react";
import {Imu} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

/**
 * Subscribes to /imu/mag_yaw via the Go backend's "magYaw" virtual topic
 * (foxglove relay, see gui/pkg/providers/ros.go). The publisher is
 * mag_yaw_publisher.py in mowgli_localization — the same pipeline whose
 * yaw is fused as imu2 by ekf_map_node. The DiagnosticsPage Magnetometer
 * card consumes {imu, lastMessageAt} for the yaw / sigma readout and the
 * staleness indicator.
 */
export const useMagYaw = (): { imu: Imu | null; lastMessageAt: number | null } => {
    const [imu, setImu] = useState<Imu | null>(null);
    const lastRef = useRef<number | null>(null);
    const [lastMessageAt, setLastMessageAt] = useState<number | null>(null);

    const stream = useWS<string>(
        () => { console.log({message: "MagYaw Stream closed"}); },
        () => { console.log({message: "MagYaw Stream connected"}); },
        (e) => {
            try {
                const parsed = JSON.parse(e) as Imu;
                setImu(parsed);
                const now = Date.now();
                lastRef.current = now;
                setLastMessageAt(now);
            } catch (err) {
                console.warn("useMagYaw: failed to parse Imu payload", err);
            }
        }
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/magYaw");
        return () => { stream.stop(); };
    }, []);

    return {imu, lastMessageAt};
};
