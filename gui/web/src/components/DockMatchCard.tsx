import {useEffect, useState} from "react";
import {Badge, Button, Card, Modal, Space, Typography, notification} from "antd";
import {AimOutlined, ReloadOutlined} from "@ant-design/icons";
import {useDockMatch} from "../hooks/useDockMatch.ts";

const {Text} = Typography;

/**
 * Confidence-badge thresholds (D-10 lock):
 *   GREEN  — trusted == true (Plan 02-04 R-3 gate already factors inlier >= 0.70 AND rmse <= 0.05)
 *   YELLOW — 0.60 <= inlier < 0.70 OR 0.05 < rmse <= 0.10 (border zone, pose not published but conf live)
 *   RED    — inlier < 0.60 OR rmse > 0.10 (degraded match)
 *   GRAY   — no /dock_match/confidence received OR last message > 5 s old (matcher down / use_lidar=false)
 *
 * Status returned via the antd Badge `status` prop:
 *   success | warning | error | default
 */
type Status = "success" | "warning" | "error" | "default";

const STALE_AFTER_MS = 5_000;

const computeStatus = (
    inlier: number | undefined,
    rmse: number | undefined,
    trusted: boolean | undefined,
    lastMessageAt: number | null,
): {status: Status; label: string} => {
    if (lastMessageAt == null) {
        return {status: "default", label: "no data"};
    }
    if (Date.now() - lastMessageAt > STALE_AFTER_MS) {
        return {status: "default", label: "stale"};
    }
    if (trusted === true) {
        return {status: "success", label: "trusted"};
    }
    const i = inlier ?? 0;
    const r = rmse ?? Infinity;
    if (i >= 0.6 && i < 0.7) return {status: "warning", label: "borderline"};
    if (r > 0.05 && r <= 0.10) return {status: "warning", label: "borderline"};
    return {status: "error", label: "degraded"};
};

/**
 * DockMatchCard — operator-visible LiDAR-dock-match card (D-09 extends the
 * dock area, D-10 status badge + numeric pair, D-11 Recapture confirm modal,
 * D-12 Go-relay topicMap pattern via useDockMatch hook).
 *
 * Renders gracefully when /dock_match/* topics are absent (Plan 02-04 not
 * deployed, or use_lidar:=false): GRAY badge + "—" placeholders. The
 * Recapture button still triggers the existing /api/calibration/imu-yaw flow
 * which runs the dock_scan_capture sub-step on the dock pre-phase.
 */
export const DockMatchCard = () => {
    const {confidence, lastMessageAt} = useDockMatch();
    const [tick, setTick] = useState(0);
    const [recapturing, setRecapturing] = useState(false);

    // Force re-render every 1 s so the staleness gate flips visibly without
    // requiring a new message.
    useEffect(() => {
        const id = setInterval(() => setTick((n) => (n + 1) % 1_000_000), 1_000);
        return () => clearInterval(id);
    }, []);
    void tick;

    const inlier = confidence?.inlier_ratio;
    const rmse = confidence?.rmse_m;
    const trusted = confidence?.trusted;
    const {status, label} = computeStatus(inlier, rmse, trusted, lastMessageAt);

    const inlierStr = inlier != null ? `${(inlier * 100).toFixed(0)}%` : "—";
    const rmseStr = rmse != null ? `${(rmse * 100).toFixed(1)} cm` : "—";

    const triggerRecapture = async () => {
        setRecapturing(true);
        try {
            const res = await fetch("/api/calibration/imu-yaw", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({duration_sec: 30}),
            });
            if (!res.ok) {
                throw new Error(`HTTP ${res.status}: ${await res.text()}`);
            }
            notification.success({
                message: "Dock-scan recapture started",
                description:
                    "Calibration drive running — dock_scan_capture sub-step will write the new dock_scan.pcd. " +
                    "DockScanMatchNode picks it up via the mtime watcher within ~100 ms (no GUI restart).",
            });
        } catch (e) {
            notification.error({
                message: "Recapture failed",
                description: e instanceof Error ? e.message : String(e),
            });
        } finally {
            setRecapturing(false);
        }
    };

    const onRecaptureClick = () => {
        Modal.confirm({
            title: "Dock-Scan neu aufnehmen?",
            content:
                "Capture wird den aktuellen LiDAR-Snapshot als neuen dock_scan.pcd speichern. " +
                "Roboter sollte sicher auf dem Dock sitzen. Fortfahren?",
            okText: "Ja, neu aufnehmen",
            okType: "primary",
            cancelText: "Abbrechen",
            onOk: triggerRecapture,
        });
    };

    return (
        <Card
            size="small"
            title={<Space><AimOutlined/> LiDAR Dock Match</Space>}
            extra={
                <Button
                    size="small"
                    icon={<ReloadOutlined/>}
                    loading={recapturing}
                    onClick={onRecaptureClick}
                >
                    Recapture
                </Button>
            }
        >
            <Space direction="vertical" size={6} style={{width: "100%"}}>
                <Space size={8}>
                    <Badge status={status} text={label}/>
                </Space>
                <Text style={{fontSize: 12}}>
                    Inlier {inlierStr} / RMSE {rmseStr}
                </Text>
                {/* TODO(phase-3): wire dock_scan_meta.captured_at age via Go relay */}
                <Text type="secondary" style={{fontSize: 11}}>
                    Scan age: —
                </Text>
                {/* TODO(phase-3): wire lateral_error_at_contact via /api/mowglinext/sessions/last/lateral_error endpoint */}
                <Text type="secondary" style={{fontSize: 11}}>
                    Last fine-dock lateral: —
                </Text>
            </Space>
        </Card>
    );
};
