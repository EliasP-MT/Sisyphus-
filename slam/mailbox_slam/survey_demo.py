"""Synthetic four-mailbox initialization demo. Never opens hardware."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from letterbox_locator import load_json

from .discovery import ROOT, MailboxDiscovery
from .geometry import integrate, inverse, planar
from .mapping import write_json
from .survey import MailboxSurvey, SurveySettings


def synthetic_config():
    """Deliberately synthetic rig. NEVER use these values on the ElRobot."""
    camera_mount = np.eye(4)
    camera_mount[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    camera_mount[:3, 3] = [0.1, 0, 0.3]
    wheels = []
    for i, angle in enumerate(np.radians([0, 120, 240])):
        wheels.append(
            dict(
                name=str(i),
                position_xy_m=[0.15 * np.cos(angle), 0.15 * np.sin(angle)],
                rolling_direction_xy=[-np.sin(angle), np.cos(angle)],
                radius_m=0.05,
                ticks_per_revolution=4096.0,
                sign=1,
                modulus=4096.0,
                max_rad_s=20.0,
            )
        )
    return dict(
        camera=dict(
            fx=700.0,
            fy=710.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
            distortion=[0.03, -0.02, 0.0, 0.0, 0.0],
        ),
        arm=dict(
            T_base_arm=np.eye(4).tolist(),
            T_last_camera=camera_mount.tolist(),
            joints=[
                dict(
                    name="joint1",
                    T_parent_joint=np.eye(4).tolist(),
                    axis=[0, 0, 1],
                    zero_tick=2048.0,
                    ticks_per_revolution=4096.0,
                    sign=1,
                    modulus=4096.0,
                    max_rad_s=10.0,
                )
            ],
        ),
        wheels=wheels,
        settings=dict(max_sample_age_s=0.15, max_encoder_gap_s=0.5),
    )


def run_demo(output, events_path=None):
    config = synthetic_config()
    config["settings"]["max_frames"] = 500
    slam = MailboxDiscovery(config)
    settings = SurveySettings(
        scan_step_rad=math.pi / 6,
        settle_s=0.1,
        approach_speed_m_s=0.12,
        image_interval_s=0.8,
        max_visual_age_s=0.9,
    )
    workflow = MailboxSurvey(slam, settings)
    ticks, pose, timestamp = np.ones(3) * 2048, np.zeros(3), 0.0
    slam.add_wheel_sample(0, ticks)
    slam.add_joint_sample(0, [2048.0])
    workflow.start(0)
    events = [
        dict(type="wheels", timestamp=0.0, positions=ticks.tolist()),
        dict(type="joints", timestamp=0.0, positions=[2048.0]),
        dict(type="start", timestamp=0.0),
    ]
    landmarks = []
    recipients = load_json(ROOT / "config/recipients.json")[:4]
    for angle, name in zip(np.radians([0, 90, 180, 270]), recipients):
        tag = np.eye(4)
        tag[:3, :3] = Rotation.from_rotvec([0, 0, angle]).as_matrix() @ np.array(
            [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        )
        tag[:3, 3] = [np.cos(angle), np.sin(angle), 0.3]
        landmarks.append((tag, name))
    for _ in range(12000):
        proposal = workflow.step(timestamp)
        events.append(dict(type="tick", timestamp=timestamp))
        if proposal["capture"]:
            camera = planar(pose) @ slam._fixed_camera_pose()
            observations, observed_names = [], []
            for tag, name in landmarks:
                measurement = inverse(camera) @ tag
                x, y, z = measurement[:3, 3]
                if z > 0 and abs(np.arctan2(x, z)) < 0.4 and abs(np.arctan2(y, z)) < 0.3:
                    observations.append(
                        dict(
                            label_center_m=measurement[:3, 3].tolist(),
                            label_rotation=measurement[:3, :3].tolist(),
                            reprojection_px=0.1,
                            label_corners_px=[[100, 100], [160, 100], [160, 150], [100, 150]],
                        )
                    )
                    observed_names.append(name)
            text = None
            if proposal["capture"] == "name" and observations:
                # Simulated text selected by physical target position, never by its map name.
                target = slam.graph.landmarks[proposal["target_id"]]["pose"][:3, 3]
                text = min(landmarks, key=lambda item: np.linalg.norm(item[0][:3, 3] - target))[1]
            workflow.observe_results(observations, timestamp, ocr_text=text)
            events.append(
                dict(type="locator", timestamp=timestamp, results=observations, ocr_text=text)
            )
        if workflow.phase in ("complete", "incomplete", "fault"):
            break
        dt = 0.05
        delta = np.array(proposal["velocity_base"]) * dt
        pose = integrate(pose, delta)
        distances = slam.wheels.kinematics @ delta
        ticks = (ticks + distances / 0.05 * 4096 / (2 * np.pi)) % 4096
        timestamp += dt
        slam.add_wheel_sample(timestamp, ticks)
        slam.add_joint_sample(timestamp, [2048.0])
        events += [
            dict(type="wheels", timestamp=timestamp, positions=ticks.tolist()),
            dict(type="joints", timestamp=timestamp, positions=[2048.0]),
        ]
    payload = slam.map_payload()
    payload["initialization"] = workflow.status()
    payload["synthetic"] = True
    write_json(output, payload)
    if events_path is not None:
        path = Path(events_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(json.dumps(e) + "\n" for e in events))
        path.with_suffix(".config.json").write_text(json.dumps(config, indent=2) + "\n")
        path.with_suffix(".survey.json").write_text(json.dumps(settings.__dict__, indent=2) + "\n")
    return dict(
        synthetic=True,
        phase=workflow.phase,
        simulated_seconds=timestamp,
        discovered=len(slam.graph.landmarks),
        named=sum(m["name_confirmed"] for m in slam.mailboxes()),
        scan_rotation_rad=workflow.yaw_progress,
        failed=workflow.failed,
        output=str(output),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("synthetic-initialization.json"))
    parser.add_argument("--events", type=Path)
    args = parser.parse_args()
    print(json.dumps(run_demo(args.output, args.events), indent=2))


if __name__ == "__main__":
    main()
