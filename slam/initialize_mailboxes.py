"""Replay or integrate fixed-arm mailbox initialization; no hardware is opened.

For live use, import MailboxDiscovery/MailboxSurvey from this module and connect
existing encoder, arm-camera and watchdog-controlled base interfaces. CLI replay
prints velocity proposals only; it never sends them to a robot.
"""

import argparse
import importlib
import json
from dataclasses import asdict
from pathlib import Path

import cv2

from letterbox_locator import load_json
from mailbox_slam.discovery import ROOT, MailboxDiscovery
from mailbox_slam.mapping import write_json
from mailbox_slam.survey import MailboxSurvey, SurveySettings

__all__ = ["MailboxDiscovery", "MailboxSurvey", "SurveySettings", "replay"]


def replay(slam, events_path, *, settings=None, on_proposal=None):
    workflow = MailboxSurvey(slam, settings)
    path = Path(events_path)
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
            kind, t = event["type"], event["timestamp"]
            if kind == "wheels":
                slam.add_wheel_sample(t, event["positions"])
            elif kind == "joints":
                slam.add_joint_sample(t, event["positions"], event.get("units", "ticks"))
            elif kind == "start":
                workflow.start(t)
            elif kind == "tick":
                proposal = workflow.step(t)
                if on_proposal is not None:
                    on_proposal(proposal)
            elif kind == "image":
                image_path = path.parent / event["path"]
                image = cv2.imread(str(image_path))
                if image is None:
                    raise ValueError(f"Cannot read {image_path}")
                workflow.observe(image, t)
            elif kind == "locator":
                workflow.observe_results(event["results"], t, ocr_text=event.get("ocr_text"))
            else:
                raise ValueError(f"Unknown event type: {kind}")
        except (KeyError, ValueError, TypeError) as exc:
            raise ValueError(f"Event line {line_number}: {exc}") from exc
    return workflow


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--constants", help="Import Python module exposing SLAM_CONFIG")
    source.add_argument("--config", type=Path, help="Measured arm/wheel configuration JSON")
    parser.add_argument(
        "--calibration",
        type=Path,
        help="Override arm intrinsics; uses config/arm-camera-intrinsics.json if missing in constants",
    )
    parser.add_argument("--geometry", type=Path, default=ROOT / "config/letterbox.json")
    parser.add_argument("--recipients", type=Path, default=ROOT / "config/recipients.json")
    parser.add_argument("--settings", type=Path, help="SurveySettings overrides as JSON")
    parser.add_argument(
        "--events", type=Path, required=True, help="Recorded telemetry/capture workflow JSONL"
    )
    parser.add_argument("--output", type=Path, default=Path("initialized-mailboxes.json"))
    parser.add_argument("--print-proposals", action="store_true")
    parser.add_argument(
        "--hardcode",
        type=int,
        metavar="N",
        help="Discovery-only full turn, expecting exactly N mailboxes",
    )
    parser.add_argument("--scan-only", action="store_true")
    args = parser.parse_args(argv)
    if args.hardcode is not None and args.hardcode <= 0:
        parser.error("--hardcode must be positive")
    try:
        config = (
            load_json(args.config)
            if args.config
            else importlib.import_module(args.constants).SLAM_CONFIG.copy()
        )
        if args.calibration:
            config["camera"] = load_json(args.calibration)
        elif config.get("camera", {}).get("fx") is None:
            config["camera"] = load_json(ROOT / "config/arm-camera-intrinsics.json")
        slam = MailboxDiscovery(
            config, geometry=load_json(args.geometry), recipients=load_json(args.recipients)
        )
        settings_data = load_json(args.settings) if args.settings else {}
        if args.hardcode is not None or args.scan_only:
            settings_data.update(discovery_only=True, expected_mailboxes=args.hardcode)
        settings = SurveySettings(**settings_data)
        report_proposal = (
            (lambda p: print(json.dumps(p), flush=True)) if args.print_proposals else None
        )
        workflow = replay(slam, args.events, settings=settings, on_proposal=report_proposal)
        payload = slam.map_payload()
        payload["initialization"] = workflow.status()
        payload["survey_settings"] = asdict(settings)
        write_json(args.output, payload)
        print(
            json.dumps(
                dict(
                    phase=workflow.phase,
                    discovered=len(slam.mailboxes()),
                    named=sum(m["name_confirmed"] for m in slam.mailboxes()),
                    output=str(args.output),
                )
            )
        )
    except (OSError, ValueError, KeyError, ImportError, TypeError, cv2.error) as exc:
        parser.exit(2, f"{exc}\n")


if __name__ == "__main__":
    main()
