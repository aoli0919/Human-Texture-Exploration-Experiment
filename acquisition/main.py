"""Capture ten texture trials. Timestamps describe sequential reads, not synchronization."""

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import re
import time


@dataclass(frozen=True)
class Config:
    session_id: str | None = None
    output_dir: str | None = None
    serial_port: str = "/dev/ttyUSB0"
    camera_index: int = 0
    duration: float = 10.0
    width: int = 640
    height: int = 480
    fps: float = 30.0
    seed: int | None = None

    def validate(self, dry_run=False):
        if not dry_run and (not self.session_id or not self.output_dir):
            raise ValueError("Hardware mode requires --session-id and --output-dir")
        if self.session_id is not None and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", self.session_id):
            raise ValueError("session-id must be an anonymous 1–64 character identifier: letters, digits, _ or -")
        if not self.serial_port.strip():
            raise ValueError("serial-port must not be empty")
        if self.output_dir is not None and not str(self.output_dir).strip():
            raise ValueError("output-dir must not be empty")
        if self.camera_index < 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("camera-index must be nonnegative; width and height must be positive")
        if not all(math.isfinite(value) and value > 0 for value in (self.duration, self.fps)):
            raise ValueError("duration and fps must be finite and positive")


def session_plan(config):
    order = random.Random(config.seed).sample(range(1, 11), 10)
    return {"config": asdict(config), "trial_order": order,
            "session_directory": str(Path(config.output_dir) / config.session_id)
            if config.output_dir and config.session_id else None}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def run_session(config, *, cv=None, sensor_factory=None, monotonic=time.monotonic,
                timestamp=time.time):
    """Run acquisition; injectable devices/clocks support hardware-free regression tests.

    Enter starts a trial. Q cancels the whole session and saves the current partial
    trial. Camera frames and force reads happen sequentially; requested FPS is a
    camera/encoder setting and does not guarantee the actual sampling frequency.
    """
    config.validate()
    if cv is None:
        import cv2 as cv
    if sensor_factory is None:
        from .forcesensor import FTSensorSerial
        sensor_factory = FTSensorSerial

    plan = session_plan(config)
    directory = Path(plan["session_directory"])
    directory.mkdir(parents=True, exist_ok=False)
    metadata = {**plan, "started_at": utc_now(), "status": "running", "trials": [],
                "timing_note": "Sequential camera/force reads; no hardware synchronization. FPS is requested only."}
    camera = sensor = writer = active = None
    rows = []

    def finish_trial(status):
        nonlocal writer, active, rows
        if active is None:
            return
        try:
            if writer is not None:
                writer.release()
        finally:
            writer = None
            active.update(status=status, ended_at=timestamp(), force_samples=len(rows))
            with (directory / active["force_file"]).open("w", newline="", encoding="utf-8") as stream:
                output = csv.writer(stream)
                output.writerow(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Timestamp"])
                output.writerows(rows)
            active = None
            rows = []

    try:
        camera = cv.VideoCapture(config.camera_index)
        if not camera.isOpened():
            raise RuntimeError("Could not open camera")
        camera.set(cv.CAP_PROP_FRAME_WIDTH, config.width)
        camera.set(cv.CAP_PROP_FRAME_HEIGHT, config.height)
        camera.set(cv.CAP_PROP_FPS, config.fps)
        sensor = sensor_factory(port=config.serial_port)
        sensor.set_zero_ref()
        print("Enter: start displayed texture trial. Q: stop session and save partial trial.")
        for texture in plan["trial_order"]:
            print(f"Texture {texture}: press Enter to record")
            while True:
                ok, frame = camera.read()
                frame_time = timestamp()
                if not ok:
                    raise RuntimeError("Camera stopped returning frames")
                cv.putText(frame, f"Texture {texture} | Enter: record | Q: quit", (10, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv.imshow("Texture acquisition", frame)
                key = cv.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    metadata["status"] = "cancelled"
                    finish_trial("partial")
                    return metadata
                if active is None and key in (10, 13):
                    height, width = frame.shape[:2]
                    active = {"texture": texture, "started_at": timestamp(),
                              "frame_size": [width, height], "frame_timestamps": [],
                              "video_file": f"video_{texture}.mp4",
                              "force_file": f"force_data_{texture}.csv"}
                    metadata["trials"].append(active)
                    writer = cv.VideoWriter(str(directory / active["video_file"]),
                                            cv.VideoWriter_fourcc(*"mp4v"), config.fps, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError("Could not open video writer")
                    started = monotonic()
                if active is not None:
                    writer.write(frame)
                    active["frame_timestamps"].append(frame_time)
                    forces = list(sensor.get_ft())
                    if len(forces) != 6:
                        raise RuntimeError("Expected six force/torque values")
                    rows.append(forces + [timestamp()])
                    if monotonic() - started >= config.duration:
                        finish_trial("complete")
                        break
        metadata["status"] = "complete"
        return metadata
    except BaseException as error:
        metadata["status"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "error"
        metadata["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        # Nested finally blocks keep later resources reachable even if cleanup fails.
        try:
            finish_trial("partial")
        finally:
            try:
                if camera is not None:
                    camera.release()
            finally:
                try:
                    if sensor is not None:
                        sensor.close()
                finally:
                    try:
                        cv.destroyAllWindows()
                    finally:
                        metadata["ended_at"] = utc_now()
                        (directory / "session_metadata.json").write_text(
                            json.dumps(metadata, indent=2), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", help="Anonymous session token; do not use a participant name")
    parser.add_argument("--output-dir", help="Parent directory; a new session-id subdirectory is created")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--dry-run", action="store_true", help="Print configuration/order without hardware or filesystem writes")
    args = vars(parser.parse_args(argv))
    dry_run = args.pop("dry_run")
    config = Config(**args)
    try:
        config.validate(dry_run=dry_run)
    except ValueError as error:
        parser.error(str(error))
    if dry_run:
        print(json.dumps(session_plan(config), indent=2))
        return 0
    run_session(config)
    return 0


if __name__ == "__main__":
    main()
