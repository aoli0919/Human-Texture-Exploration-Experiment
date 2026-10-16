# Collecting new texture trials

The `acquisition` CLI replaces hardcoded settings in the original collection scripts, preserved under `legacy/acquisition/`. It records camera video and six force/torque channels for ten texture labels. This is separate from analysing the existing dataset.

## Inspect a plan without hardware

From the repository root, using Python 3.11 or later:

```bash
python -m acquisition --dry-run --seed 42
python -m acquisition --help
```

Dry-run prints configuration and a randomized permutation of labels 1–10. It does not import device dependencies, open devices, or write files. A fixed seed reproduces the new CLI's planned order, not historical presentation order.

## Hardware setup

Install optional dependencies into the same virtual environment:

```bash
python -m pip install -e '.[acquisition]'
```

Hardware mode requires a compatible camera, an accessible RobotiQ FT300-S serial connection, and a graphical desktop for the OpenCV preview. `/dev/ttyUSB0` is a Linux device-path default; select your actual device. The adapter uses the FT300-S serial protocol and acquires a zero reference before trials. Check the unloaded sensor, mounting, and material-number mapping for the experimental setup.

```bash
python -m acquisition --session-id pilot001 --output-dir outputs/acquisition --serial-port /dev/ttyUSB0 --camera-index 0 --duration 10 --fps 30 --seed 42
```

This creates `outputs/acquisition/pilot001/`. Use an anonymous 1–64 character session token containing letters, digits, underscores, or hyphens, beginning with a letter or digit. Do not enter a participant name. Hardware mode requires both `--session-id` and `--output-dir`. The session directory must not exist; use a new token to avoid overwrites.

## Controls and flags

The preview displays the current texture. Press **Enter** to start its trial. Recording stops after the requested duration and the next label appears. Press **Q** to stop the whole session; an active trial is saved as partial. Metadata records completion, cancellation, interruption, or errors.

| Flag | Default | Purpose |
| --- | --- | --- |
| `--session-id` | Required for hardware | Anonymous session directory name |
| `--output-dir` | Required for hardware | Parent output directory |
| `--serial-port` | `/dev/ttyUSB0` | Sensor serial device |
| `--camera-index` | `0` | OpenCV camera index |
| `--duration` | `10` | Trial duration, seconds |
| `--width`, `--height` | `640`, `480` | Requested capture dimensions |
| `--fps` | `30` | Requested camera/encoder frame rate |
| `--seed` | Unset | Seed for the randomized plan |
| `--dry-run` | Off | Print plan without device access or writes |

## Output and timing

Each attempted trial writes `force_data_<texture>.csv` and `video_<texture>.mp4`. The suffix is the displayed texture, not its position in the trial sequence. CSV headers are `Fx,Fy,Fz,Tx,Ty,Tz,Timestamp`. `session_metadata.json` records configuration, planned order, per-trial status, timestamps, sample counts, and actual frame dimensions.

Camera capture and force reads occur sequentially, without hardware synchronization. Requested FPS does not establish actual frame or force frequency. Use timestamps and metadata to inspect timing. CSV timestamps are wall-clock times after sensor reads. Frame timestamps describe camera-read timing, not calibrated exposure times. Slow device access can reduce throughput; encoded video FPS can differ from elapsed capture timing.

Hardware-free analysis, tests, and dry-run do not establish physical camera/sensor operation. The CLI does not provide the missing historical force preprocessing, speed extraction, or spatial calibration. Define and document those methods for a new experiment before comparing derived quantities with the original workbook.
