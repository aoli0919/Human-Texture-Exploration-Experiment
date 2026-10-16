import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from acquisition.main import Config, main, run_session
from acquisition.forcesensor import FTSensorSerial


class FakeFrame:
    shape = (123, 234, 3)


class FakeCamera:
    released = False

    def isOpened(self):
        return True

    def set(self, *args):
        pass

    def read(self):
        return True, FakeFrame()

    def release(self):
        self.released = True


class FakeWriter(FakeCamera):
    def write(self, frame):
        pass


class FakeCV:
    CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS = 3, 4, 5
    FONT_HERSHEY_SIMPLEX = 0

    def __init__(self, keys):
        self.camera = FakeCamera()
        self.writer = FakeWriter()
        self.keys = iter(keys)
        self.destroyed = False

    def VideoCapture(self, index):
        return self.camera

    def VideoWriter_fourcc(self, *args):
        return 0

    def VideoWriter(self, path, codec, fps, size):
        self.writer_size = size
        return self.writer

    def putText(self, *args):
        pass

    def imshow(self, *args):
        pass

    def waitKey(self, delay):
        return next(self.keys)

    def destroyAllWindows(self):
        self.destroyed = True


class FakeSensor:
    closed = False

    def set_zero_ref(self):
        pass

    def get_ft(self):
        return [1, 2, 3, 4, 5, 6]

    def close(self):
        self.closed = True


class AcquisitionTests(unittest.TestCase):
    def test_dry_run_no_hardware_imports_or_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "must-not-exist"
            # Fail if dry-run even attempts to import a hardware dependency.
            script = '''
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in ('cv2', 'serial', 'minimalmodbus', 'libscrc'):
        raise AssertionError('hardware import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from acquisition.main import main
main(__import__('sys').argv[1:])
'''
            result = subprocess.run([sys.executable, "-c", script, "--dry-run", "--seed", "7",
                                     "--session-id", "anon-001", "--output-dir", str(target)],
                                    capture_output=True, text=True, check=True,
                                    cwd=Path(__file__).resolve().parents[1])
            plan = json.loads(result.stdout)
            self.assertEqual(sorted(plan["trial_order"]), list(range(1, 11)))
            self.assertFalse(target.exists())
            with contextlib.redirect_stdout(io.StringIO()) as output:
                main(["--dry-run", "--seed", "7"])
            self.assertEqual(json.loads(output.getvalue())["trial_order"], plan["trial_order"])

    def test_invalid_config(self):
        for config in (Config(), Config(session_id="../name", output_dir="out"),
                       Config(duration=0), Config(fps=float("nan")), Config(width=-1),
                       Config(camera_index=-1), Config(serial_port="")):
            with self.subTest(config=config), self.assertRaises(ValueError):
                config.validate()
        for args in (["--dry-run", "--duration", "0"], ["--dry-run", "--height", "-2"], []):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args)

    def test_cancel_saves_partial_and_releases_devices(self):
        with tempfile.TemporaryDirectory() as temporary:
            cv = FakeCV([13, ord("q")])
            sensor = FakeSensor()
            result = run_session(Config(session_id="anon-001", output_dir=temporary, seed=1),
                                 cv=cv, sensor_factory=lambda **kwargs: sensor)
            self.assertEqual(result["status"], "cancelled")
            self.assertEqual(len(result["trials"]), 1)
            trial = result["trials"][0]
            self.assertEqual(trial["status"], "partial")
            self.assertEqual(trial["force_samples"], 1)
            self.assertEqual(len(trial["frame_timestamps"]), 1)
            self.assertEqual(cv.writer_size, (234, 123))
            self.assertTrue(cv.camera.released and cv.writer.released and sensor.closed and cv.destroyed)
            directory = Path(temporary) / "anon-001"
            saved = json.loads((directory / "session_metadata.json").read_text())
            self.assertEqual(saved["status"], "cancelled")
            self.assertEqual(len((directory / trial["force_file"]).read_text().splitlines()), 2)

    def test_sensor_error_saves_partial_and_releases_devices(self):
        with tempfile.TemporaryDirectory() as temporary:
            cv = FakeCV([13])
            sensor = FakeSensor()
            def fail():
                raise RuntimeError("simulated sensor error")
            sensor.get_ft = fail
            with self.assertRaisesRegex(RuntimeError, "simulated sensor error"):
                run_session(Config(session_id="anon-002", output_dir=temporary), cv=cv,
                            sensor_factory=lambda **kwargs: sensor)
            saved = json.loads((Path(temporary) / "anon-002" / "session_metadata.json").read_text())
            self.assertEqual(saved["status"], "error")
            self.assertEqual(saved["trials"][0]["status"], "partial")
            self.assertTrue(cv.camera.released and cv.writer.released and sensor.closed and cv.destroyed)

    def test_all_ten_trials_finish_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            cv = FakeCV([13] * 10)
            sensor = FakeSensor()
            ticks = iter(range(20))
            result = run_session(Config(session_id="anon-003", output_dir=temporary, duration=0.1),
                                 cv=cv, sensor_factory=lambda **kwargs: sensor,
                                 monotonic=lambda: next(ticks))
            self.assertEqual(result["status"], "complete")
            self.assertEqual([t["texture"] for t in result["trials"]], result["trial_order"])
            self.assertTrue(all(t["status"] == "complete" for t in result["trials"]))
            self.assertEqual(len(list((Path(temporary) / "anon-003").glob("force_data_*.csv"))), 10)

    def test_serial_crc_error_closes_every_connection(self):
        connections = []
        class Connection:
            def __init__(self, **kwargs):
                self.closed = False
                connections.append(self)

            def write(self, value):
                pass

            def read_until(self, marker):
                return bytes(14) + marker

            def close(self):
                self.closed = True

        class Instrument:
            def __init__(self, *args, **kwargs):
                self.serial = Connection()

            def write_register(self, *args):
                pass

        dependencies = {"serial": SimpleNamespace(Serial=Connection),
                        "minimalmodbus": SimpleNamespace(Instrument=Instrument),
                        "libscrc": SimpleNamespace(modbus=lambda data: 1)}
        with patch.dict(sys.modules, dependencies):
            sensor = FTSensorSerial(port="fake-port")
            with self.assertRaisesRegex(RuntimeError, "CRC mismatch"):
                sensor.get_ft()
            sensor.close()
        self.assertTrue(connections)
        self.assertTrue(all(connection.closed for connection in connections))


if __name__ == "__main__":
    unittest.main()
