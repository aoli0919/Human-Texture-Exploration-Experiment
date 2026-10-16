"""Regression tests for workbook interpretation and participant-level analysis."""

from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from human_texture.analysis import participant_correlations
from human_texture.data import DIMENSIONS, load_observations, parse_force_reference, rounded_speed
from human_texture.quality import FORCE_COLUMNS, inspect_force_file, inspect_raw_force


ROOT = Path(__file__).resolve().parents[1]


class ForceReferenceTests(unittest.TestCase):
    def test_reference_parses_anonymous_session_texture_and_scientific_notation(self):
        self.assertEqual(parse_force_reference(
            " Folder: anon-session_03, File: force_data_10.csv, Average Force: -1.25e+1 "),
            ("anon-session_03", "force_data_10.csv", 10, -12.5))

    def test_reference_rejects_path_components_and_extra_text(self):
        malformed = (
            "Folder: ../private, File: force_data_1.csv, Average Force: 1",
            "Folder: anon/name, File: force_data_1.csv, Average Force: 1",
            "Folder: anon\\name, File: force_data_1.csv, Average Force: 1",
            "Folder: /absolute, File: force_data_1.csv, Average Force: 1",
            "Folder: anon, File: ../force_data_1.csv, Average Force: 1",
            "Folder: anon, File: force_data_1.csv, Average Force: NaN",
            "Folder: anon, File: force_data_1.csv, Average Force: inf",
            "Folder: anon, File: force_data_1.csv, Average Force: 1 trailing",
        )
        for value in malformed:
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_force_reference(value)

    def test_reference_rejects_numeric_overflow(self):
        with self.assertRaises(ValueError):
            parse_force_reference("Folder: anon, File: force_data_1.csv, Average Force: 1e999")

    def test_excel_round_positive_and_negative_ties(self):
        for value, expected in ((1.005, 1.01), (-1.005, -1.01), (2.675, 2.68),
                                (-2.675, -2.68), (0.005, 0.01), (-0.005, -0.01),
                                (1.004, 1.0), (-1.004, -1.0)):
            with self.subTest(value=value):
                self.assertEqual(rounded_speed(value), expected)


def observation(participant, texture, force, rating, speed=None):
    return {"participant_id": participant, "texture_id": texture,
            "reported_force": force, "reported_speed": force if speed is None else speed,
            **dict.fromkeys(DIMENSIONS, rating)}


class ParticipantCorrelationTests(unittest.TestCase):
    def test_correlations_are_within_participant_despite_opposite_pooled_result(self):
        observations = pd.DataFrame([
            observation("P01", i + 1, x, y) for i, (x, y) in enumerate(((1, 3), (2, 2), (3, 1)))
        ] + [
            observation("P02", i + 1, x, y) for i, (x, y) in enumerate(((10, 12), (11, 11), (12, 10)))
        ])
        self.assertGreater(spearmanr(observations.reported_force, observations.traction).statistic, 0)
        correlations, summary = participant_correlations(observations)
        self.assertEqual(len(correlations), 2 * 2 * 5)
        self.assertTrue(np.allclose(correlations.rho, -1))
        self.assertTrue((correlations.n_textures == 3).all())
        self.assertTrue((correlations.status == "defined").all())
        self.assertTrue((summary.n_valid == 2).all())
        self.assertTrue(np.allclose(summary.mean_rho, -1))

    def test_constant_or_insufficient_pairs_have_nan_and_explicit_status(self):
        frame = pd.DataFrame([
            observation("constant_metric", i, 1, i) for i in range(1, 4)
        ] + [
            observation("constant_rating", i, i, 1) for i in range(1, 4)
        ] + [
            observation("only_two", i, i, i) for i in range(1, 3)
        ])
        correlations, summary = participant_correlations(frame)
        self.assertTrue(correlations.rho.isna().all())
        self.assertEqual(set(correlations.status), {"constant_or_insufficient"})
        self.assertTrue((summary.n_valid == 0).all())
        self.assertTrue((summary.n_undefined == 3).all())
        self.assertTrue(summary.mean_rho.isna().all())

    def test_duplicate_texture_is_rejected_instead_of_counting_twice(self):
        frame = pd.DataFrame([observation("P01", 1, 1, 2), observation("P01", 1, 2, 3)])
        with self.assertRaisesRegex(ValueError, "Duplicate texture"):
            participant_correlations(frame)


class ForceCSVQualityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.data_dir = Path(self.temporary.name)
        self.path = self.data_dir / "raw" / "force" / "anon" / "force_data_1.csv"
        self.path.parent.mkdir(parents=True)
        self.frame = pd.DataFrame([[1, 2, -3, 4, 5, 6, 100],
                                   [1, 2, -5, 4, 5, 6, 100.02]], columns=FORCE_COLUMNS, dtype=float)
        self.observations = pd.DataFrame([{
            "participant_id": "P01", "session_id": "anon", "texture_id": 1,
            "source_force_csv": "raw/force/anon/force_data_1.csv", "reported_force": 4,
        }])

    def write(self, frame):
        frame.to_csv(self.path, index=False)

    def test_valid_file_reports_measured_duration_and_raw_fz(self):
        self.write(self.frame)
        detail = inspect_force_file(self.path)
        self.assertEqual(detail["samples"], 2)
        self.assertTrue(detail["finite"] and detail["timestamps_increasing"])
        self.assertAlmostEqual(detail["duration_s"], .02)
        self.assertAlmostEqual(detail["median_interval_s"], .02)
        self.assertEqual(detail["raw_mean_abs_fz"], 4)
        quality, audit, hashes = inspect_raw_force(self.data_dir, self.observations)
        self.assertEqual(len(quality), 1)
        self.assertEqual(audit["files_checked"], 1)
        self.assertEqual(audit["total_samples"], 2)
        self.assertEqual(len(next(iter(hashes.values()))), 64)

    def test_schema_and_short_recording_are_rejected(self):
        cases = (self.frame.drop(columns="Fx"),
                 self.frame[list(reversed(FORCE_COLUMNS))], self.frame.iloc[:1])
        for frame in cases:
            with self.subTest(columns=list(frame), samples=len(frame)):
                self.write(frame)
                with self.assertRaisesRegex(ValueError, "Invalid force CSV"):
                    inspect_force_file(self.path)

    def test_non_numeric_measurement_is_rejected(self):
        frame = self.frame.astype(object)
        frame.loc[0, "Fx"] = "broken"
        self.write(frame)
        with self.assertRaises(ValueError):
            inspect_force_file(self.path)

    def test_nonfinite_measurements_fail_raw_quality_gate(self):
        for value in (float("inf"), float("-inf"), float("nan")):
            with self.subTest(value=value):
                frame = self.frame.copy()
                frame.loc[0, "Fx"] = value
                self.write(frame)
                self.assertFalse(inspect_force_file(self.path)["finite"])
                with self.assertRaisesRegex(ValueError, "Force data quality failure"):
                    inspect_raw_force(self.data_dir, self.observations)

    def test_decreasing_or_repeated_timestamps_fail_raw_quality_gate(self):
        for end in (99, 100):
            with self.subTest(end=end):
                frame = self.frame.copy()
                frame.loc[1, "Timestamp"] = end
                self.write(frame)
                self.assertFalse(inspect_force_file(self.path)["timestamps_increasing"])
                with self.assertRaisesRegex(ValueError, "Force data quality failure"):
                    inspect_raw_force(self.data_dir, self.observations)

    def test_missing_file_and_external_reference_are_rejected(self):
        self.write(self.frame)
        for path in ("raw/force/anon/missing.csv", "../outside.csv", str(self.path.resolve())):
            with self.subTest(path=path):
                observations = self.observations.copy()
                observations["source_force_csv"] = path
                with self.assertRaises(FileNotFoundError):
                    inspect_raw_force(self.data_dir, observations)


class ArchivedWorkbookTests(unittest.TestCase):
    def test_original_workbooks_yield_400_unique_aligned_observations(self):
        # One load covers original workbook alignment and all 2,000 rating comparisons.
        observations, audit = load_observations(ROOT / "data")
        self.assertEqual(len(observations), 400)
        self.assertEqual(observations.participant_id.nunique(), 40)
        self.assertEqual(observations.session_id.nunique(), 40)
        self.assertEqual(observations.source_force_csv.nunique(), 400)
        for _, group in observations.groupby("participant_id"):
            self.assertEqual(sorted(group.texture_id), list(range(1, 11)))
        self.assertEqual(audit["ratings_identical"], 2000)
        self.assertTrue(np.isfinite(observations[list(DIMENSIONS) + ["reported_force", "reported_speed"]]).all().all())
        self.assertTrue(observations[list(DIMENSIONS)].ge(-10).all().all())
        self.assertTrue(observations[list(DIMENSIONS)].le(10).all().all())
        for row in observations.itertuples():
            self.assertEqual(row.reported_speed, rounded_speed(row.speed_unrounded))


if __name__ == "__main__":
    unittest.main()
