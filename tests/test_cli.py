"""Output safety and useful CLI error handling."""

from contextlib import redirect_stderr
import io
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from human_texture.cli import MARKER, check_output, main


class OutputSafetyTests(unittest.TestCase):
    def test_cannot_write_inside_or_over_input_tree(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            data.mkdir()
            for output in (root, data, data / "processed" / "results"):
                with self.subTest(output=output), self.assertRaises(ValueError):
                    check_output(output, data)

    def test_existing_unrelated_output_is_preserved(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "existing"
            output.mkdir()
            note = output / "report.md"
            note.write_text("My own report")
            with self.assertRaises(ValueError):
                check_output(output, root / "data")
            self.assertEqual(note.read_text(), "My own report")

    def test_generated_directory_can_be_rerun(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "generated"
            output.mkdir()
            (output / MARKER).write_text("generated")
            check_output(output, root / "data")

    def test_nested_symlink_cannot_redirect_generated_figures(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "generated"
            output.mkdir()
            (output / MARKER).write_text("generated")
            external = root / "data" / "processed"
            external.mkdir(parents=True)
            (output / "figures").symlink_to(external, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "symbolic links"):
                check_output(output, root / "data")
            self.assertEqual(list(external.iterdir()), [])

    def test_missing_input_reports_error_without_traceback(self):
        with TemporaryDirectory() as temporary:
            stderr = io.StringIO()
            with redirect_stderr(stderr):
                status = main(["verify", "--data-dir", temporary])
            self.assertEqual(status, 1)
            self.assertIn("Missing input", stderr.getvalue())
            self.assertNotIn("Traceback", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
