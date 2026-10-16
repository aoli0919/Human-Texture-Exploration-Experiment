"""Validate inputs and generate a traceable offline analysis."""

import argparse
from importlib.metadata import version
import json
from pathlib import Path
import platform
import sys
from tempfile import TemporaryDirectory

from . import __version__
from .analysis import describe_observations, participant_correlations
from .data import load_observations
from .quality import inspect_raw_force

MARKER = ".human-texture-output"
CAVEATS = [
    "Reported force units and the original contact-window/filtering method are undocumented.",
    "Speed used pixel-to-mm spatial calibration; its factor/file is unavailable and the final time unit is unconfirmed.",
    "Raw-force QC does not reproduce the historical Average Force calculation; videos are not processed.",
    "Correlations summarize participant-level associations, without pooled inferential tests or paper-result claims.",
]


def validate(data_dir, skip_raw):
    observations, audit = load_observations(data_dir)
    quality = None
    if skip_raw:
        audit["raw_force"] = {"status": "skipped", "reason": "--skip-raw requested"}
    else:
        quality, raw_audit, hashes = inspect_raw_force(data_dir, observations)
        audit["raw_force"] = raw_audit
        audit["input_sha256"].update(hashes)
    audit.update(analysis_version=__version__, caveats=CAVEATS,
                 software={"python": platform.python_version(), "platform": platform.platform(),
                           **{name: version(name) for name in ("numpy", "pandas", "scipy", "matplotlib", "openpyxl")}})
    return observations, quality, audit


def check_output(output_dir, data_dir):
    output, data = output_dir.resolve(), data_dir.resolve()
    if output == data or output in data.parents or data in output.parents:
        raise ValueError("Output directory must be separate from the input data directory and its ancestors.")
    if output.exists() and (not output.is_dir() or (any(output.iterdir()) and not (output / MARKER).is_file())):
        raise ValueError(f"Refusing to write into an existing nonempty directory without {MARKER}: {output}")
    if output_dir.is_symlink() or (output.exists() and any(path.is_symlink() for path in output.rglob("*"))):
        raise ValueError("Output directory must not contain symbolic links.")


def write_report(directory, audit, associations):
    raw = audit["raw_force"]
    raw_line = (f"Checked {raw['files_checked']} raw-force CSVs with {raw['total_samples']:,} samples; "
                "all numeric values finite and all within-file timestamps strictly increasing."
                if raw["status"] == "checked" else "Raw-force checks were skipped (--skip-raw).")
    lines = ["# Human texture exploration: analysis report", "",
             f"Validated {audit['participants']} participants × {audit['textures']} textures = {audit['observations']} observations.",
             "The two workbooks agree on all 2,000 perceptual ratings. NF is extracted from Force text; SS is Speed rounded to two decimals.",
             "", raw_line, "", "## Participant-level associations", "",
             "Each coefficient compares ten textures within one participant. These are descriptive distributions, not causal or pooled significance tests.", "",
             "| Exploration measure | Rating | Valid participants | Median rho | 25th–75th percentile |",
             "| --- | --- | ---: | ---: | ---: |"]
    for row in associations.itertuples(index=False):
        lines.append(f"| {row.metric} | {row.dimension} | {row.n_valid} | {row.median_rho:.3f} | {row.q25_rho:.3f} to {row.q75_rho:.3f} |")
    lines.extend(["", "## Figures", "", "![Mean ratings](figures/ratings.png)", "",
                  "![Reported exploration values](figures/exploration.png)", "",
                  "![Within-participant associations](figures/associations.png)", "", "## Limits", ""])
    lines.extend(f"- {caveat}" for caveat in CAVEATS)
    lines.extend(["", "Tables and audit.json contain the underlying values, validation counts, software versions, and SHA-256 input hashes.",
                  "See docs/methods.md and docs/data_dictionary.md in the repository for definitions.", ""])
    (directory / "report.md").write_text("\n".join(lines), encoding="utf-8")


def reproduce(data_dir, output_dir, skip_raw):
    check_output(output_dir, data_dir)
    observations, quality, audit = validate(data_dir, skip_raw)
    rating_summary, exploration_summary = describe_observations(observations)
    correlations, associations = participant_correlations(observations)
    from .plotting import make_figures
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".texture-analysis-", dir=output_dir.parent) as temporary:
        staged = Path(temporary)
        tables = {"observations": observations, "rating_summary": rating_summary,
                  "exploration_summary": exploration_summary, "within_participant_correlations": correlations,
                  "association_summary": associations}
        if quality is not None:
            tables["raw_force_qc"] = quality
        for name, table in tables.items():
            table.to_csv(staged / f"{name}.csv", index=False, float_format="%.10g")
        audit["output_rows"] = {name: len(table) for name, table in tables.items()}
        (staged / "audit.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        write_report(staged, audit, associations)
        make_figures(observations, rating_summary, correlations, staged / "figures")
        (staged / MARKER).write_text("Generated by human-texture-exploration; safe to rerun this command here.\n", encoding="utf-8")
        output_dir.mkdir(parents=True, exist_ok=True)
        for source in staged.rglob("*"):
            destination = output_dir / source.relative_to(staged)
            if source.is_dir():
                destination.mkdir(exist_ok=True)
            else:
                source.replace(destination)
        if skip_raw:
            (output_dir / "raw_force_qc.csv").unlink(missing_ok=True)
    return audit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=__version__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("verify", "reproduce"):
        command_parser = subparsers.add_parser(command)
        command_parser.add_argument("--data-dir", type=Path, default=Path("data"))
        command_parser.add_argument("--skip-raw", action="store_true", help="Validate processed workbooks only")
        if command == "reproduce":
            command_parser.add_argument("--output-dir", type=Path, default=Path("outputs/reproduction"))
    args = parser.parse_args(argv)
    try:
        if args.command == "verify":
            _, _, audit = validate(args.data_dir, args.skip_raw)
        else:
            audit = reproduce(args.data_dir, args.output_dir, args.skip_raw)
    except (ValueError, OSError, KeyError, IndexError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    print(f"Validated {audit['observations']} observations / {audit['ratings_identical']} ratings; raw force: {audit['raw_force']['status']}.")
    if args.command == "reproduce":
        print(f"Results: {(args.output_dir / 'report.md').resolve()}")
    return 0
