"""Inspect archived force recordings without assuming the original contact window."""

from pathlib import Path

import numpy as np
import pandas as pd

from .data import sha256

FORCE_COLUMNS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Timestamp"]


def inspect_force_file(path):
    raw = pd.read_csv(path)
    if list(raw.columns) != FORCE_COLUMNS or len(raw) < 2:
        raise ValueError(f"Invalid force CSV columns or fewer than 2 samples: {path}")
    numeric = raw.apply(pd.to_numeric, errors="raise")
    finite = bool(np.isfinite(numeric.to_numpy()).all())
    differences = np.diff(numeric["Timestamp"].to_numpy())
    increasing = bool((differences > 0).all())
    return {"samples": len(numeric), "finite": finite, "timestamps_increasing": increasing,
            "duration_s": numeric["Timestamp"].iloc[-1] - numeric["Timestamp"].iloc[0],
            "median_interval_s": float(np.median(differences)),
            "raw_mean_abs_fz": numeric["Fz"].abs().mean()}


def inspect_raw_force(data_dir, observations):
    data_dir = Path(data_dir)
    rows, hashes = [], {}
    references = set(observations["source_force_csv"])
    available = {str(path.relative_to(data_dir).as_posix()) for path in (data_dir / "raw" / "force").rglob("*.csv")}
    missing = sorted(references - available)
    if missing:
        raise FileNotFoundError(f"Missing {len(missing)} referenced force CSV(s), first: {missing[0]}. "
                                "Use --skip-raw only for processed-data analysis.")
    for observation in observations.itertuples(index=False):
        relative = observation.source_force_csv
        path = data_dir / relative
        quality = inspect_force_file(path)
        rows.append({"participant_id": observation.participant_id, "session_id": observation.session_id,
                     "texture_id": observation.texture_id, "source_force_csv": relative, **quality,
                     "reported_force": observation.reported_force})
        hashes[relative] = sha256(path)
    quality = pd.DataFrame(rows)
    invalid = quality.loc[~quality["finite"] | ~quality["timestamps_increasing"]]
    if len(invalid):
        raise ValueError(f"Force data quality failure in {len(invalid)} files; first: {invalid.iloc[0]['source_force_csv']}")
    return quality, {"status": "checked", "files_checked": len(quality),
                     "additional_unreferenced_csv": sorted(available - references),
                     "total_samples": int(quality["samples"].sum()),
                     "duration_s_min": float(quality["duration_s"].min()),
                     "duration_s_max": float(quality["duration_s"].max()),
                     "median_interval_s_min": float(quality["median_interval_s"].min()),
                     "median_interval_s_max": float(quality["median_interval_s"].max()),
                     "all_finite": True, "all_timestamps_increasing": True}, hashes
