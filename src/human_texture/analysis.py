"""Descriptive reanalysis; participants are never treated as 400 independent people."""

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from .data import DIMENSIONS

METRICS = ("reported_force", "reported_speed")


def describe_observations(observations):
    ratings = observations.melt(id_vars=["participant_id", "texture_id", "texture"],
                                value_vars=list(DIMENSIONS), var_name="dimension", value_name="rating")
    rating_summary = ratings.groupby(["texture_id", "texture", "dimension"], sort=False)["rating"].agg(
        n="count", mean="mean", sd="std", median="median", minimum="min", maximum="max").reset_index()
    rows = []
    for (texture_id, texture), group in observations.groupby(["texture_id", "texture"], sort=True):
        row = {"texture_id": texture_id, "texture": texture, "n": len(group)}
        for metric in METRICS:
            values = group[metric]
            row.update({f"{metric}_{name}": value for name, value in {
                "mean": values.mean(), "sd": values.std(ddof=1), "median": values.median(),
                "minimum": values.min(), "maximum": values.max(),
            }.items()})
        rows.append(row)
    return rating_summary, pd.DataFrame(rows)


def participant_correlations(observations):
    rows = []
    for participant, group in observations.groupby("participant_id", sort=True):
        if group["texture_id"].duplicated().any():
            raise ValueError(f"Duplicate texture observations for {participant}")
        for metric in METRICS:
            for dimension in DIMENSIONS:
                pair = group[[metric, dimension]].dropna()
                defined = len(pair) >= 3 and pair[metric].nunique() > 1 and pair[dimension].nunique() > 1
                rho = float(spearmanr(pair[metric], pair[dimension]).statistic) if defined else np.nan
                rows.append({"participant_id": participant, "metric": metric, "dimension": dimension,
                             "n_textures": len(pair), "rho": rho,
                             "status": "defined" if defined else "constant_or_insufficient"})
    correlations = pd.DataFrame(rows)
    summaries = []
    for (metric, dimension), group in correlations.groupby(["metric", "dimension"], sort=False):
        defined = group["rho"].dropna()
        summaries.append({"metric": metric, "dimension": dimension, "n_valid": len(defined),
                          "n_undefined": len(group) - len(defined),
                          "mean_rho": defined.mean(), "median_rho": defined.median(),
                          "q25_rho": defined.quantile(0.25), "q75_rho": defined.quantile(0.75)})
    return correlations, pd.DataFrame(summaries)
