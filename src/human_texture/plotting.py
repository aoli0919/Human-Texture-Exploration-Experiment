"""Publication-friendly static figures for the explicitly defined reanalysis."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .analysis import METRICS
from .data import DIMENSIONS

LABELS = {"traction": "Traction", "roughness": "Roughness", "coarseness": "Coarseness",
          "hardness": "Hardness", "temperature": "Temperature sensation"}


def _save(figure, directory, name):
    for extension in ("png", "svg"):
        figure.savefig(directory / f"{name}.{extension}", dpi=180, bbox_inches="tight")
    plt.close(figure)


def make_figures(observations, rating_summary, correlations, directory):
    directory.mkdir(parents=True, exist_ok=True)
    materials = observations[["texture_id", "texture"]].drop_duplicates().sort_values("texture_id")
    means = rating_summary.pivot(index="texture_id", columns="dimension", values="mean")
    values = means.loc[materials["texture_id"], list(DIMENSIONS)].to_numpy()
    fig, axis = plt.subplots(figsize=(9, 7), layout="constrained")
    heat = axis.imshow(values, cmap="RdBu_r", vmin=-10, vmax=10, aspect="auto")
    axis.set_xticks(range(5), [LABELS[d].replace(" sensation", "") for d in DIMENSIONS], rotation=20)
    axis.set_yticks(range(10), materials["texture"])
    for (row, column), value in np.ndenumerate(values):
        axis.text(column, row, f"{value:.1f}", ha="center", va="center",
                  color="white" if abs(value) > 6 else "black", fontsize=10)
    axis.set_title("Perceptual ratings by texture\nMean of 40 participants; original scale direction")
    fig.colorbar(heat, ax=axis, label="Rating (−10 to +10)", shrink=0.8)
    _save(fig, directory, "ratings")

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), layout="constrained")
    for axis, metric in zip(axes, METRICS):
        groups = [observations.loc[observations["texture_id"] == tid, metric].to_numpy()
                  for tid in materials["texture_id"]]
        axis.boxplot(groups, tick_labels=materials["texture"], showfliers=True,
                     medianprops={"color": "#c64d36", "linewidth": 1.6})
        axis.set_ylabel(f"{metric.replace('_', ' ').capitalize()}\n(units unspecified)")
        axis.tick_params(axis="x", rotation=22)
        axis.grid(axis="y", alpha=0.2)
    fig.suptitle("Exploration values supplied in the workbook\n40 participants per texture; boxes show quartiles and medians")
    _save(fig, directory, "exploration")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained", sharey=True)
    for axis, metric in zip(axes, METRICS):
        groups = [correlations.loc[(correlations["metric"] == metric) &
                                  (correlations["dimension"] == dimension), "rho"].dropna().to_numpy()
                  for dimension in DIMENSIONS]
        axis.boxplot([group if len(group) else [np.nan] for group in groups],
                     tick_labels=[LABELS[d].replace(" sensation", "") for d in DIMENSIONS],
                     medianprops={"color": "#c64d36", "linewidth": 1.6})
        for position, group in enumerate(groups, 1):
            axis.text(position, 1.05, f"n={len(group)}", ha="center", fontsize=9)
        axis.axhline(0, color="gray", linewidth=0.8, linestyle="--")
        axis.set_ylim(-1.12, 1.16)
        axis.set_title(metric.replace("_", " ").capitalize())
        axis.tick_params(axis="x", rotation=30)
        axis.grid(axis="y", alpha=0.2)
    axes[0].set_ylabel("Within-participant Spearman rho")
    fig.suptitle("Rating–exploration associations across ten textures\nOne coefficient per participant; undefined correlations excluded")
    _save(fig, directory, "associations")
