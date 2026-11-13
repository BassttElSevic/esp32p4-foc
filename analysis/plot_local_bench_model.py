"""Plot the eleven BENCH1 trials using the exported common-time waveforms.

Run from any directory with Python, pandas and matplotlib. The optional
directory contains both waveforms.csv (file, t_us, reference, measured,
nominal, local, angle_{measured,nominal,local}_rad) and metrics.csv (trial
metadata, split and trip times). Angles use mechanical shaft radians.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np
import pandas as pd


DEFAULT_DIRECTORY = Path(__file__).resolve().parents[1] / "bench-results" / "local-model"
COLORS = {"reference": "#76787b", "measured": "#913c30", "nominal": "#335e83", "local": "#197861"}
STYLE = {
    "reference": {"linestyle": ":", "linewidth": 1.05},
    "measured": {"linestyle": "-", "linewidth": 0.83},
    "nominal": {"linestyle": (0, (4.3, 2.3)), "linewidth": 1.12},
    "local": {"linestyle": (0, (6, 1.8, 1.3, 1.8)), "linewidth": 1.2},
}


def chinese_font():
    available = set(font_manager.get_font_names())
    for name in ("Microsoft YaHei", "Noto Sans CJK SC", "Source Han Sans SC",
                 "PingFang SC", "WenQuanYi Zen Hei", "SimHei"):
        if name in available:
            return name
    raise RuntimeError("Install a Simplified Chinese font, e.g. Noto Sans CJK SC.")


def style_time_axis(ax):
    ax.spines[["right", "top"]].set_visible(False)
    ax.tick_params(direction="out", length=2.8, width=0.6, pad=2.5)
    ax.set_axisbelow(True)
    ax.grid(True, linewidth=0.48, color="#e3e5e6")


def plot_angle(ax, data):
    t = data["t_us"].to_numpy() * 1e-6
    style_time_axis(ax)
    for key in ("nominal", "measured", "local"):
        theta = data[f"angle_{key}_rad"].to_numpy()
        # Express each displacement relative to its own first sampled value.
        # A constant origin is the only adjustment. No detrending or time shift.
        ax.plot(t, np.rad2deg(theta - theta[0]), color=COLORS[key], **STYLE[key])
    ax.set_xlim(0, t[-1])
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax.set_xlabel("时间 (s)", labelpad=1)
    ax.set_ylabel("机械角位移 (°)", labelpad=4)
    ax.set_title("第 02 组 · 留组验证   /   速度试验的机械角位移",
                 loc="left", fontsize=9.8, pad=4.3, fontweight="bold")
    ax.text(1, 1.046, "首个采样点为零点", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=7.3, color="#464a4d")


def plot_trial(ax, rec, data):
    t = data["t_us"].to_numpy() * 1e-6
    if not len(t) or not np.isfinite(t).all() or not np.all(np.diff(t) > 0):
        raise ValueError(f"Missing or invalid timestamps in {rec.file}")
    if not np.isfinite(data[["reference", "measured"]]).all().all():
        raise ValueError(f"Incomplete reference or measurement in {rec.file}")
    speed = rec.mode == "speed"
    bench_fault = rec.bench_end != "ok"
    end = t[-1] if bench_fault else max(t[-1], rec.cycles / rec.frequency_Hz)
    scale = 1000 if end < 0.1 else 1
    x = t * scale
    style_time_axis(ax)

    for key in ("reference", "nominal", "measured", "local"):
        line, = ax.plot(x, data[key], color=COLORS[key], **STYLE[key])
        if key == "measured" and len(x) <= 20:
            line.set_marker("o")
            line.set_markersize(3.5)
    # Where reference and model coincide, mark extrema at recorded samples.
    if speed and len(x) > 20:
        peaks = np.arange(0.25, end * rec.frequency_Hz, 0.5) / rec.frequency_Hz
        if len(peaks):
            indices = np.abs(t[:, None] - peaks).argmin(axis=0)
            ax.scatter(x[indices], data["reference"].to_numpy()[indices], marker="o",
                       facecolor="white", edgecolor=COLORS["reference"],
                       s=8, linewidth=0.55, zorder=4)
    ax.set_xlim(0, end * scale)
    peak = np.nanmax(np.abs(data[list(STYLE)].to_numpy()))
    ax.set_ylim(-1.12 * peak, 1.12 * peak)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5, symmetric=True))
    ax.xaxis.set_major_locator(MaxNLocator(nbins=7, min_n_ticks=3))
    ax.set_xlabel("时间 (ms)" if scale == 1000 else "时间 (s)", labelpad=1)
    ax.set_ylabel("电角速度 (rad/s)" if speed else "q 轴电流 (mA)", labelpad=4)

    group = "首批 · 辨识" if rec.split == "train" else f"第 {rec.group} 组 · 留组验证"
    unit = "电 rad/s" if speed else "mA"
    ax.set_title(f"{group}   /   {rec.frequency_Hz:g} Hz，±{rec.amplitude:g} {unit}",
                 loc="left", fontsize=9.8, pad=4.3, fontweight="bold")
    events = []
    if bench_fault:
        events.append(f"实测 {rec.bench_end}（{len(t)} 点）")
    for key, label in (("nominal", "名义"), ("local", "辨识")):
        trip = getattr(rec, key + "_trip_s")
        if np.isfinite(trip):
            outside = trip > end
            events.append(f"{label} {trip*1000:g} ms 停" + ("（窗外）" if outside else ""))
            if not outside:
                ax.axvline(trip * scale, color=COLORS[key], linewidth=0.85, alpha=0.8, linestyle=":")
                ax.plot(trip * scale, 1, marker="v", transform=ax.get_xaxis_transform(),
                        color=COLORS[key], markersize=4.2, clip_on=False)
    if events:
        ax.text(1, 1.046, "  ·  ".join(events), transform=ax.transAxes,
                ha="right", va="bottom", color="#464a4d", fontsize=7.3)


def plot_comparison(directory):
    metrics = pd.read_csv(directory / "metrics.csv", dtype={"group": str})
    waves = pd.read_csv(directory / "waveforms.csv")
    if not metrics.file.is_unique or set(waves.file) != set(metrics.file):
        raise ValueError("Waveform keys must match the unique trial IDs in metrics.csv.")
    training = metrics[metrics.split == "train"]
    validation = metrics[metrics.split == "validation"]
    groups = sorted(validation.group.unique())
    if (len(training) != 3 or len(groups) != 4 or len(metrics) != 11
            or not metrics["mode"].isin(["current", "speed"]).all()):
        raise ValueError("Expected 3 initial training trials and 4 current/speed validation pairs.")
    slots = {}
    for mode, column in (("current", 0), ("speed", 1)):
        initial = training[training["mode"] == mode].sort_values("amplitude")
        expected = 1 if mode == "speed" else 2
        if len(initial) != expected:
            raise ValueError(f"Expected {expected} initial {mode} trials.")
        for row, file in enumerate(initial.file):
            slots[file] = (row, column)
        for row, group in enumerate(groups, start=2):
            match = validation[(validation.group == group) & (validation["mode"] == mode)]
            slots[match.file.item()] = (row, column)

    settings = {"font.family": chinese_font(), "axes.unicode_minus": False,
                "font.size": 9, "axes.labelsize": 8, "xtick.labelsize": 7.4, "ytick.labelsize": 7.4,
                "axes.linewidth": 0.65, "axes.edgecolor": "#565c60", "text.color": "#22292d",
                "axes.labelcolor": "#333a3e", "xtick.color": "#333a3e", "ytick.color": "#333a3e",
                "pdf.fonttype": 42, "savefig.facecolor": "white"}
    with plt.rc_context(settings):
        fig, axes = plt.subplots(6, 2, figsize=(13.4, 15.4))
        fig.subplots_adjust(left=0.066, right=0.984, top=0.916, bottom=0.050,
                            hspace=0.68, wspace=0.18)
        fig.suptitle("DENG FOC 2208  |  名义模型与台架辨识模型的正弦响应",
                     fontsize=18, y=0.985, fontweight="bold")
        for rec in metrics.itertuples(index=False):
            ax = axes[slots[rec.file]]
            plot_trial(ax, rec, waves[waves.file == rec.file])
        for column, heading in enumerate(("电流内环", "速度外环")):
            fig.text(axes[0, column].get_position().x0, 0.935, heading,
                     fontsize=12, fontweight="bold", ha="left")
        angle_file = validation[(validation.group == "02") & (validation["mode"] == "speed")].file.item()
        plot_angle(axes[1, 1], waves[waves.file == angle_file])
        keys = ("reference", "measured", "nominal", "local")
        labels = ("输入正弦", "台架实测", "原名义模型", "台架辨识模型")
        handles = [Line2D([0], [0], color=COLORS[k], **STYLE[k]) for k in keys]
        fig.legend(handles, labels, ncol=4, loc="upper center", bbox_to_anchor=(0.5, 0.957),
                   frameon=False, borderaxespad=0, handlelength=3.3, columnspacing=2.8, fontsize=9.5)
        fig.text(0.5, 0.012, "电流与速度面板显示滤波反馈。三角标出模型保护动作，实机故障记录止于最后一个采样点。",
                 ha="center", fontsize=8.6, color="#464a4d")
        path = directory / "model_comparison"
        fig.savefig(path.with_suffix(".pdf"), format="pdf", metadata={"Title": "DENG FOC 2208 名义与辨识模型对照"})
        fig.savefig(path.with_suffix(".png"), dpi=155)
        plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path, default=DEFAULT_DIRECTORY)
    plot_comparison(parser.parse_args().directory)
