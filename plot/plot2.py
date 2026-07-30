#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


# =========================
# 全局字体大小设置
# =========================
SMALL_FONT = 12
LARGE_FONT = 16

plt.rcParams.update({
    "font.size": SMALL_FONT,
    "axes.titlesize": LARGE_FONT,
    "axes.labelsize": LARGE_FONT,
    "xtick.labelsize": SMALL_FONT,
    "ytick.labelsize": SMALL_FONT,
    "legend.fontsize": SMALL_FONT,
    "figure.titlesize": LARGE_FONT,
})


# =========================
# 参数设置
# =========================
DT = 0.1
MARKER = 10.0

# 如果文件名不同，请自行修改
OURS_FILE = "error_ours2.txt"
OTHER_FILE = "error_lun.txt"

OURS_NAME = "Ours"
OTHER_NAME = "SF-ALAS-PAAS"

OUT_DIR = Path("formation_figures_combined")
OUT_DIR.mkdir(exist_ok=True)


# =========================
# 数据读取
# =========================
def load_error_file(file_path, name):
    data = np.loadtxt(file_path, dtype=float)

    marker_mask = np.isclose(data, MARKER)
    marker_idx = np.where(marker_mask)[0]

    valid_mask = ~marker_mask
    t_original = np.arange(len(data)) * DT

    return {
        "name": name,
        "data": data,
        "marker_idx": marker_idx,
        "valid_mask": valid_mask,
        "t_original": t_original[valid_mask],
        "error_original": data[valid_mask],
    }


# =========================
# 分段提取
# 第 i 段：
# 第 i 次数量变化后 -> 第 i+1 次数量变化前
# 最后一段：第4次数量变化后 -> 数据结束
# =========================
def get_segments(method):
    data = method["data"]
    marker_idx = method["marker_idx"]

    segments = []

    for i in range(len(marker_idx)):
        start = marker_idx[i] + 1

        if i < len(marker_idx) - 1:
            end = marker_idx[i + 1]
        else:
            end = len(data)

        y = data[start:end]
        y = y[~np.isclose(y, MARKER)]

        if len(y) < 2:
            continue

        tau = np.arange(len(y)) * DT

        duration = tau[-1] - tau[0]
        start_error = y[0]
        end_error = y[-1]
        min_error = np.min(y)

        # 平均下降速率：正值表示整体误差下降
        drop_rate = (start_error - end_error) / duration if duration > 0 else np.nan

        min_idx = np.argmin(y)
        time_to_min = min_idx * DT

        if time_to_min > 0:
            drop_rate_to_min = (start_error - min_error) / time_to_min
        else:
            drop_rate_to_min = np.nan

        segments.append({
            "event": i + 1,
            "tau": tau,
            "error": y,
            "duration": duration,
            "start_error": start_error,
            "end_error": end_error,
            "min_error": min_error,
            "time_to_min": time_to_min,
            "drop_rate": drop_rate,
            "drop_rate_to_min": drop_rate_to_min,
        })

    return segments


# =========================
# 读取数据
# =========================
ours = load_error_file(OURS_FILE, OURS_NAME)
other = load_error_file(OTHER_FILE, OTHER_NAME)

ours_segments = get_segments(ours)
other_segments = get_segments(other)

num_changes = min(4, len(ours_segments), len(other_segments))

if num_changes < 4:
    print(f"警告：检测到可用分段数只有 {num_changes} 段，不足 4 段。")


# =========================
# 计算四次下降速率
# =========================
rate_rows = []

for i in range(num_changes):
    ours_seg = ours_segments[i]
    other_seg = other_segments[i]

    rate_rows.append({
        "quantity_change": i + 1,

        "ours_start_error": ours_seg["start_error"],
        "ours_end_error": ours_seg["end_error"],
        "ours_duration_s": ours_seg["duration"],
        "ours_drop_rate_start_to_end": ours_seg["drop_rate"],
        "ours_min_error": ours_seg["min_error"],
        "ours_time_to_min_s": ours_seg["time_to_min"],
        "ours_drop_rate_to_min": ours_seg["drop_rate_to_min"],

        "other_start_error": other_seg["start_error"],
        "other_end_error": other_seg["end_error"],
        "other_duration_s": other_seg["duration"],
        "other_drop_rate_start_to_end": other_seg["drop_rate"],
        "other_min_error": other_seg["min_error"],
        "other_time_to_min_s": other_seg["time_to_min"],
        "other_drop_rate_to_min": other_seg["drop_rate_to_min"],
    })

rate_df = pd.DataFrame(rate_rows)

print("\n四次数量变化后的下降速率：")
print(rate_df.round(6).to_string(index=False))

rate_df.to_csv(
    OUT_DIR / "four_quantity_change_drop_rates.csv",
    index=False,
    encoding="utf-8-sig"
)


# =========================
# 合成大图
# 布局说明：
# 第1行：原始误差曲线图 | 柱状图
# 第2行：四张小的分段对齐图
# =========================
fig = plt.figure(figsize=(18, 8))

gs = fig.add_gridspec(
    2, 4,
    height_ratios=[1.2, 0.8],
    wspace=0.35,
    hspace=0.45
)

# 第一行两个大图
ax_main = fig.add_subplot(gs[0, 0:2])
ax_bar = fig.add_subplot(gs[0, 2:4])

# 第二行四个小图
ax_seg1 = fig.add_subplot(gs[1, 0])
ax_seg2 = fig.add_subplot(gs[1, 1])
ax_seg3 = fig.add_subplot(gs[1, 2])
ax_seg4 = fig.add_subplot(gs[1, 3])

seg_axes = [ax_seg1, ax_seg2, ax_seg3, ax_seg4]


# =========================
# 子图1：原始误差曲线
# =========================
ax_main.plot(
    ours["t_original"],
    ours["error_original"],
    linewidth=2.0,
    label=ours["name"]
)

ax_main.plot(
    other["t_original"],
    other["error_original"],
    linewidth=2.0,
    label=other["name"]
)

ax_main.set_xlabel("Time (s)", fontsize=LARGE_FONT)
ax_main.set_ylabel("Formation error", fontsize=LARGE_FONT)
ax_main.set_title("Original formation error curves", fontsize=LARGE_FONT)

ax_main.tick_params(axis="both", labelsize=SMALL_FONT)
ax_main.grid(True, alpha=0.3)
ax_main.legend(fontsize=SMALL_FONT)


# =========================
# 子图2：四次下降速率柱状图
# =========================
x = np.arange(num_changes)
width = 0.35

ax_bar.bar(
    x - width / 2,
    rate_df["ours_drop_rate_start_to_end"],
    width,
    label=OURS_NAME
)

ax_bar.bar(
    x + width / 2,
    rate_df["other_drop_rate_start_to_end"],
    width,
    label=OTHER_NAME
)

ax_bar.axhline(0, linewidth=0.8)

ax_bar.set_xticks(x)
ax_bar.set_xticklabels(
    [f"Change {i}" for i in range(1, num_changes + 1)],
    fontsize=SMALL_FONT
)

ax_bar.set_xlabel("Quantity change index", fontsize=LARGE_FONT)
ax_bar.set_ylabel(r"Average error decrease rate $r_e$", fontsize=LARGE_FONT)
ax_bar.set_title("Average convergence rate", fontsize=LARGE_FONT)

ax_bar.tick_params(axis="both", labelsize=SMALL_FONT)
ax_bar.grid(True, axis="y", alpha=0.3)
ax_bar.legend(fontsize=SMALL_FONT)


# =========================
# 子图3-6：四个分段对齐图
# =========================
for i in range(min(4, num_changes)):
    ax = seg_axes[i]

    ours_seg = ours_segments[i]
    other_seg = other_segments[i]

    ours_rel = ours_seg["error"] - ours_seg["error"][0]
    other_rel = other_seg["error"] - other_seg["error"][0]

    ax.plot(
        ours_seg["tau"],
        ours_rel,
        linewidth=1.6,
        label=rf"{OURS_NAME}, $r_e$={ours_seg['drop_rate']:.3f}"
    )

    ax.plot(
        other_seg["tau"],
        other_rel,
        linewidth=1.6,
        label=rf"{OTHER_NAME}, $r_e$={other_seg['drop_rate']:.3f}"
    )

    ax.axhline(0, linewidth=0.8, linestyle="--", alpha=0.6)

    ax.set_title(f"Number change {i + 1}", fontsize=LARGE_FONT)
    ax.set_xlabel("Time (s)", fontsize=LARGE_FONT)
    ax.set_ylabel("Relative error change", fontsize=LARGE_FONT)

    ax.tick_params(axis="x", labelsize=SMALL_FONT)
    ax.tick_params(axis="y", labelleft=False, labelsize=SMALL_FONT)

    ax.grid(True, alpha=0.3)

    # 关键修改：
    # 固定每个小图的图例在小图上方，避免自动跑到中间
    ax.legend(
        fontsize=SMALL_FONT,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.98),
        framealpha=0.7,
        borderaxespad=0.2
    )


# 如果分段不足4个，关闭多余子图
for j in range(num_changes, 4):
    seg_axes[j].axis("off")


# =========================
# 总标题与保存
# =========================
fig.suptitle(
    "Formation error comparison and segment-wise convergence analysis",
    fontsize=LARGE_FONT,
    y=0.98
)

plt.savefig(
    OUT_DIR / "combined_figure_top2_bottom4.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()
