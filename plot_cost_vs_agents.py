#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 result/experimental_results.csv，筛选出：
- map_file == "random-32-32-20.map"
- agent_file == "random-32-32-20-random-1.scen"
并绘制 num_agents (横轴) 与 cost (纵轴) 的散点图。
"""

import argparse
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="绘制 num_agents vs cost 散点图（按指定的 map_file 与 agent_file 筛选）")
    parser.add_argument(
        "--csv",
        default="result/experimental_results.csv",
        help="CSV 文件路径（默认：result/experimental_results.csv）",
    )
    parser.add_argument(
        "--map-file",
        default="random-32-32-20.map",
        help='map_file 过滤值（默认："random-32-32-20.map"）',
    )
    parser.add_argument(
        "--agent-file",
        default="random-32-32-20-random-1.scen",
        help='agent_file 过滤值（默认："random-32-32-20-random-1.scen"）',
    )
    parser.add_argument(
        "--out",
        default="",
        help="输出图片路径（留空则直接显示图窗，例如：plots/cost_vs_agents.png）",
    )
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"未找到 CSV 文件：{args.csv}", file=sys.stderr)
        sys.exit(1)

    # 读取 CSV
    df = pd.read_csv(args.csv)

    args.map_file = "arena.map"
    args.agent_file = "randGen"

    # 筛选条件
    mask_CBS = (
            (df.get("map_file") == args.map_file) &
            (df.get("agent_file") == args.agent_file) &
            (df.get("device") == "12400F") &
            (df.get("low level planner") == "PIBT") &
            (df.get("disappear_at_goal") == 2)
    )

    mask_CBSFlow = (
            (df.get("map_file") == args.map_file) &
            (df.get("agent_file") == args.agent_file) &
            (df.get("device") == "12400F") &
            (df.get("low level planner") == "PIBT-allTimeFlow") &
            (df.get("disappear_at_goal") == 2)
    )

    filtered_CBS = df[mask_CBS].copy()
    filtered_CBSFlow = df[mask_CBSFlow].copy()

    if filtered_CBS.empty:
        print("筛选结果为空，请检查 map_file 与 agent_file 是否正确，或数据是否存在。", file=sys.stderr)
        sys.exit(2)

    if filtered_CBSFlow.empty:
        print("筛选结果为空，请检查 map_file 与 agent_file 是否正确，或数据是否存在。", file=sys.stderr)
        sys.exit(2)

    # 转换为数值类型，无法转换的设为 NaN 并剔除
    filtered_CBS["num_agents"] = pd.to_numeric(filtered_CBS["num_agents"], errors="coerce")
    filtered_CBS["cost"] = pd.to_numeric(filtered_CBS["cost"], errors="coerce")
    filtered_CBS = filtered_CBS.dropna(subset=["num_agents", "cost"])

    if filtered_CBSFlow.empty:
        print("筛选后数值列为空（num_agents 或 cost 无法转换为数值）。", file=sys.stderr)
        sys.exit(3)

    filtered_CBSFlow["num_agents"] = pd.to_numeric(filtered_CBSFlow["num_agents"], errors="coerce")
    filtered_CBSFlow["cost"] = pd.to_numeric(filtered_CBSFlow["cost"], errors="coerce")
    filtered_CBSFlow = filtered_CBSFlow.dropna(subset=["num_agents", "cost"])

    if filtered_CBSFlow.empty:
        print("筛选后数值列为空（num_agents 或 cost 无法转换为数值）。", file=sys.stderr)
        sys.exit(3)

    if filtered_CBSFlow.empty:
        print("筛选后数值列为空（num_agents 或 cost 无法转换为数值）。", file=sys.stderr)
        sys.exit(3)

    # 按 num_agents 排序（便于观察）
    filtered_CBS = filtered_CBS.sort_values(by="num_agents")
    filtered_CBSFlow = filtered_CBSFlow.sort_values(by="num_agents")

    # 绘图
    plt.figure(figsize=(8, 5))
    plt.scatter(filtered_CBS["num_agents"], filtered_CBS["cost"], s=28, alpha=0.8, edgecolor="k",
                linewidths=0.3, label="CBS", marker="^")
    plt.scatter(filtered_CBSFlow["num_agents"], filtered_CBSFlow["cost"], s=28, alpha=0.8, edgecolor="k", linewidths=0.3,
            label="CBSFlow", marker="o")

    plt.legend()  # 添加图例说明

    plt.title("Comparision: CBS, CBSFlow, CBSFlowBeam2", fontsize=12)
    plt.xlabel("num_agents (x-axis)", fontsize=11)
    plt.ylabel("cost (y-axis)", fontsize=11)
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True) if os.path.dirname(args.out) else None
        plt.savefig(args.out, dpi=150)
        print(f"已保存图表到：{args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()