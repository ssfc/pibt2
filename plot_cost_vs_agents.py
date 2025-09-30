#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 result/experimental_results.csv，筛选出指定 map_file 与 agent_file 的数据，
并对 CBS / CBSFlow / CBSFlowBeam2 / CBSFlowBeam8 绘制 num_agents vs cost 的散点图。
"""

import argparse
import os
import sys
from typing import Dict, Tuple

import pandas as pd
import matplotlib.pyplot as plt


def filter_and_prepare(df: pd.DataFrame, filters: Dict[str, object],
                       xcol: str = "num_agents", ycol: str = "cost") -> pd.DataFrame:
    """按 filters 过滤，并将 x/y 列转为数值、去 NaN、按 x 排序。"""
    mask = pd.Series(True, index=df.index)
    for col, val in filters.items():
        # 若列不存在，视为无法匹配
        if col not in df.columns:
            return pd.DataFrame(columns=[xcol, ycol])
        mask &= (df[col] == val)

    sub = df.loc[mask, [xcol, ycol]].copy()
    if sub.empty:
        return sub

    sub[xcol] = pd.to_numeric(sub[xcol], errors="coerce")
    sub[ycol] = pd.to_numeric(sub[ycol], errors="coerce")
    sub = sub.dropna(subset=[xcol, ycol]).sort_values(by=xcol)
    return sub


def main():
    parser = argparse.ArgumentParser(description="绘制 num_agents vs SOC 散点图（按指定的 map_file 与 agent_file 筛选）")
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
        help="输出图片路径（留空则直接显示图窗，例如：plots/soc_vs_agents.png）",
    )
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"未找到 CSV 文件：{args.csv}", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(args.csv)

    # 公共筛选条件
    base_filters = {
        "map_file": args.map_file,
        "agent_file": args.agent_file,
        "device": "12400F",
        "disappear_at_goal": 2,
    }

    # 不同方法的附加条件与样式
    methods: Tuple[Tuple[str, Dict[str, object], Dict[str, object]], ...] = (
        #("PIBT", {"high level planner": "no", "low level planner": "PIBT"}, {"marker": "^"}),
        ("PIBT-allTimeFlow", {"high level planner": "no", "low level planner": "PIBT-allTimeFlow"}, {"marker": "v"}),
        ("PIBT-allTimeFlow-reverse", {"high level planner": "no", "low level planner": "PIBT-allTimeFlow-reverse"}, {"marker": "^"}),
        #("CBS", {"high level planner": "CBS-whoenig"}, {"marker": "^"}),
        #("CBSDepth", {"high level planner": "CBSDepth-whoenig"}, {"marker": "v"}),
        #("CBSDepthKNN", {"high level planner": "CBSDepthKNN"}, {"marker": "^"}),
        #("CBSDepthKNNSparse", {"high level planner": "CBSDepthKNNSparse"}, {"marker": "v"}),
        #("CBSDepthSelectConflict", {"high level planner": "CBSDepthSelectConflict"}, {"marker": "^"}),
        # ("CBSDepthSecond", {"high level planner": "CBSDepthSecondPriority"}, {"marker": "^"}),
        # ("CBSDepthRandomAvoid", {"high level planner": "CBSDepthRandomAvoid"}, {"marker": "^"}),
        #("CBSDepthIncrementalUpdateOrder", {"high level planner": "CBSDepthIncrementalUpdateOrder"}, {"marker": "^"}),
        #("CBSDepthLazyAvoidBuzy", {"high level planner": "CBSDepthLazyAvoidBusy"}, {"marker": "v"}),
        #("SelectConflict+constraintByOrder", {"high level planner": "CBSDepthSelectConflict+constraintByOrder"}, {"marker": "*"}),
        #("CBSDepthLengthOrder", {"high level planner": "CBSDepthLengthOrder"}, {"marker": "^"}),
        #("CBSDepthLengthOrderReverse", {"high level planner": "CBSDepthLengthOrderReverse"}, {"marker": "^"}),
        #("CBSDepthBeam2", {"high level planner": "CBSFlowBeam-whoenig", "comment": 2}, {"marker": "*"}),
        #("CBSDepthBeam8", {"high level planner": "CBSFlowBeam-whoenig", "comment": 8}, {"marker": "v"}),
    )

    prepared = {}
    missing = []

    for name, extra_filters, _style in methods:
        filters = {**base_filters, **extra_filters}
        sub = filter_and_prepare(df, filters, xcol="num_agents", ycol="cost")
        if sub.empty:
            missing.append(name)
        else:
            prepared[name] = sub

    if missing:
        print(
            "筛选结果为空，请检查 map_file 与 agent_file 是否正确，或数据是否存在："
            + ", ".join(missing),
            file=sys.stderr,
        )
        sys.exit(2)

    # 绘图
    plt.figure(figsize=(8, 5))
    for name, extra_filters, style in methods:
        sub = prepared.get(name)
        if sub is None:
            continue
        plt.scatter(
            sub["num_agents"],
            sub["cost"],
            s=28,
            alpha=0.8,
            edgecolor="k",
            linewidths=0.3,
            label=name,
            **style,
        )

    plt.legend()

    active_names = [name for name, _, _ in methods if name in prepared]
    plt.title("Comparison: " + ", ".join(active_names), fontsize=12)

    plt.xlabel("num_agents (x-axis)", fontsize=11)
    plt.ylabel("sum of cost (y-axis)", fontsize=11)
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()

    if args.out:
        out_dir = os.path.dirname(args.out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(args.out, dpi=150)
        print(f"已保存图表到：{args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()