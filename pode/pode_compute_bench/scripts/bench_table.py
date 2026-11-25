#!/usr/bin/env python3
"""compute_bench 결과(samples.csv, runs.csv)를 요약 CSV와 Markdown 표로 만든다.

    bench_table.py RESULT_DIR [--title 제목]   -> RESULT_DIR/summary.csv, RESULT_DIR/table.md
"""
import argparse
import csv
import math
from pathlib import Path

BUDGET_MS = 50.0  # 20 Hz 한 주기
FIELDS = ["case", "plugin", "batch_size", "time_steps", "model_dt", "rate_hz", "n", "errors",
          "overruns", "p50_ms", "p99_ms", "max_ms", "mean_ms", "p99_over_50ms",
          "proc_cpu_pct", "thread_cpu_pct"]


def percentile(values, q):
    """가장 가까운 순위 백분위(q는 0~100)."""
    if not values:
        raise ValueError("no samples")
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


def number_text(text):
    """C++ to_string의 '0.050000'을 '0.05'로. 빈 칸은 그대로."""
    if text == "":
        return ""
    return f"{float(text):g}"


def summarize(samples, runs):
    """samples: [{case, ms}], runs: [runs.csv 행] -> 경우마다 요약 dict(runs 순서)."""
    by_case = {}
    for row in samples:
        by_case.setdefault(row["case"], []).append(float(row["ms"]))
    out = []
    for run in runs:
        ms = by_case.get(run["case"], [])
        wall = float(run["wall_s"])
        proc = float(run["proc_user_s"]) + float(run["proc_sys_s"])
        p99 = percentile(ms, 99)
        out.append({
            "case": run["case"],
            "plugin": run["plugin"],
            "batch_size": run["batch_size"],
            "time_steps": run["time_steps"],
            "model_dt": number_text(run["model_dt"]),
            "rate_hz": number_text(run["rate_hz"]),
            "n": len(ms),
            "errors": int(run["errors"]),
            "overruns": int(run["overruns"]),
            "p50_ms": percentile(ms, 50),
            "p99_ms": p99,
            "max_ms": max(ms),
            "mean_ms": sum(ms) / len(ms),
            "p99_over_50ms": p99 / BUDGET_MS,
            "proc_cpu_pct": 100.0 * proc / wall,
            "thread_cpu_pct": 100.0 * float(run["thread_cpu_s"]) / wall,
        })
    return out


def markdown(rows, title):
    head = ("| 경우 | 플러그인 | batch | steps | dt (s) | 주기 (Hz) | n | 오류 | 넘침 | p50 (ms) | "
            "p99 (ms) | max (ms) | 평균 (ms) | p99 / 50 ms | 프로세스 CPU (%) | 측정 스레드 CPU (%) |")
    lines = [f"### {title}", "", head, "|" + "---|" * 16]
    for r in rows:
        lines.append(
            f"| {r['case']} | `{r['plugin']}` | {r['batch_size']} | {r['time_steps']} | "
            f"{r['model_dt']} | {r['rate_hz']} | {r['n']} | {r['errors']} | {r['overruns']} | "
            f"{r['p50_ms']:.3f} | {r['p99_ms']:.3f} | {r['max_ms']:.3f} | {r['mean_ms']:.3f} | "
            f"{r['p99_over_50ms']:.3f} | {r['proc_cpu_pct']:.1f} | {r['thread_cpu_pct']:.1f} |")
    lines += ["", "넘침: 그 경우의 주기(1/주기 Hz)보다 오래 걸린 호출 수. 프로세스 CPU: 측정 구간의 "
              "(user+system) / 벽시계, 한 코어 = 100 %."]
    return "\n".join(lines) + "\n"


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("result_dir")
    parser.add_argument("--title", default="compute_bench")
    args = parser.parse_args(argv)
    d = Path(args.result_dir)
    rows = summarize(read_csv(d / "samples.csv"), read_csv(d / "runs.csv"))
    with open(d / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    text = markdown(rows, args.title)
    (d / "table.md").write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
