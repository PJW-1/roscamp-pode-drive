"""bench_table.py를 합성 자료로 시험한다(ROS 없이)."""
import csv
import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "bench_table.py"
spec = importlib.util.spec_from_file_location("bench_table", SCRIPT)
bench_table = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench_table)


def run_row(case, **kw):
    row = {"case": case, "kind": "controller", "plugin": "p::C", "rate_hz": "20",
           "batch_size": "", "time_steps": "", "model_dt": "", "n": "100", "errors": "0",
           "overruns": "0", "wall_s": "10", "proc_user_s": "1.5", "proc_sys_s": "0.5",
           "thread_cpu_s": "1", "first_error": ""}
    row.update(kw)
    return row


def test_percentile_nearest_rank():
    v = list(range(1, 101))
    assert bench_table.percentile(v, 50) == 50
    assert bench_table.percentile(v, 99) == 99
    assert bench_table.percentile(v, 100) == 100
    assert bench_table.percentile([7.0], 99) == 7.0
    with pytest.raises(ValueError):
        bench_table.percentile([], 50)


def test_summarize_values_and_order():
    samples = [{"case": "a", "ms": str(m)} for m in range(1, 101)]
    samples += [{"case": "b", "ms": "2.0"}, {"case": "b", "ms": "4.0"}]
    runs = [run_row("b", model_dt="0.050000", rate_hz="10.000000", batch_size="1000"),
            run_row("a")]
    rows = bench_table.summarize(samples, runs)
    assert [r["case"] for r in rows] == ["b", "a"]
    b, a = rows
    assert b["model_dt"] == "0.05" and b["rate_hz"] == "10" and b["batch_size"] == "1000"
    assert b["mean_ms"] == pytest.approx(3.0) and b["max_ms"] == 4.0 and b["p50_ms"] == 2.0
    assert a["p99_ms"] == 99 and a["p99_over_50ms"] == pytest.approx(99 / 50)
    assert a["proc_cpu_pct"] == pytest.approx(20.0)
    assert a["thread_cpu_pct"] == pytest.approx(10.0)
    assert a["n"] == 100


def test_main_writes_csv_and_table(tmp_path):
    with open(tmp_path / "samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case", "iter", "ms"])
        for i, m in enumerate([1.0, 2.0, 3.0]):
            w.writerow(["rpp", i, m])
    with open(tmp_path / "runs.csv", "w", newline="") as f:
        row = run_row("rpp")
        w = csv.DictWriter(f, fieldnames=list(row))
        w.writeheader()
        w.writerow(row)
    bench_table.main([str(tmp_path), "--title", "합성"])
    summary = list(csv.DictReader(open(tmp_path / "summary.csv", encoding="utf-8")))
    assert summary[0]["case"] == "rpp" and summary[0]["max_ms"] == "3.0000"
    table = (tmp_path / "table.md").read_text(encoding="utf-8")
    assert "### 합성" in table
    assert "| rpp | `p::C` |" in table
