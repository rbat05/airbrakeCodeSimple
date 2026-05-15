#!/usr/bin/env python3
"""
analyse_loop.py — parse ESP32 loop-timer serial logs and produce a report.

Usage:
    python analyse_loop.py COM12_log.txt
    python analyse_loop.py COM12_log.txt --html report.html
    python analyse_loop.py COM12_log.txt --plot

# Terminal report only
python analyse_loop.py COM12_log.txt

# Terminal + save HTML report
python analyse_loop.py COM12_log.txt --html report.html

# Terminal + matplotlib charts
python analyse_loop.py COM12_log.txt --plot

# All three
python analyse_loop.py COM12_log.txt --html report.html --plot
"""

import re
import sys
import argparse
from pathlib import Path
 
# ── Regex ──────────────────────────────────────────────────────────────────────
RE_LOOP = re.compile(
    r"dt:\s*([\d.]+)\s*ms.*?busy:\s*([\d.]+)\s*ms.*?"
    r"avg busy:\s*([\d.]+)\s*ms.*?max busy:\s*([\d.]+)\s*ms.*?overruns:\s*(\d+)"
)
RE_APOGEE  = re.compile(r"Apogee reached", re.IGNORECASE)
RE_LAUNCH  = re.compile(r"launch|liftoff|ignition", re.IGNORECASE)
 
SPIKE_THRESHOLD_MS   = 20.0   # dt above this → overrun / SD flush spike
FLIGHT_BUSY_THRESHOLD = 0.5   # busy above this for N consecutive → launch detected
FLIGHT_CONFIRM_RUNS  = 5
 
 
# ── Parsing ───────────────────────────────────────────────────────────────────
def parse(path: str):
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    records = []
    apogee_line  = None
    explicit_launch_line = None
 
    for i, line in enumerate(lines):
        m = RE_LOOP.search(line)
        if m:
            records.append({
                "line":     i + 1,
                "idx":      len(records),
                "dt":       float(m.group(1)),
                "busy":     float(m.group(2)),
                "avg_busy": float(m.group(3)),
                "max_busy": float(m.group(4)),
                "overruns": int(m.group(5)),
            })
        if RE_APOGEE.search(line):
            apogee_line = i + 1
        if RE_LAUNCH.search(line) and not RE_LOOP.search(line):
            explicit_launch_line = i + 1
 
    return records, apogee_line, explicit_launch_line
 
 
# ── Phase detection ───────────────────────────────────────────────────────────
def detect_launch(records):
    for i, r in enumerate(records[FLIGHT_CONFIRM_RUNS:], start=FLIGHT_CONFIRM_RUNS):
        window = records[i : i + FLIGHT_CONFIRM_RUNS]
        if len(window) == FLIGHT_CONFIRM_RUNS and all(
            w["busy"] > FLIGHT_BUSY_THRESHOLD for w in window
        ):
            return i
    return None
 
 
# ── Stats helpers ─────────────────────────────────────────────────────────────
def stats(records):
    if not records:
        return {}
    busy  = [r["busy"] for r in records]
    dts   = [r["dt"]   for r in records if r["dt"] > 1]
    spikes = [r for r in records if r["dt"] > SPIKE_THRESHOLD_MS]
    overruns_total = max(r["overruns"] for r in records) if records else 0
    return {
        "count":          len(records),
        "duration_s":     len(records) * 10 / 1000,
        "busy_avg":       sum(busy) / len(busy),
        "busy_min":       min(busy),
        "busy_max":       max(busy),
        "budget_pct":     sum(busy) / len(busy) / 10 * 100,
        "headroom_ms":    10 - sum(busy) / len(busy),
        "overruns":       overruns_total,
        "spikes":         spikes,
        "spike_count":    len(spikes),
        "spike_dts":      [round(r["dt"], 2) for r in spikes],
        "spike_intervals": (
            [spikes[j + 1]["line"] - spikes[j]["line"] for j in range(len(spikes) - 1)]
            if len(spikes) >= 2 else []
        ),
    }
 
 
# ── Terminal report ───────────────────────────────────────────────────────────
W = 52
 
def hr(char="═"):    print(char * W)
def row(k, v):       print(f"  {k:<28}{v:>20}")
 
def print_report(path, records, apogee_line, launch_idx, idle_st, flight_st):
    hr()
    print(f"{'LOOP TIMING ANALYSIS':^{W}}")
    print(f"  {Path(path).name}")
    hr()
    row("Total records",           str(len(records)))
    row("Apogee line",             str(apogee_line) if apogee_line else "not found")
    row("Launch detected at idx",  str(launch_idx) if launch_idx is not None else "not detected")
    hr("─")
 
    for phase_name, st in [("IDLE PHASE", idle_st), ("FLIGHT PHASE (launch→apogee)", flight_st)]:
        if not st:
            continue
        print(f"\n  {phase_name}")
        hr("─")
        row("Records",             str(st["count"]))
        row("Duration",            f"{st['duration_s']:.1f} s")
        row("Avg busy",            f"{st['busy_avg']:.3f} ms")
        row("Min / max busy",      f"{st['busy_min']:.3f} / {st['busy_max']:.3f} ms")
        row("Budget used",         f"{st['budget_pct']:.1f}%")
        row("Headroom (avg)",      f"{st['headroom_ms']:.2f} ms")
        row("Total overruns",      str(st["overruns"]))
        row("SD flush spikes",     str(st["spike_count"]))
        if st["spike_intervals"]:
            avg_interval_ms = sum(st["spike_intervals"]) / len(st["spike_intervals"]) * 10
            row("Avg spike interval",  f"{avg_interval_ms:.0f} ms")
        if st["spike_dts"]:
            row("Spike dur range",
                f"{min(st['spike_dts']):.1f} – {max(st['spike_dts']):.1f} ms")
 
    hr()
    print()
 
 
# ── Plot ───────────────────────────────────────────────────────────────────────
def plot(records, launch_idx, apogee_line):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("matplotlib not installed: pip install matplotlib")
        return
 
    idxs  = [r["idx"]  for r in records]
    busy  = [r["busy"] for r in records]
    dts   = [r["dt"]   for r in records]
 
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("ESP32 Loop Timer Analysis", fontsize=13, fontweight="bold")
 
    # ── Busy time ─────────────────────────────────────────────────────────────
    colors = []
    for r in records:
        if r["dt"] > SPIKE_THRESHOLD_MS:
            colors.append("#E24B4A")
        elif launch_idx and r["idx"] >= launch_idx:
            colors.append("#639922")
        else:
            colors.append("#378ADD")
    ax1.bar(idxs, busy, color=colors, width=1.0)
    if launch_idx is not None:
        ax1.axvline(launch_idx, color="#BA7517", lw=1.5, ls="--", label="Launch")
    ax1.set_ylabel("Busy time (ms)")
    ax1.set_title("Busy time per loop iteration")
    ax1.legend(handles=[
        mpatches.Patch(color="#378ADD", label="Idle"),
        mpatches.Patch(color="#639922", label="Flight"),
        mpatches.Patch(color="#E24B4A", label="SD flush spike"),
    ], fontsize=9)
    ax1.grid(axis="y", alpha=0.3)
 
    # ── dt ────────────────────────────────────────────────────────────────────
    ax2.bar(idxs, dts, color=colors, width=1.0)
    ax2.axhline(10, color="#888", lw=1, ls=":", label="10 ms target")
    if launch_idx is not None:
        ax2.axvline(launch_idx, color="#BA7517", lw=1.5, ls="--")
    ax2.set_ylabel("dt (ms)")
    ax2.set_xlabel("Loop index")
    ax2.set_title("Loop period (dt) — spikes = missed deadline")
    ax2.legend(fontsize=9)
    ax2.grid(axis="y", alpha=0.3)
 
    plt.tight_layout()
    plt.show()
 
 
# ── HTML report ───────────────────────────────────────────────────────────────
def write_html(path, records, launch_idx, idle_st, flight_st, apogee_line, out_path):
    busy_data  = [r["busy"] for r in records]
    dt_data    = [r["dt"]   for r in records]
    spike_mask = [1 if r["dt"] > SPIKE_THRESHOLD_MS else 0 for r in records]
    flight_mask = [1 if (launch_idx and r["idx"] >= launch_idx) else 0 for r in records]
 
    def fmt(v, decimals=3): return f"{v:.{decimals}f}"
 
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Loop Timing Report — {Path(path).name}</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js"></script>
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ font-family: 'Courier New', monospace; background: #0e0e0e; color: #d4d4d4; padding: 2rem; }}
  h1 {{ font-size: 1.2rem; color: #fff; margin-bottom: 0.25rem; }}
  .sub {{ font-size: 0.8rem; color: #666; margin-bottom: 2rem; }}
  .grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; margin-bottom: 2rem; }}
  .card {{ background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 6px; padding: 1rem; }}
  .card .label {{ font-size: 11px; color: #666; margin-bottom: 6px; text-transform: uppercase; letter-spacing: 0.05em; }}
  .card .value {{ font-size: 1.6rem; color: #fff; font-weight: bold; }}
  .card .value.warn {{ color: #E24B4A; }}
  .card .value.ok   {{ color: #639922; }}
  .phases {{ display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 2rem; }}
  .phase {{ background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 6px; padding: 1rem; }}
  .phase h2 {{ font-size: 0.85rem; color: #888; margin-bottom: 0.75rem; text-transform: uppercase; letter-spacing: 0.05em; }}
  .phase table {{ width: 100%; font-size: 12px; border-collapse: collapse; }}
  .phase td {{ padding: 3px 0; }}
  .phase td:last-child {{ text-align: right; color: #fff; }}
  .chart-wrap {{ background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 6px; padding: 1rem; margin-bottom: 1.5rem; }}
  .chart-wrap h2 {{ font-size: 0.85rem; color: #888; margin-bottom: 1rem; text-transform: uppercase; letter-spacing: 0.05em; }}
  .legend {{ display: flex; gap: 20px; font-size: 11px; color: #666; margin-top: 10px; }}
  .legend span {{ display: flex; align-items: center; gap: 6px; }}
  .dot {{ width: 10px; height: 10px; border-radius: 2px; display: inline-block; }}
</style>
</head>
<body>
<h1>Loop Timing Report</h1>
<div class="sub">{Path(path).name} — {len(records)} records</div>
 
<div class="grid">
  <div class="card"><div class="label">Total records</div><div class="value">{len(records)}</div></div>
  <div class="card"><div class="label">Flight duration</div><div class="value">{fmt(flight_st['duration_s'], 1) if flight_st else 'N/A'} s</div></div>
  <div class="card"><div class="label">Total overruns</div><div class="value warn">{idle_st['overruns'] + (flight_st['overruns'] if flight_st else 0)}</div></div>
  <div class="card"><div class="label">Flight budget used</div><div class="value ok">{fmt(flight_st['budget_pct'], 1) if flight_st else 'N/A'}%</div></div>
</div>
 
<div class="phases">
  <div class="phase">
    <h2>Idle phase</h2>
    <table>
      <tr><td style="color:#666">Records</td><td>{idle_st['count']}</td></tr>
      <tr><td style="color:#666">Duration</td><td>{fmt(idle_st['duration_s'],1)} s</td></tr>
      <tr><td style="color:#666">Avg busy</td><td>{fmt(idle_st['busy_avg'])} ms</td></tr>
      <tr><td style="color:#666">Budget used</td><td>{fmt(idle_st['budget_pct'],1)}%</td></tr>
      <tr><td style="color:#666">SD flush spikes</td><td style="color:#E24B4A">{idle_st['spike_count']}</td></tr>
      <tr><td style="color:#666">Spike interval</td><td>~{round(sum(idle_st['spike_intervals'])/max(len(idle_st['spike_intervals']),1)*10)} ms</td></tr>
      <tr><td style="color:#666">Spike dur range</td><td>{min(idle_st['spike_dts']):.1f} – {max(idle_st['spike_dts']):.1f} ms</td></tr>
    </table>
  </div>
  <div class="phase">
    <h2>Flight phase (launch → apogee)</h2>
    <table>
      <tr><td style="color:#666">Records</td><td>{flight_st['count'] if flight_st else 'N/A'}</td></tr>
      <tr><td style="color:#666">Duration</td><td>{fmt(flight_st['duration_s'],1) if flight_st else 'N/A'} s</td></tr>
      <tr><td style="color:#666">Avg busy</td><td>{fmt(flight_st['busy_avg']) if flight_st else 'N/A'} ms</td></tr>
      <tr><td style="color:#666">Budget used</td><td>{fmt(flight_st['budget_pct'],1) if flight_st else 'N/A'}%</td></tr>
      <tr><td style="color:#666">Headroom (avg)</td><td style="color:#639922">{fmt(flight_st['headroom_ms']) if flight_st else 'N/A'} ms</td></tr>
      <tr><td style="color:#666">SD flush spikes</td><td style="color:#E24B4A">{flight_st['spike_count'] if flight_st else 'N/A'}</td></tr>
      <tr><td style="color:#666">Spike dur range</td><td>{min(flight_st['spike_dts']):.1f} – {max(flight_st['spike_dts']):.1f} ms</td></tr>
    </table>
  </div>
</div>
 
<div class="chart-wrap">
  <h2>Busy time per iteration (ms)</h2>
  <div style="position:relative;height:220px;"><canvas id="c1" role="img" aria-label="Busy time chart">Busy time per loop iteration</canvas></div>
  <div class="legend">
    <span><span class="dot" style="background:#378ADD"></span>Idle</span>
    <span><span class="dot" style="background:#639922"></span>Flight</span>
    <span><span class="dot" style="background:#E24B4A"></span>SD flush spike</span>
    {"<span><span class='dot' style='background:#BA7517'></span>Launch</span>" if launch_idx else ""}
  </div>
</div>
 
<div class="chart-wrap">
  <h2>Loop period dt (ms) — spikes = missed deadline</h2>
  <div style="position:relative;height:220px;"><canvas id="c2" role="img" aria-label="dt chart">Loop dt per iteration</canvas></div>
</div>
 
<script>
const busy = {busy_data};
const dt   = {dt_data};
const spike = {spike_mask};
const flight = {flight_mask};
const launchIdx = {launch_idx if launch_idx is not None else 'null'};
 
function barColors(arr, useArr) {{
  return arr.map((_, i) => {{
    if (spike[i]) return '#E24B4A';
    if (flight[i]) return '#639922';
    return '#378ADD';
  }});
}}
 
const opts = (yLabel, max) => ({{
  responsive: true, maintainAspectRatio: false, animation: false,
  plugins: {{ legend: {{ display: false }}, tooltip: {{ callbacks: {{ label: c => c.parsed.y.toFixed(2) + ' ms' }} }} }},
  scales: {{
    x: {{ display: false }},
    y: {{ max, grid: {{ color: 'rgba(255,255,255,0.05)' }}, ticks: {{ color: '#666', callback: v => v + ' ms' }}, title: {{ display: true, text: yLabel, color: '#666', font: {{ size: 11 }} }} }}
  }}
}});
 
new Chart(document.getElementById('c1'), {{
  type: 'bar',
  data: {{ labels: busy.map((_, i) => i), datasets: [{{ data: busy, backgroundColor: barColors(busy), barPercentage: 1, categoryPercentage: 1 }}] }},
  options: opts('ms', Math.max(...busy) * 1.1)
}});
 
new Chart(document.getElementById('c2'), {{
  type: 'bar',
  data: {{ labels: dt.map((_, i) => i), datasets: [
    {{ data: dt, backgroundColor: barColors(dt), barPercentage: 1, categoryPercentage: 1 }},
    {{ type: 'line', data: dt.map(() => 10), borderColor: '#555', borderWidth: 1, borderDash: [4,4], pointRadius: 0, label: '10ms target' }}
  ] }},
  options: opts('ms', Math.max(...dt) * 1.1)
}});
</script>
</body>
</html>"""
 
    Path(out_path).write_text(html, encoding="utf-8")
    print(f"[info] HTML report written to '{out_path}'")
 
 
# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Analyse ESP32 loop-timer serial log")
    parser.add_argument("input",               help="Path to serial log .txt file")
    parser.add_argument("-o", "--html",        help="Write HTML report to this path")
    parser.add_argument("--plot", action="store_true", help="Show matplotlib chart")
    args = parser.parse_args()
 
    records, apogee_line, explicit_launch = parse(args.input)
    if not records:
        print("[error] No loop records found in file.", file=sys.stderr)
        sys.exit(1)
 
    launch_idx = detect_launch(records)
    idle_records   = records[:launch_idx] if launch_idx else records
    flight_records = records[launch_idx:] if launch_idx else []
 
    idle_st   = stats(idle_records)
    flight_st = stats(flight_records) if flight_records else None
 
    # Normalise overruns to phase-relative counts
    if flight_st and idle_st:
        flight_st["overruns"] = flight_st["overruns"] - idle_st["overruns"]
 
    print_report(args.input, records, apogee_line, launch_idx, idle_st, flight_st)
 
    if args.html:
        write_html(args.input, records, launch_idx, idle_st, flight_st, apogee_line, args.html)
 
    if args.plot:
        plot(records, launch_idx, apogee_line)
 
 
if __name__ == "__main__":
    main()
