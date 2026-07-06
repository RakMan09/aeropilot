#!/usr/bin/env python3
"""Render an animated GIF/MP4 of an AeroPilot flight from decoded telemetry
JSON (as produced by `decode.py --json`).

Usage:
    python groundstation/animate.py --json web/data/flight_nominal.json \
        --out docs/flight.gif [--fps 15] [--max-frames 90]
"""
import argparse
import json
import sys

STATE_COLORS = {
    "IDLE": "#6b7c91", "ARMED": "#f5c451", "BOOST": "#ff8a3d",
    "COAST": "#35c9e6", "APOGEE": "#d977ff", "DESCENT": "#4f8cff",
    "LANDED": "#3ddc84", "SAFE": "#ff4d5e",
}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Animate an AeroPilot flight")
    ap.add_argument("--json", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--max-frames", type=int, default=90)
    args = ap.parse_args(argv)

    with open(args.json) as fh:
        data = json.load(fh)
    frames = data["frames"]
    meta = data["meta"]
    if not frames:
        print("no frames", file=sys.stderr)
        return 1

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    t = [f["t"] for f in frames]
    alt = [f["alt"] for f in frames]
    vel = [f["vel"] for f in frames]

    # Downsample the indices we render as keyframes.
    n = len(frames)
    step = max(1, n // args.max_frames)
    keyframes = list(range(0, n, step)) + [n - 1]

    plt.style.use("dark_background")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fig.patch.set_facecolor("#0a0e14")
    for ax in (ax1, ax2):
        ax.set_facecolor("#111823")
        ax.grid(True, color="#1e2a3a")
        ax.set_xlim(0, meta.get("duration_s", t[-1]))

    ax1.set_ylim(min(alt) - 10, max(alt) * 1.15)
    ax1.set_ylabel("Altitude (m)")
    ax2.set_ylim(min(vel) * 1.2, max(vel) * 1.2)
    ax2.set_ylabel("Vertical velocity (m/s)")
    ax2.set_xlabel("Time (s)")

    # apogee / deploy / safe markers
    if meta.get("apogee_t") is not None:
        for ax in (ax1, ax2):
            ax.axvline(meta["apogee_t"], color=STATE_COLORS["APOGEE"],
                       ls="--", lw=1, alpha=0.6)
    if meta.get("deploy_t") is not None:
        for ax in (ax1, ax2):
            ax.axvline(meta["deploy_t"], color=STATE_COLORS["LANDED"],
                       ls="--", lw=1, alpha=0.6)

    (line_a,) = ax1.plot([], [], color="#35c9e6", lw=2)
    (line_v,) = ax2.plot([], [], color="#8affc1", lw=2)
    (dot_a,) = ax1.plot([], [], "o", color="#35c9e6", ms=7)
    (dot_v,) = ax2.plot([], [], "o", color="#8affc1", ms=7)
    title = ax1.set_title("", fontsize=13, family="monospace")

    def draw(i):
        f = frames[i]
        line_a.set_data(t[:i + 1], alt[:i + 1])
        line_v.set_data(t[:i + 1], vel[:i + 1])
        dot_a.set_data([t[i]], [alt[i]])
        dot_v.set_data([t[i]], [vel[i]])
        state = f["state_name"]
        title.set_text(f"AeroPilot   T+{f['t']:05.2f}s   "
                       f"STATE: {state}   ALT: {f['alt']:6.1f} m   "
                       f"V: {f['vel']:6.1f} m/s")
        title.set_color(STATE_COLORS.get(state, "#d7e0ea"))
        return line_a, line_v, dot_a, dot_v, title

    anim = FuncAnimation(fig, draw, frames=keyframes, blit=False,
                         interval=1000 / args.fps)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    anim.save(args.out, writer=PillowWriter(fps=args.fps))
    print(f"wrote {args.out} ({len(keyframes)} keyframes @ {args.fps} fps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
