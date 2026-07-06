#!/usr/bin/env python3
"""AeroPilot ground station.

Decodes the telemetry byte stream produced by the flight computer over the
simulated ARINC-429 avionics bus, validates each layer (word parity/label,
frame sync + CRC-16/CCITT), reconstructs the flight, plots altitude / vertical
velocity / flight state, and reports apogee-detection accuracy.

Usage:
    python groundstation/decode.py --file /tmp/telemetry.bin [--plot out.png]
                                   [--true-apogee 106.6] [--check]
"""
import argparse
import json
import struct
import sys

# --- Flight state names (must match src/aeropilot.c) ---
STATE_NAMES = ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
               "DESCENT", "LANDED", "SAFE"]

# --- Frame format (must match src/proto/frame.h) ---
SYNC = 0xA55A
FRAME_SIZE = 18
FLAG_DEPLOYED = 1 << 0
FLAG_SAFE = 1 << 1

# --- ARINC-429 (must match src/proto/arinc429.c) ---
LABEL_TELEMETRY = 0o311


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def reverse8(v: int) -> int:
    r = 0
    for i in range(8):
        if v & (1 << i):
            r |= 1 << (7 - i)
    return r


def arinc_parity_ok(word: int) -> bool:
    return bin(word & 0xFFFFFFFF).count("1") % 2 == 1


def arinc_decode_payload(word: int):
    """Return the 16-bit payload of a telemetry word, or None if invalid."""
    if not arinc_parity_ok(word):
        return None
    if reverse8(word & 0xFF) != LABEL_TELEMETRY:
        return None
    return (word >> 10) & 0xFFFF


class Stats:
    def __init__(self):
        self.words_total = 0
        self.words_bad = 0
        self.frames_ok = 0
        self.frames_bad = 0


def unpack_bus_stream(raw: bytes, stats: Stats) -> bytes:
    """Decode ARINC-429 wire words (4 LE bytes each) back into frame bytes."""
    out = bytearray()
    n = len(raw) - (len(raw) % 4)
    for i in range(0, n, 4):
        word = struct.unpack_from("<I", raw, i)[0]
        stats.words_total += 1
        payload = arinc_decode_payload(word)
        if payload is None:
            stats.words_bad += 1
            continue
        out.append(payload & 0xFF)
        out.append((payload >> 8) & 0xFF)
    return bytes(out)


def decode_frame(buf: bytes):
    """Decode a FRAME_SIZE buffer. Returns a dict or None on CRC/sync error."""
    if len(buf) < FRAME_SIZE:
        return None
    sync = struct.unpack_from("<H", buf, 0)[0]
    if sync != SYNC:
        return None
    if crc16_ccitt(buf[0:16]) != struct.unpack_from("<H", buf, 16)[0]:
        return None
    seq, state, flags = struct.unpack_from("<HBB", buf, 2)
    alt, vel = struct.unpack_from("<ff", buf, 6)
    batt = struct.unpack_from("<H", buf, 14)[0]
    return {
        "seq": seq,
        "state": state,
        "flags": flags,
        "alt": alt,
        "vel": vel,
        "batt_mv": batt,
    }


def parse_frames(stream: bytes, stats: Stats):
    """Resynchronise on the sync word and decode all valid frames."""
    frames = []
    i = 0
    n = len(stream)
    while i + FRAME_SIZE <= n:
        if struct.unpack_from("<H", stream, i)[0] != SYNC:
            i += 1  # hunt for the next sync word
            continue
        frame = decode_frame(stream[i:i + FRAME_SIZE])
        if frame is None:
            stats.frames_bad += 1
            i += 1
            continue
        frames.append(frame)
        stats.frames_ok += 1
        i += FRAME_SIZE
    return frames


def plot_flight(frames, out_path, telemetry_period_s):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [f["seq"] * telemetry_period_s for f in frames]
    alt = [f["alt"] for f in frames]
    vel = [f["vel"] for f in frames]
    state = [f["state"] for f in frames]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax1.plot(t, alt, color="tab:blue")
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True)
    ax1.set_title("AeroPilot telemetry")

    ax2.plot(t, vel, color="tab:green")
    ax2.set_ylabel("Vertical velocity (m/s)")
    ax2.grid(True)

    ax3.step(t, state, where="post", color="tab:red")
    ax3.set_yticks(range(len(STATE_NAMES)))
    ax3.set_yticklabels(STATE_NAMES)
    ax3.set_ylabel("Flight state")
    ax3.set_xlabel("Time (s)")
    ax3.grid(True)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print(f"[gs] wrote plot to {out_path}")


def write_json(path, frames, stats, period, apogee, apogee_frame, deployed,
               safe, states_seen, true_apogee, label):
    """Serialise the decoded flight to a compact JSON for the web dashboard."""
    out_frames = []
    for f in frames:
        out_frames.append({
            "seq": f["seq"],
            "t": round(f["seq"] * period, 4),
            "state": f["state"],
            "state_name": STATE_NAMES[f["state"]]
            if f["state"] < len(STATE_NAMES) else "UNKNOWN",
            "alt": round(f["alt"], 3),
            "vel": round(f["vel"], 3),
            "batt_mv": f["batt_mv"],
            "deployed": bool(f["flags"] & FLAG_DEPLOYED),
            "safe": bool(f["flags"] & FLAG_SAFE),
        })

    deploy_frame = next((f for f in out_frames if f["deployed"]), None)
    meta = {
        "label": label,
        "telemetry_period_s": period,
        "frames_ok": stats.frames_ok,
        "frames_bad": stats.frames_bad,
        "words_total": stats.words_total,
        "words_bad": stats.words_bad,
        "observed_apogee_m": round(apogee, 2),
        "apogee_seq": apogee_frame["seq"],
        "apogee_t": round(apogee_frame["seq"] * period, 3),
        "true_apogee_m": true_apogee,
        "apogee_error_m": round(apogee - true_apogee, 2)
        if true_apogee is not None else None,
        "deployed": deployed,
        "deploy_t": deploy_frame["t"] if deploy_frame else None,
        "safe": safe,
        "state_sequence": [STATE_NAMES[s] for s in states_seen],
        "duration_s": round(out_frames[-1]["t"], 2) if out_frames else 0.0,
    }

    with open(path, "w") as fh:
        json.dump({"meta": meta, "frames": out_frames}, fh)


def main(argv=None):
    ap = argparse.ArgumentParser(description="AeroPilot telemetry ground station")
    ap.add_argument("--file", required=True, help="telemetry byte stream file")
    ap.add_argument("--plot", help="output PNG path for the flight plot")
    ap.add_argument("--true-apogee", type=float, default=None,
                    help="true apogee altitude (m) for accuracy reporting")
    ap.add_argument("--telemetry-period", type=float, default=0.05,
                    help="telemetry frame period in seconds (for time axis)")
    ap.add_argument("--json", dest="json_out",
                    help="write decoded telemetry + summary as JSON")
    ap.add_argument("--label", default="",
                    help="human label stored in the JSON metadata")
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero unless a nominal flight is decoded")
    args = ap.parse_args(argv)

    with open(args.file, "rb") as fh:
        raw = fh.read()

    stats = Stats()
    stream = unpack_bus_stream(raw, stats)
    frames = parse_frames(stream, stats)

    print(f"[gs] wire bytes:      {len(raw)}")
    print(f"[gs] ARINC words:     {stats.words_total} "
          f"({stats.words_bad} rejected on parity/label)")
    print(f"[gs] frames decoded:  {stats.frames_ok} "
          f"({stats.frames_bad} rejected on sync/CRC)")

    if not frames:
        print("[gs] ERROR: no valid telemetry frames decoded", file=sys.stderr)
        return 1

    apogee = max(f["alt"] for f in frames)
    apogee_frame = max(frames, key=lambda f: f["alt"])
    deployed = any(f["flags"] & FLAG_DEPLOYED for f in frames)
    safe = any(f["flags"] & FLAG_SAFE for f in frames)
    states_seen = []
    for f in frames:
        if not states_seen or states_seen[-1] != f["state"]:
            states_seen.append(f["state"])
    state_seq = " -> ".join(STATE_NAMES[s] for s in states_seen)

    print(f"[gs] state sequence:  {state_seq}")
    print(f"[gs] observed apogee: {apogee:.1f} m "
          f"(seq {apogee_frame['seq']})")
    print(f"[gs] deploy flag:     {'YES' if deployed else 'no'}")
    print(f"[gs] safe flag:       {'YES' if safe else 'no'}")
    if args.true_apogee is not None:
        err = apogee - args.true_apogee
        print(f"[gs] apogee error:    {err:+.1f} m vs true "
              f"{args.true_apogee:.1f} m")

    if args.json_out:
        write_json(args.json_out, frames, stats, args.telemetry_period,
                   apogee, apogee_frame, deployed, safe, states_seen,
                   args.true_apogee, args.label)
        print(f"[gs] wrote JSON to {args.json_out}")

    if args.plot:
        plot_flight(frames, args.plot, args.telemetry_period)

    if args.check:
        ok = (stats.frames_ok > 10 and deployed and
              STATE_NAMES.index("LANDED") in states_seen)
        if not ok:
            print("[gs] CHECK FAILED: expected >10 frames, deploy, and LANDED",
                  file=sys.stderr)
            return 2
        print("[gs] CHECK PASSED")

    return 0


if __name__ == "__main__":
    sys.exit(main())
