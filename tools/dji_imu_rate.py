#!/usr/bin/env python3
"""Estimate DJI IMU/attitude sample frequency directly from the MP4 metadata track.

Parses the 'djmd' DJI metadata track, walks the protobuf to the repeated
quaternion attitude array (4x float32 per sample), counts samples per frame,
and divides total samples by clip duration to get the effective IMU rate.
"""
import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from inspect_dji_meta import find_box, read_boxes, parse_trak, sample_offsets  # noqa: E402


def read_varint(b, i):
    shift = result = 0
    while True:
        byte = b[i]; i += 1
        result |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            break
        shift += 7
    return result, i


def iter_fields(b):
    """Yield (field_number, wire_type, value_bytes_or_int)."""
    i, n = 0, len(b)
    while i < n:
        tag, i = read_varint(b, i)
        fnum, wt = tag >> 3, tag & 7
        if wt == 0:
            v, i = read_varint(b, i); yield fnum, wt, v
        elif wt == 1:
            yield fnum, wt, b[i:i+8]; i += 8
        elif wt == 5:
            yield fnum, wt, b[i:i+4]; i += 4
        elif wt == 2:
            ln, i = read_varint(b, i); yield fnum, wt, b[i:i+ln]; i += ln
        else:
            raise ValueError(f"bad wire type {wt}")


def get_submsgs(b, fnum):
    return [v for f, wt, v in iter_fields(b) if f == fnum and wt == 2]


def find_attitude_arrays(blob):
    """Return list of quaternion-arrays found anywhere in the message tree.

    A quaternion is a sub-message with exactly fields 1..4 each float32 (wt5).
    An attitude array is a parent field repeated >=2 times whose children match.
    """
    results = []

    def is_quat(sub):
        try:
            fs = list(iter_fields(sub))
        except Exception:
            return None
        if len(fs) != 4:
            return None
        vals = []
        for fnum, wt, v in fs:
            if wt != 5 or fnum not in (1, 2, 3, 4):
                return None
            vals.append(struct.unpack("<f", v)[0])
        return vals

    def walk(b):
        # group repeated length-delimited fields by field number
        byfield = {}
        for fnum, wt, v in iter_fields(b):
            if wt == 2:
                byfield.setdefault(fnum, []).append(v)
        for fnum, subs in byfield.items():
            if len(subs) >= 2:
                quats = [is_quat(s) for s in subs]
                if all(q is not None for q in quats):
                    results.append(quats)
                    continue
            # recurse into submessages (try; ignore leaf bytes)
            for s in subs:
                try:
                    list(iter_fields(s))
                except Exception:
                    continue
                walk(s)

    walk(blob)
    return results


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mp4", type=Path, help="Input DJI .mp4/.mov file")
    return parser.parse_args()


def main():
    args = parse_args()
    path = args.mp4
    with path.open("rb") as f:
        fsize = path.stat().st_size
        mp, me = find_box(f, 0, fsize, ["moov"])
        traks = [parse_trak(f, p, end) for bt, _, p, end in read_boxes(f, mp, me) if bt == "trak"]
        djmd = next((t for t in traks if t.get("format") == "djmd"), None)
        if not djmd:
            print("no djmd track"); return
        ts = djmd["timescale"]
        dur_s = djmd["duration"] / ts
        offs = sample_offsets(djmd)
        n = len(offs)
        print(f"djmd track: {n} samples, duration {dur_s:.3f} s, "
              f"frame rate {n/dur_s:.2f} fps")

        per_frame_counts = []
        norms_ok = 0
        norms_tot = 0
        sample_quat = None
        for k, (o, sz) in enumerate(offs):
            f.seek(o); blob = f.read(sz)
            arrays = find_attitude_arrays(blob)
            # pick the largest array as the attitude/IMU array
            if arrays:
                arr = max(arrays, key=len)
                per_frame_counts.append(len(arr))
                if sample_quat is None:
                    sample_quat = arr[0]
                for q in arr:
                    norms_tot += 1
                    nrm = (q[0]**2+q[1]**2+q[2]**2+q[3]**2) ** 0.5
                    if 0.9 < nrm < 1.1:
                        norms_ok += 1
            else:
                per_frame_counts.append(0)

        total = sum(per_frame_counts)
        from collections import Counter
        dist = Counter(per_frame_counts)
        print(f"attitude/IMU samples per frame: {dict(sorted(dist.items()))}")
        print(f"total attitude/IMU samples: {total}")
        print(f"quaternion norm in [0.9,1.1]: {norms_ok}/{norms_tot}")
        if sample_quat:
            print(f"example quaternion (w,x,y,z): "
                  f"{', '.join(f'{c:.4f}' for c in sample_quat)}")
        if dur_s:
            print()
            print(f">>> IMU/attitude frequency = {total} / {dur_s:.3f} s "
                  f"= {total/dur_s:.1f} Hz")
            mode = dist.most_common(1)[0][0]
            print(f">>> per-frame mode {mode} x {n/dur_s:.2f} fps "
                  f"= {mode * n/dur_s:.1f} Hz")


if __name__ == "__main__":
    main()
