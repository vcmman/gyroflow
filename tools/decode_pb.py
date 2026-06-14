#!/usr/bin/env python3
"""Walk raw protobuf wire format (no schema) and summarize structure."""
import argparse
import struct
from collections import Counter
from pathlib import Path


def read_varint(b, i):
    shift = 0
    result = 0
    while True:
        byte = b[i]
        i += 1
        result |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            break
        shift += 7
    return result, i


def looks_like_message(b):
    """Heuristic: can we parse all of b as protobuf fields without error?"""
    i = 0
    n = len(b)
    if n == 0:
        return False
    fields = 0
    try:
        while i < n:
            tag, i = read_varint(b, i)
            wt = tag & 7
            if wt == 0:
                _, i = read_varint(b, i)
            elif wt == 1:
                i += 8
            elif wt == 2:
                ln, i = read_varint(b, i)
                i += ln
            elif wt == 5:
                i += 4
            else:
                return False
            fields += 1
        return i == n and fields > 0
    except (IndexError, Exception):
        return False


def walk(b, depth=0, max_depth=6, path=""):
    i = 0
    n = len(b)
    out = []
    field_counter = Counter()
    while i < n:
        try:
            tag, i = read_varint(b, i)
        except IndexError:
            break
        fnum = tag >> 3
        wt = tag & 7
        field_counter[(fnum, wt)] += 1
        if wt == 0:
            val, i = read_varint(b, i)
            out.append((fnum, "varint", val, None))
        elif wt == 1:
            raw = b[i:i + 8]
            i += 8
            dval = struct.unpack("<d", raw)[0] if len(raw) == 8 else None
            out.append((fnum, "f64", dval, None))
        elif wt == 5:
            raw = b[i:i + 4]
            i += 4
            fval = struct.unpack("<f", raw)[0] if len(raw) == 4 else None
            out.append((fnum, "f32", fval, None))
        elif wt == 2:
            ln, i = read_varint(b, i)
            sub = b[i:i + ln]
            i += ln
            if depth < max_depth and looks_like_message(sub):
                out.append((fnum, "msg", len(sub), sub))
            else:
                out.append((fnum, "bytes", len(sub), sub))
        else:
            break
    return out, field_counter


def summarize(b, depth=0, path="root", max_print_depth=4):
    out, counter = walk(b, depth)
    indent = "  " * depth
    # print field summary
    parts = []
    for (fnum, wt), c in sorted(counter.items()):
        parts.append(f"f{fnum}(wt{wt})x{c}")
    print(f"{indent}{path}: {len(b)}B  fields: {', '.join(parts)}")
    if depth >= max_print_depth:
        return
    # recurse into submessages, but group repeated ones
    seen_msg_fields = {}
    for fnum, kind, meta, sub in out:
        if kind == "msg":
            seen_msg_fields.setdefault(fnum, []).append(sub)
    for fnum, subs in seen_msg_fields.items():
        print(f"{indent}  -> field {fnum}: {len(subs)} sub-message(s)")
        # only descend into the first one to show structure
        summarize(subs[0], depth + 1, f"f{fnum}[0]", max_print_depth)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("protobuf", type=Path, help="Raw protobuf payload to inspect")
    parser.add_argument("--max-print-depth", type=int, default=4, help="Maximum nested message depth to print")
    return parser.parse_args()


def main():
    args = parse_args()
    data = args.protobuf.read_bytes()
    print(f"=== {args.protobuf} ({len(data)} bytes) ===")
    summarize(data, max_print_depth=args.max_print_depth)


if __name__ == "__main__":
    main()
