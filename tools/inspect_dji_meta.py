#!/usr/bin/env python3
"""Minimal MP4 parser to inspect DJI metadata tracks and estimate sample rates."""
import argparse
import struct
from pathlib import Path


def read_boxes(f, start, end):
    """Yield (type, box_start, payload_start, box_end) for boxes in [start, end)."""
    pos = start
    while pos < end:
        f.seek(pos)
        hdr = f.read(8)
        if len(hdr) < 8:
            break
        size, btype = struct.unpack(">I4s", hdr)
        payload = pos + 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]
            payload = pos + 16
        elif size == 0:
            size = end - pos
        yield btype.decode("latin1"), pos, payload, pos + size
        pos += size


def find_box(f, start, end, path):
    """Descend a list of box types, return (payload_start, box_end) of last."""
    cur_start, cur_end = start, end
    result = None
    for want in path:
        found = None
        for btype, _, p, be in read_boxes(f, cur_start, cur_end):
            if btype == want:
                found = (p, be)
                break
        if not found:
            return None
        result = found
        cur_start, cur_end = found
    return result


def parse_trak(f, p, end):
    info = {}
    # mdia/hdlr -> handler type + name
    hdlr = find_box(f, p, end, ["mdia", "hdlr"])
    if hdlr:
        hp, he = hdlr
        f.seek(hp)
        data = f.read(he - hp)
        # version(1)+flags(3)+predefined(4)+handler(4)+reserved(12)+name(...)
        handler = data[8:12].decode("latin1", "replace")
        name = data[24:].split(b"\x00")[0].decode("latin1", "replace")
        info["handler"] = handler
        info["name"] = name
    # mdia/mdhd -> timescale + duration
    mdhd = find_box(f, p, end, ["mdia", "mdhd"])
    if mdhd:
        mp, me = mdhd
        f.seek(mp)
        d = f.read(me - mp)
        ver = d[0]
        if ver == 1:
            timescale = struct.unpack(">I", d[20:24])[0]
            duration = struct.unpack(">Q", d[24:32])[0]
        else:
            timescale = struct.unpack(">I", d[12:16])[0]
            duration = struct.unpack(">I", d[16:20])[0]
        info["timescale"] = timescale
        info["duration"] = duration
    # stbl
    stbl = find_box(f, p, end, ["mdia", "minf", "stbl"])
    if stbl:
        sp, se = stbl
        # stsd -> sample format (codec/meta format)
        stsd = find_box(f, sp, se, ["stsd"])
        if stsd:
            dp, de = stsd
            f.seek(dp)
            d = f.read(de - dp)
            # version+flags(4)+count(4)+entry: size(4)+format(4)
            if len(d) >= 16:
                info["format"] = d[12:16].decode("latin1", "replace")
        # stsz -> sample count + sizes
        stsz = find_box(f, sp, se, ["stsz"])
        if stsz:
            zp, ze = stsz
            f.seek(zp)
            d = f.read(ze - zp)
            sample_size = struct.unpack(">I", d[4:8])[0]
            count = struct.unpack(">I", d[8:12])[0]
            info["sample_count"] = count
            sizes = []
            if sample_size == 0:
                for i in range(count):
                    sizes.append(struct.unpack(">I", d[12 + i * 4:16 + i * 4])[0])
            else:
                sizes = [sample_size] * count
            info["sizes"] = sizes
        # stts -> timing
        stts = find_box(f, sp, se, ["stts"])
        if stts:
            tp, te = stts
            f.seek(tp)
            d = f.read(te - tp)
            n = struct.unpack(">I", d[4:8])[0]
            entries = []
            for i in range(n):
                cnt, delta = struct.unpack(">II", d[8 + i * 8:16 + i * 8])
                entries.append((cnt, delta))
            info["stts"] = entries
        # chunk offsets stco/co64 and stsc
        for btype in ("stco", "co64"):
            b = find_box(f, sp, se, [btype])
            if b:
                cp, ce = b
                f.seek(cp)
                d = f.read(ce - cp)
                n = struct.unpack(">I", d[4:8])[0]
                offs = []
                if btype == "stco":
                    for i in range(n):
                        offs.append(struct.unpack(">I", d[8 + i * 4:12 + i * 4])[0])
                else:
                    for i in range(n):
                        offs.append(struct.unpack(">Q", d[8 + i * 8:16 + i * 8])[0])
                info["chunk_offsets"] = offs
        stsc = find_box(f, sp, se, ["stsc"])
        if stsc:
            cp, ce = stsc
            f.seek(cp)
            d = f.read(ce - cp)
            n = struct.unpack(">I", d[4:8])[0]
            entries = []
            for i in range(n):
                first, spc, didx = struct.unpack(">III", d[8 + i * 12:20 + i * 12])
                entries.append((first, spc, didx))
            info["stsc"] = entries
    return info


def sample_offsets(info):
    """Compute absolute file offset + size for each sample."""
    sizes = info.get("sizes", [])
    chunks = info.get("chunk_offsets", [])
    stsc = info.get("stsc", [])
    if not chunks or not stsc:
        return []
    # expand stsc to samples-per-chunk per chunk
    result = []
    sample_idx = 0
    n_chunks = len(chunks)
    # build list of (chunk_index, samples_in_chunk)
    spc_list = []
    for i, (first, spc, _didx) in enumerate(stsc):
        last = stsc[i + 1][0] - 1 if i + 1 < len(stsc) else n_chunks
        for c in range(first, last + 1):
            spc_list.append((c - 1, spc))
    for chunk_index, spc in spc_list:
        if chunk_index >= n_chunks:
            break
        off = chunks[chunk_index]
        for _ in range(spc):
            if sample_idx >= len(sizes):
                break
            sz = sizes[sample_idx]
            result.append((off, sz))
            off += sz
            sample_idx += 1
    return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mp4", type=Path, help="Input DJI .mp4/.mov file")
    parser.add_argument(
        "--dump-first-samples",
        action="store_true",
        help="Write the first djmd/dbgi sample payloads next to the input file for protobuf inspection.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    path = args.mp4
    with path.open("rb") as f:
        fsize = path.stat().st_size
        moov = find_box(f, 0, fsize, ["moov"])
        if not moov:
            print("no moov")
            return
        mp, me = moov
        traks = []
        for btype, _, p, end in read_boxes(f, mp, me):
            if btype == "trak":
                traks.append(parse_trak(f, p, end))
        print(f"file: {path.name}  size: {fsize/1e6:.1f} MB")
        print("=" * 70)
        for i, t in enumerate(traks, 1):
            ts = t.get("timescale", 0)
            dur = t.get("duration", 0)
            dur_s = dur / ts if ts else 0
            cnt = t.get("sample_count", 0)
            rate = cnt / dur_s if dur_s else 0
            print(f"Track {i}: handler={t.get('handler')!r} name={t.get('name')!r} "
                  f"format={t.get('format')!r}")
            print(f"   timescale={ts} duration={dur} ({dur_s:.3f}s) "
                  f"samples={cnt} -> {rate:.2f} samples/s")
            # report stored sample object so analyzer can use it
            t["_dur_s"] = dur_s
        # Save metadata-track sample byte offsets for the DJI meta + dbgi tracks.
        for i, t in enumerate(traks, 1):
            fmt = t.get("format")
            if fmt in ("djmd", "dbgi", "dvtm") or t.get("handler") == "meta" or "DJI" in (t.get("name") or ""):
                offs = sample_offsets(t)
                t["_offsets"] = offs
        if args.dump_first_samples:
            for i, t in enumerate(traks, 1):
                if t.get("format") in ("djmd", "dbgi"):
                    offs = t.get("_offsets", [])
                    if offs:
                        o, s = offs[0]
                        f.seek(o)
                        blob = f.read(s)
                        outp = path.with_name(f"sample_track{i}_{t['format']}.bin")
                        outp.write_bytes(blob)
                        print(f"   wrote first sample ({s} bytes) of track{i} {t['format']} -> {outp.name}")


if __name__ == "__main__":
    main()
