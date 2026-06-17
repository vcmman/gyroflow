#!/usr/bin/env python3
"""Build a .gyroflow project for an apples-to-apples C++ vs Gyroflow comparison.

Gyroflow can't render the original 10-bit DJI clip in some environments (no GPU encoder;
the software encoder rejects YUV420P10LE). The fix is to transcode the source to 8-bit once
and have BOTH tools read that identical file — so the comparison isolates the stabilization
math, not the decoder. But Gyroflow parses telemetry from the *video* it renders, and an
8-bit transcode has no DJI metadata, so we keep `gyro_source.filepath` pointing at the
original while `videofile` points at the transcode.

This script clones an existing project and rewires exactly those fields (and optionally the
interpolation, so it can be forced to Bilinear to match the C++ kernel).

Usage:
    python3 tools/make_comparison_project.py \
        --project   data/MyClip.gyroflow \
        --transcode /tmp/src8.mp4 \
        --original  data/MyClip.MP4 \
        --out       /tmp/proj_cmp.gyroflow \
        --output    /tmp/gf_cmp.mp4 \
        --interpolation Bilinear
"""
import argparse
import json
import os


def file_url(path):
    return "file://" + os.path.abspath(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--project", required=True, help="existing .gyroflow project to clone")
    ap.add_argument("--transcode", required=True, help="8-bit source video to render")
    ap.add_argument("--original", required=True, help="original DJI video (telemetry source)")
    ap.add_argument("--out", required=True, help="output .gyroflow project path")
    ap.add_argument("--output", required=True, help="render target the project should write")
    ap.add_argument("--interpolation", default=None,
                    help="override interpolation, e.g. Bilinear|Lanczos4 (default: keep project's)")
    a = ap.parse_args()

    d = json.load(open(a.project))
    d["videofile"] = file_url(a.transcode)
    d.setdefault("gyro_source", {})["filepath"] = file_url(a.original)

    o = d.setdefault("output", {})
    o["input_filename"] = os.path.basename(a.transcode)
    o["input_url"] = file_url(a.transcode)
    o["output_folder"] = file_url(os.path.dirname(a.output) or ".") + "/"
    o["output_filename"] = os.path.basename(a.output)
    if a.interpolation:
        o["interpolation"] = a.interpolation

    json.dump(d, open(a.out, "w"))
    print(f"wrote {a.out}")
    print(f"  render source (8-bit): {d['videofile']}")
    print(f"  telemetry source     : {d['gyro_source']['filepath']}")
    print(f"  render target        : {o['output_folder']}{o['output_filename']}")
    print(f"  interpolation        : {o.get('interpolation', '(project default)')}")


if __name__ == "__main__":
    main()
