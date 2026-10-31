#!/usr/bin/env python3
"""
Generate printable PDF calibration sheets with colored warehouse navigation borders
(GO, STOP, TURN, BIDIR) for STag fiducial markers.

Ubiquity Robotics Fiducials Stack
"""

import argparse
import base64
import os
import sys
import time
from datetime import date

# Ensure local marker_generation is importable
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import cairosvg
import cv2
from marker_generation.fiducial_svg import genFiducialwithoutBorder
from marker_generation.stag_generator import render_stag_image

def check_dependencies():
    if os.system('which pdfunite > /dev/null 2>&1') != 0:
        print('Error: pdfunite is required. Install via: sudo apt install poppler-utils')
        sys.exit(1)


def generate_single_stag_pdf(marker_id: int, tag_type: str, pack_num: int, output_dir: str, fid_len: float = 184.5) -> str:
    """Renders a single STag marker and wraps it into a calibrated warehouse vector PDF."""
    # 1. Render the STag marker PNG in memory
    img = render_stag_image(marker_id, image_size=1200)
    temp_png = os.path.join(output_dir, f"temp_stag_{marker_id}_{time.time_ns()}.png")
    cv2.imwrite(temp_png, img)

    try:
        # 2. Embed into Ubiquity SVG navigation template
        svg_content = genFiducialwithoutBorder(
            upper=marker_id,
            lower=marker_id,
            tag=tag_type,
            pack_id=pack_num,
            marker_path_in=temp_png,
            fid_len=fid_len,
            year=date.today().year
        )

        # 3. Convert SVG to PDF via CairoSVG (scaled to 220mm border)
        pdf_path = os.path.join(output_dir, f"stag_marker_{marker_id}_{tag_type}.pdf")
        svg_scale = 220.0 / 180.0
        cairosvg.svg2pdf(bytestring=svg_content.encode('utf-8'), scale=svg_scale, write_to=pdf_path)
        return pdf_path
    finally:
        if os.path.exists(temp_png):
            os.remove(temp_png)


def main():
    parser = argparse.ArgumentParser(
        description="Generate printable STag fiducial markers with Ubiquity warehouse action borders."
    )
    parser.add_argument("--start", type=int, default=2000, help="First marker ID to generate (default: 2000)")
    parser.add_argument("--count", type=int, default=1, help="Number of markers to generate (default: 1)")
    parser.add_argument(
        "--type",
        choices=["GO", "STOP", "TURN", "BIDIR"],
        default="GO",
        help="Action border style: GO (Green), STOP (Red), TURN (Blue), BIDIR (Purple) [default: GO]"
    )
    parser.add_argument("--pack", type=int, default=1, help="Pack number identifier (default: 1)")
    parser.add_argument("--output", type=str, default="stag_markers.pdf", help="Output PDF file name (default: stag_markers.pdf)")
    parser.add_argument("--fid-len", type=float, default=184.5, help="Fiducial length in mm (default: 184.5)")

    args = parser.parse_args()
    check_dependencies()

    generated_pdfs = []
    temp_dir = "/tmp"

    print(f"Generating {args.count} STag '{args.type}' markers starting from ID {args.start}...")
    for marker_id in range(args.start, args.start + args.count):
        print(f"  Rendering STag ID {marker_id} [{args.type}]...", end="", flush=True)
        pdf = generate_single_stag_pdf(marker_id, args.type, args.pack, temp_dir, args.fid_len)
        generated_pdfs.append(pdf)
        print(" Done.")

    if len(generated_pdfs) == 1:
        os.replace(generated_pdfs[0], args.output)
    else:
        print(f"Combining {len(generated_pdfs)} markers into {args.output}...")
        os.system(f"pdfunite {' '.join(generated_pdfs)} {args.output}")
        for pdf in generated_pdfs:
            if os.path.exists(pdf):
                os.remove(pdf)

    print("\033[92m" + f"Successfully generated: {os.path.abspath(args.output)}" + "\033[0m")
    print("\033[93m" + "After printing, verify that the physical border dimensions match the calibration marks exactly." + "\033[0m")

if __name__ == "__main__":
    main()
