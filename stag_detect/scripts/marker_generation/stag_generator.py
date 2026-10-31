#!/usr/bin/env python3
"""
STag (Circular Bullseye) Marker Renderer and Decoder Module.

Renders HD11 / HD23 STag markers directly from codeword definitions into
vector SVG and high-resolution bitmap images.
"""

import math
import os
import re
import numpy as np
import cv2

# STag geometry parameters
# Canonical quad space: [-0.5, 0.5] x [-0.5, 0.5] centered at (0, 0)
# Outer radius = 0.40 (of unit quad), inner radius = 0.36
OUTER_CIRCLE_R = 0.40
INNER_CIRCLE_R = 0.36
BULLSEYE_INNER_R = 0.10
BULLSEYE_OUTER_R = 0.13
HALF_PI = math.pi / 2.0

# 12 polar coordinate sample radii ratios within the 4 quadrants:
QUAD_RADII = [
    0.088363142525988,
    0.206935928182607,
    0.206935928182607,
    0.313672146827381,
    0.327493143484516,
    0.327493143484516,
    0.313672146827381,
    0.437421957035861,
    0.437226762361658,
    0.430628029742607,
    0.437226762361658,
    0.437421957035861,
]

QUAD_ANGLES = [
    0.785398163397448,
    0.459275804122858,
    HALF_PI - 0.459275804122858,
    0.200579720495241,
    0.591687617505840,
    HALF_PI - 0.591687617505840,
    HALF_PI - 0.200579720495241,
    0.145724938287167,
    0.433363129825345,
    0.785398163397448,
    HALF_PI - 0.433363129825345,
    HALF_PI - 0.145724938287167,
]

_HD11_TABLE = None

def load_hd11_table():
    """Load HD11 codeword lookup table from MarkerIDs.h."""
    global _HD11_TABLE
    if _HD11_TABLE is not None:
        return _HD11_TABLE

    header_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "include", "stag", "MarkerIDs.h"
    )
    if not os.path.exists(header_path):
        raise FileNotFoundError(f"Could not locate MarkerIDs.h at {header_path}")

    with open(header_path, "r") as f:
        content = f.read()

    match = re.search(r"HD11\[\d+\]\s*=\s*\{([^}]+)\}", content)
    if not match:
        raise ValueError("Could not parse HD11 array from MarkerIDs.h")

    raw_tokens = match.group(1).split(",")
    _HD11_TABLE = [int(tok.strip()) for tok in raw_tokens if tok.strip()]
    return _HD11_TABLE


def get_codeword_bits(marker_id: int) -> list[int]:
    """Retrieve 48-bit boolean codeword for marker ID."""
    table = load_hd11_table()
    if marker_id < 0 or marker_id >= len(table) // 4:
        raise IndexError(f"Marker ID {marker_id} out of bounds (max {len(table)//4 - 1})")
    val = table[marker_id]
    return [(val >> bit_idx) & 1 for bit_idx in range(48)]


def render_stag_image(marker_id: int, image_size: int = 1000) -> np.ndarray:
    """
    Render high-resolution raster image of an STag circular marker.
    Outer white margin, black bounding circle, circular bullseye, and 48 code cells.
    """
    img = np.ones((image_size, image_size, 3), dtype=np.uint8) * 255
    cx, cy = image_size // 2, image_size // 2
    r_outer = int(image_size * OUTER_CIRCLE_R)
    r_inner = int(image_size * INNER_CIRCLE_R)
    r_bull_out = int(image_size * BULLSEYE_OUTER_R)
    r_bull_in = int(image_size * BULLSEYE_INNER_R)

    # Outer black ring
    cv2.circle(img, (cx, cy), r_outer, (0, 0, 0), -1, cv2.LINE_AA)
    # White data ring track
    cv2.circle(img, (cx, cy), r_inner, (255, 255, 255), -1, cv2.LINE_AA)

    # Bullseye center
    cv2.circle(img, (cx, cy), r_bull_out, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r_bull_in, (255, 255, 255), -1, cv2.LINE_AA)

    # Draw the 48 code cells (black circles where bit == 0, white where bit == 1)
    bits = get_codeword_bits(marker_id)
    cell_radius = int(image_size * 0.022)

    for quad_idx in range(4):
        quad_offset = quad_idx * HALF_PI
        for cell_idx in range(12):
            bit_num = quad_idx * 12 + cell_idx
            is_white = bits[bit_num]
            r = QUAD_RADII[cell_idx] * (image_size * INNER_CIRCLE_R)
            theta = QUAD_ANGLES[cell_idx] + quad_offset
            px = int(cx + r * math.cos(theta))
            py = int(cy + r * math.sin(theta))
            color = (255, 255, 255) if is_white else (0, 0, 0)
            cv2.circle(img, (px, py), cell_radius, color, -1, cv2.LINE_AA)

    return img
