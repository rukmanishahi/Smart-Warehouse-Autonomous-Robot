"""
decision_engine.py — Decision logic: box attributes -> destination bin.

Simple, explainable rules engine (checked in priority order) plus a
barcode -> bin lookup table for destination-coded boxes. Swap this out for
a trained classifier later without touching perception or actuation code.
"""

from vision_classifier import BoxAttributes

# Barcode payload -> bin, e.g. warehouse zone codes on the label
BARCODE_BIN_MAP = {
    "ZONE-A": 1,
    "ZONE-B": 2,
    "ZONE-C": 3,
}

DEFAULT_BIN = 1
FRAGILE_BIN = 2   # e.g. red boxes
HEAVY_BIN = 3
HEAVY_THRESHOLD_G = 5000.0  # 5 kg


def classify_to_bin(attrs: BoxAttributes, load_g: float) -> int:
    """
    Priority order:
      1. Explicit destination barcode wins (most specific).
      2. Heavy boxes go to the heavy-handling bin.
      3. Fragile (red-labelled) boxes go to the fragile bin.
      4. Otherwise, default bin.
    """
    if attrs.barcode and attrs.barcode in BARCODE_BIN_MAP:
        return BARCODE_BIN_MAP[attrs.barcode]

    if load_g >= HEAVY_THRESHOLD_G:
        return HEAVY_BIN

    if attrs.color == "red":
        return FRAGILE_BIN

    return DEFAULT_BIN