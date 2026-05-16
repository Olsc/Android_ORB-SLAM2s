# Generation Tools

This directory contains utility scripts used for project maintenance and optimization.

## ORB LUT Generator (`generate_orb_lut.py`)

This script pre-calculates the rotation lookup table (LUT) for ORB descriptors. 

*   **Purpose**: Normally, rotating descriptors to achieve rotation invariance is computationally expensive. By pre-calculating the bit offsets for all 360 degrees, we can perform the rotation with a simple table lookup.
*   **Output**: Generates a binary file or header data that is embedded into the C++ core (`ORBextractor.cc`).

## Aesthetic Flowchart Generator (`generate_aesthetic_flowchart.py`)

A specialized tool for creating the professional-looking SVGs used in this project's documentation.

*   **Features**:
    *   Gradient fills and modern color palettes.
    *   Smooth rounded corners and subtle shadows.
    *   Responsive layout for complex logic flows.
*   **Usage**: Run the script to update `docs/aesthetic_visual_causal_flow_en.svg` and `docs/aesthetic_visual_causal_flow_zh.svg`.
