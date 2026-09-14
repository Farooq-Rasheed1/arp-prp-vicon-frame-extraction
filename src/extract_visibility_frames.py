#!/usr/bin/env python3
"""Extract annotated compressedDepth frames for visibility candidates."""

from __future__ import annotations

import argparse
import csv
import io
import sqlite3
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


def topic_id(db_path: Path, topic_name: str) -> int:
    with sqlite3.connect(db_path) as con:
        row = con.execute('select id from topics where name = ?', (topic_name,)).fetchone()
    if row is None:
        raise SystemExit(f'topic not found in {db_path}: {topic_name}')
    return int(row[0])


def read_message_data(db_path: Path, topic_name: str, stamp_ns: int) -> bytes:
    tid = topic_id(db_path, topic_name)
    with sqlite3.connect(db_path) as con:
        row = con.execute(
            'select data from messages where topic_id = ? and timestamp = ?',
            (tid, stamp_ns),
        ).fetchone()
    if row is None:
        raise SystemExit(f'image timestamp not found: {stamp_ns}')
    return bytes(row[0])


def png_from_compressed_depth(serialized: bytes) -> bytes:
    start = serialized.find(PNG_SIGNATURE)
    if start < 0:
        raise ValueError('compressedDepth payload did not contain a PNG signature')
    return serialized[start:]


def colorize_depth(
    png_bytes: bytes,
    min_depth_m: float = 0.35,
    max_depth_m: float = 6.0,
) -> Image.Image:
    """Use one metric color scale for every frame: warm=near, cool=far."""
    depth_mm = np.asarray(Image.open(io.BytesIO(png_bytes)), dtype=np.uint16)
    depth_m = depth_mm.astype(np.float32) * 0.001
    normalized = np.clip((depth_m - min_depth_m) / (max_depth_m - min_depth_m), 0.0, 1.0)
    color_input = np.uint8(np.rint((1.0 - normalized) * 255.0))
    colored_bgr = cv2.applyColorMap(color_input, cv2.COLORMAP_TURBO)
    colored_bgr[depth_mm == 0] = 0
    return Image.fromarray(cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB))


def visible_segments(rows: list[dict[str, str]], max_gap_s: float = 0.25) -> list[list[dict[str, str]]]:
    segments: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    last_stamp: int | None = None
    max_gap_ns = int(max_gap_s * 1e9)
    for row in rows:
        stamp = int(row['stamp_ns'])
        if row['visible'] != '1':
            if current:
                segments.append(current)
                current = []
            last_stamp = None
            continue
        if last_stamp is not None and stamp - last_stamp > max_gap_ns:
            segments.append(current)
            current = []
        current.append(row)
        last_stamp = stamp
    if current:
        segments.append(current)
    return segments


def sample_rows(csv_path: Path, count: int, extract_all: bool = False) -> list[dict[str, str]]:
    with csv_path.open() as handle:
        rows = list(csv.DictReader(handle))

    if extract_all:
        return [row for row in rows if row['visible'] == '1']

    segments = sorted(visible_segments(rows), key=len, reverse=True)
    samples = []
    for segment in segments[:count]:
        samples.append(segment[len(segment) // 2])
    return samples


def bbox_coordinates(
    image: Image.Image,
    row: dict[str, str],
    center_v_fraction: float,
) -> tuple[float, float, float, float]:
    """Return the clipped projected box in image pixel coordinates."""
    u = float(row['u_px'])
    center_v = image.height * center_v_fraction
    box_w = float(row['bbox_w_px'])
    box_h = float(row['bbox_h_px'])
    return (
        max(0.0, u - box_w / 2.0),
        max(0.0, center_v - box_h / 2.0),
        min(float(image.width - 1), u + box_w / 2.0),
        min(float(image.height - 1), center_v + box_h / 2.0),
    )


def annotate(
    image: Image.Image,
    row: dict[str, str],
    label: str,
    center_v_fraction: float,
) -> tuple[Image.Image, tuple[float, float, float, float]]:
    annotated = image.copy()
    draw = ImageDraw.Draw(annotated)
    left, top, right, bottom = bbox_coordinates(annotated, row, center_v_fraction)
    color = (255, 255, 255)
    draw.rectangle([(left, top), (right, bottom)], outline=(0, 0, 0), width=7)
    draw.rectangle([(left, top), (right, bottom)], outline=color, width=3)
    text = (
        f'{label} t={float(row["time_s_from_start"]):.1f}s '
        f'r={float(row["range_m"]):.2f}m '
        f'b={float(row["bearing_deg"]):.1f}deg '
        f'box=({left:.0f},{top:.0f})-({right:.0f},{bottom:.0f})'
    )
    margin = 12
    text_box = draw.textbbox((margin, margin), text)
    draw.rectangle(
        [(text_box[0] - 6, text_box[1] - 4), (text_box[2] + 6, text_box[3] + 4)],
        fill=(0, 0, 0),
    )
    draw.text((margin, margin), text, fill=(255, 255, 255))
    return annotated, (left, top, right, bottom)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--db', required=True)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--csv', required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--all', action='store_true', help='Extract all visible frames instead of sampling')
    parser.add_argument(
        '--center-v-fraction', type=float, required=True,
        help='Camera principal-point cy divided by the source calibration height',
    )
    args = parser.parse_args()

    db_path = Path(args.db)
    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    extracted_rows = []
    for index, row in enumerate(sample_rows(csv_path, args.count, args.all), 1):
        stamp = int(row['stamp_ns'])
        data = read_message_data(db_path, args.topic, stamp)
        png_bytes = png_from_compressed_depth(data)
        raw_path = output_dir / f'{args.label}_{index:02d}_t{stamp}_raw.png'
        annotated_path = output_dir / f'{args.label}_{index:02d}_t{stamp}_annotated.png'
        raw_path.write_bytes(png_bytes)
        preview = colorize_depth(png_bytes)
        annotated, box = annotate(preview, row, args.label, args.center_v_fraction)
        annotated.save(annotated_path)
        extracted_rows.append({
            **row,
            'bbox_x0_px': f'{box[0]:.2f}',
            'bbox_y0_px': f'{box[1]:.2f}',
            'bbox_x1_px': f'{box[2]:.2f}',
            'bbox_y1_px': f'{box[3]:.2f}',
            'raw_file': raw_path.name,
            'annotated_file': annotated_path.name,
        })
        print(annotated_path)

    if extracted_rows:
        with (output_dir / 'extracted_frames.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(extracted_rows[0]))
            writer.writeheader()
            writer.writerows(extracted_rows)


if __name__ == '__main__':
    main()
