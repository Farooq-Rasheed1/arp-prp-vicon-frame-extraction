#!/usr/bin/env python3
"""Create compact README visualizations from annotated visible-depth frames."""

from __future__ import annotations

import argparse
import re
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


STAMP_RE = re.compile(r'_t(\d+)_')


def frame_stamp(path: Path) -> int:
    match = STAMP_RE.search(path.name)
    if match is None:
        raise ValueError(f'cannot parse timestamp from {path.name}')
    return int(match.group(1))


def annotated_frames(folder: Path) -> list[Path]:
    frames = sorted(folder.glob('*_annotated.png'), key=frame_stamp)
    if not frames:
        raise SystemExit(f'no annotated frames found in {folder}')
    return frames


def sample_evenly(items: list[Path], count: int) -> list[Path]:
    if len(items) <= count:
        return items
    return [items[round(index * (len(items) - 1) / (count - 1))] for index in range(count)]


def prepare_frame(path: Path, width: int, title: str, index: int, total: int) -> Image.Image:
    image = Image.open(path).convert('RGB')
    if image.width != width:
        height = round(image.height * width / image.width)
        image = image.resize((width, height), Image.Resampling.LANCZOS)
    overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    font = ImageFont.load_default()
    text = f'{title} | visible frame {index}/{total}'
    box = draw.textbbox((0, 0), text, font=font)
    draw.rectangle((8, 8, box[2] + 18, box[3] + 18), fill=(0, 0, 0, 180))
    draw.text((13, 13), text, fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(image.convert('RGBA'), overlay).convert('RGB')


def write_gif(frames: list[Path], output: Path, title: str, width: int, fps: int, max_frames: int) -> None:
    selected = sample_evenly(frames, max_frames)
    images = [prepare_frame(path, width, title, index + 1, len(frames)) for index, path in enumerate(selected)]
    output.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=round(1000 / fps),
        loop=0,
        optimize=True,
    )


def write_mp4(frames: list[Path], output: Path, title: str, width: int, fps: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        for index, path in enumerate(frames):
            prepare_frame(path, width, title, index + 1, len(frames)).save(tmp_path / f'frame_{index:05d}.png')
        subprocess.run(
            [
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                '-framerate', str(fps), '-i', str(tmp_path / 'frame_%05d.png'),
                '-vf', 'format=yuv420p', '-movflags', '+faststart',
                '-c:v', 'libx264', '-crf', '24', str(output),
            ],
            check=True,
        )


def build(folder: Path, output_stem: Path, title: str, width: int, fps: int, gif_frames: int) -> None:
    frames = annotated_frames(folder)
    write_gif(frames, output_stem.with_suffix('.gif'), title, width, fps, gif_frames)
    write_mp4(frames, output_stem.with_suffix('.mp4'), title, width, fps)
    print(f'{title}: {len(frames)} input frames')
    print(output_stem.with_suffix('.gif'))
    print(output_stem.with_suffix('.mp4'))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--tb1-dir', required=True, type=Path)
    parser.add_argument('--tb2-dir', required=True, type=Path)
    parser.add_argument('--output-dir', default=Path('docs/media'), type=Path)
    parser.add_argument('--width', default=480, type=int)
    parser.add_argument('--fps', default=12, type=int)
    parser.add_argument('--gif-frames', default=90, type=int)
    args = parser.parse_args()

    build(args.tb1_dir, args.output_dir / 'tb1_camera_sees_tb2_depth_boxes',
          'TB1 camera sees TB2', args.width, args.fps, args.gif_frames)
    build(args.tb2_dir, args.output_dir / 'tb2_camera_sees_tb1_depth_boxes',
          'TB2 camera sees TB1', args.width, args.fps, args.gif_frames)


if __name__ == '__main__':
    main()
