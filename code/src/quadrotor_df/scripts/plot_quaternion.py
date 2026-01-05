#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
from typing import List, Tuple


def load_csv(path: Path):
    t_vals, qx, qy, qz, qw = [], [], [], [], []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_vals.append(float(row["t"]))
            qx.append(float(row["x"]))
            qy.append(float(row["y"]))
            qz.append(float(row["z"]))
            qw.append(float(row["w"]))
    return t_vals, qx, qy, qz, qw


def scale_series(values: List[float], min_val: float, max_val: float, height: float) -> List[float]:
    span = max_val - min_val if max_val != min_val else 1.0
    return [height - (v - min_val) / span * height for v in values]


def scale_time(times: List[float], width: float) -> List[float]:
    t_min, t_max = min(times), max(times)
    span = t_max - t_min if t_max != t_min else 1.0
    return [(t - t_min) / span * width for t in times]


def polyline(points: List[Tuple[float, float]], color: str, stroke_width: float = 1.5) -> str:
    coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline fill="none" stroke="{color}" stroke-width="{stroke_width}" points="{coords}" />'


def render_axes(width: float, height: float, margin: float) -> str:
    return (
        f'<rect x="{margin}" y="{margin}" width="{width}" height="{height}" '
        'fill="none" stroke="#444" stroke-width="1" />'
    )


def render_combined_svg(output_path: Path, t_vals, components, labels, colors):
    width, height, margin = 800, 400, 40
    plot_width, plot_height = width - 2 * margin, height - 2 * margin

    all_values = [v for series in components for v in series]
    y_min, y_max = min(all_values), max(all_values)
    x_scaled = scale_time(t_vals, plot_width)

    lines = []
    for series, color in zip(components, colors):
        y_scaled = scale_series(series, y_min, y_max, plot_height)
        points = [(margin + x, margin + y) for x, y in zip(x_scaled, y_scaled)]
        lines.append(polyline(points, color))

    legend_items = []
    legend_x, legend_y = width - 180, margin + 10
    for i, (label, color) in enumerate(zip(labels, colors)):
        y = legend_y + i * 18
        legend_items.append(
            f'<rect x="{legend_x}" y="{y}" width="10" height="10" fill="{color}" />'
        )
        legend_items.append(
            f'<text x="{legend_x + 16}" y="{y + 9}" font-size="12" fill="#222">{label}</text>'
        )

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white" />',
        render_axes(plot_width, plot_height, margin),
        *lines,
        *legend_items,
        f'<text x="{margin}" y="{margin - 10}" font-size="14" fill="#222">Quaternion components</text>',
        '</svg>',
    ]

    output_path.write_text("\n".join(svg), encoding="utf-8")


def render_subplots_svg(output_path: Path, t_vals, components, labels, colors):
    width, height, margin = 800, 700, 40
    subplot_height = (height - 2 * margin) / 4.0
    plot_width = width - 2 * margin

    x_scaled = scale_time(t_vals, plot_width)

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white" />',
        f'<text x="{margin}" y="{margin - 10}" font-size="14" fill="#222">Quaternion components (subplots)</text>',
    ]

    for idx, (series, label, color) in enumerate(zip(components, labels, colors)):
        y_min, y_max = min(series), max(series)
        y_scaled = scale_series(series, y_min, y_max, subplot_height - 20)
        offset_y = margin + idx * subplot_height
        points = [(margin + x, offset_y + 10 + y) for x, y in zip(x_scaled, y_scaled)]
        svg_parts.append(
            polyline(points, color, stroke_width=1.3)
        )
        svg_parts.append(
            f'<text x="{margin + 5}" y="{offset_y + 15}" font-size="12" fill="#222">{label}</text>'
        )
        svg_parts.append(
            f'<rect x="{margin}" y="{offset_y + 10}" width="{plot_width}" height="{subplot_height - 20}" '
            'fill="none" stroke="#444" stroke-width="1" />'
        )

    svg_parts.append('</svg>')
    output_path.write_text("\n".join(svg_parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Plot quaternion components from df_quaternion.csv")
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("documents/solutions/df_quaternion.csv"),
        help="Path to df_quaternion.csv",
    )
    parser.add_argument(
        "--output-combined",
        type=Path,
        default=Path("documents/solutions/df_quaternion_plot.svg"),
        help="Output combined SVG plot path",
    )
    parser.add_argument(
        "--output-subplots",
        type=Path,
        default=Path("documents/solutions/df_quaternion_subplots.svg"),
        help="Output subplot SVG plot path",
    )
    args = parser.parse_args()

    t_vals, qx, qy, qz, qw = load_csv(args.input)

    components = [qx, qy, qz, qw]
    labels = ["q_x", "q_y", "q_z", "q_w"]
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]

    args.output_combined.parent.mkdir(parents=True, exist_ok=True)
    render_combined_svg(args.output_combined, t_vals, components, labels, colors)
    render_subplots_svg(args.output_subplots, t_vals, components, labels, colors)


if __name__ == "__main__":
    main()
