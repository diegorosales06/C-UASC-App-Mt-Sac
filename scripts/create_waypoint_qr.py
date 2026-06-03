#!/usr/bin/env python3
"""
Create a WAYPOINTS QR code for the VS Waypoint Mission tablet importer.



Setup:
    make sure to create a venv. Run whats below. You only need to do this once per environment.

    python3 -m venv venv
    source venv/bin/activate
    python3 -m pip install -r scripts/requirements-qr.txt

Run after editing WAYPOINTS below:
    python3 scripts/create_waypoint_qr.py --output waypoints_qr.png

Optional CSV input:
    python3 scripts/create_waypoint_qr.py --input scripts/my_waypoints.csv --output waypoints_qr.png

CSV rows can be:
    name,latitude,longitude,altitude_m
    latitude,longitude,altitude_m
    latitude,longitude
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


# Edit this list when you want to type coordinates directly into the script.
# Format: ("name", latitude, longitude, altitude_meters)
WAYPOINTS = [
    ("wp1", 34.048510, -117.837831, 10.0),
    ("wp2", 34.048414, -117.837468, 10.0),
]

DEFAULT_ALTITUDE_M = 10.0

Waypoint = Tuple[str, float, float, float]


def load_qr_library():
    try:
        import qrcode
        from qrcode.constants import ERROR_CORRECT_M
    except ModuleNotFoundError:
        requirements = Path(__file__).with_name("requirements-qr.txt")
        sys.exit(
            "Missing QR dependency.\n"
            "Install it with:\n"
            f"  python3 -m pip install -r {requirements}\n"
        )
    return qrcode, ERROR_CORRECT_M


def is_number(value: str) -> bool:
    try:
        float(value)
        return True
    except ValueError:
        return False


def validate_waypoint(name: str, latitude: float, longitude: float, altitude_m: float) -> Waypoint:
    if not name:
        raise ValueError("Waypoint name cannot be empty.")
    if latitude < -90 or latitude > 90:
        raise ValueError(f"{name}: latitude must be between -90 and 90.")
    if longitude < -180 or longitude > 180:
        raise ValueError(f"{name}: longitude must be between -180 and 180.")
    if altitude_m < 0:
        raise ValueError(f"{name}: altitude must be 0m or higher.")
    return name, latitude, longitude, altitude_m


def normalize_script_waypoints(raw_waypoints: Iterable[Sequence[object]]) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    for index, row in enumerate(raw_waypoints, start=1):
        if len(row) == 2:
            name = f"wp{index}"
            latitude, longitude = row
            altitude_m = DEFAULT_ALTITUDE_M
        elif len(row) == 3:
            if is_number(str(row[0])):
                name = f"wp{index}"
                latitude, longitude, altitude_m = row
            else:
                name, latitude, longitude = row
                altitude_m = DEFAULT_ALTITUDE_M
        elif len(row) >= 4:
            name, latitude, longitude, altitude_m = row[:4]
        else:
            raise ValueError(f"Waypoint row {index} is empty.")

        waypoints.append(
            validate_waypoint(str(name), float(latitude), float(longitude), float(altitude_m))
        )
    return waypoints


def load_waypoints_from_csv(csv_path: Path) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.reader(csv_file)
        for row_number, row in enumerate(reader, start=1):
            columns = [column.strip() for column in row]
            if not columns or not columns[0] or columns[0].startswith("#"):
                continue

            first = columns[0].lower()
            if row_number == 1 and first in {"name", "label", "lat", "latitude"}:
                continue

            try:
                if len(columns) == 2:
                    name = f"wp{len(waypoints) + 1}"
                    latitude = float(columns[0])
                    longitude = float(columns[1])
                    altitude_m = DEFAULT_ALTITUDE_M
                elif len(columns) == 3 and is_number(columns[0]):
                    name = f"wp{len(waypoints) + 1}"
                    latitude = float(columns[0])
                    longitude = float(columns[1])
                    altitude_m = float(columns[2])
                elif len(columns) >= 3:
                    name = columns[0]
                    latitude = float(columns[1])
                    longitude = float(columns[2])
                    altitude_m = float(columns[3]) if len(columns) >= 4 and columns[3] else DEFAULT_ALTITUDE_M
                else:
                    raise ValueError("not enough columns")
            except ValueError as exc:
                raise ValueError(f"{csv_path}:{row_number}: invalid waypoint row: {row}") from exc

            waypoints.append(validate_waypoint(name, latitude, longitude, altitude_m))

    return waypoints


def build_payload(waypoints: Sequence[Waypoint]) -> str:
    if not waypoints:
        raise ValueError("Add at least one waypoint before creating a QR code.")

    lines = ["WAYPOINTS"]
    for name, latitude, longitude, altitude_m in waypoints:
        lines.append(f"{name},{latitude:.7f},{longitude:.7f},{altitude_m:.1f}")
    return "\n".join(lines)


def create_qr_code(payload: str, output_path: Path) -> None:
    qrcode, error_correct_m = load_qr_library()
    qr = qrcode.QRCode(
        version=None,
        error_correction=error_correct_m,
        box_size=12,
        border=4,
    )
    qr.add_data(payload)
    qr.make(fit=True)
    image = qr.make_image(fill_color="black", back_color="white")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a QR code for importing waypoints into the VS Waypoint Mission screen."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("waypoints_qr.png"),
        help="PNG file to create. Default: waypoints_qr.png",
    )
    parser.add_argument(
        "-i",
        "--input",
        type=Path,
        help="Optional CSV file instead of the WAYPOINTS list in this script.",
    )
    parser.add_argument(
        "--payload-only",
        action="store_true",
        help="Print the QR text payload and do not create an image.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        waypoints = (
            load_waypoints_from_csv(args.input)
            if args.input
            else normalize_script_waypoints(WAYPOINTS)
        )
        payload = build_payload(waypoints)

        print("QR payload:")
        print(payload)

        if args.payload_only:
            return 0

        create_qr_code(payload, args.output)
        print(f"\nCreated QR code: {args.output.resolve()}")
        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
