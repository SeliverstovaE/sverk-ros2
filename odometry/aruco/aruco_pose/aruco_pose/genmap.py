import argparse
from pathlib import Path
from typing import Optional


def get_aruco_map_config_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore[import-not-found]
    except ImportError:
        return Path(".")

    share_dir = Path(get_package_share_directory("aruco_map"))
    return share_dir / "config"


def find_aruco_map_source_config_dir() -> Optional[Path]:
    """
    Try to find aruco_map/config in the current workspace to also write a copy
    next to the repository.

    Walk up from cwd and from this file until we find:
      sverk_ws/src/sverk_drone/odometry/aruco/aruco_map/config
    """
    rel = Path("sverk_ws") / "src" / "sverk_drone" / "odometry" / "aruco" / "aruco_map" / "config"

    candidates = []
    try:
        candidates.append(Path.cwd())
    except Exception:
        pass

    candidates.append(Path(__file__).resolve())

    for start in candidates:
        p = start if start.is_dir() else start.parent
        for parent in [p, *p.parents]:
            candidate = parent / rel
            if candidate.is_dir():
                return candidate

    # Common path inside project container
    fallback = Path("/home/sverk/sverk_ws/src/sverk_drone/odometry/aruco/aruco_map/config")
    if fallback.is_dir():
        return fallback

    return None


def default_output_path() -> Path:
    """
    Default map file path:
    <share/aruco_map>/config/generated_map.txt

    Package share is resolved via ament_index_python.
    """
    config_dir = get_aruco_map_config_dir()
    return config_dir / "generated_map.txt"


def resolve_output_path(output_arg: Optional[str]) -> Path:
    """
    If -o/--output is omitted — write to the default file under config/.
    If a relative path or bare filename (e.g. new_map.txt) —
    save to <share/aruco_map>/config/<name>.
    If absolute — use as-is.
    """
    if not output_arg:
        return default_output_path()

    p = Path(output_arg)
    if p.is_absolute():
        out = p
    else:
        # Allow filename only (no directories) → always under aruco_map config/
        name = p.name
        out = get_aruco_map_config_dir() / name

    if out.suffix == "":
        out = out.with_suffix(".txt")

    return out


def generate_map(length, nx, ny, dist_x, dist_y, first_id, bottom_left, output_path: Path):
    """
    Generate a text ArUco marker map compatible with aruco_map.

    Line format:
    id  length  x  y  z  rot_z  rot_y  rot_x
    """
    lines = []
    lines.append("# id    length  x       y       z       rot_z   rot_y   rot_x")

    current_id = first_id

    # X: left to right (i = 0..nx-1).
    # Y:
    #   bottom_left=True  -> bottom to top (j = 0..ny-1)
    #   bottom_left=False -> top to bottom (j = ny-1..0)
    if bottom_left:
        y_indices = range(0, ny)
    else:
        y_indices = range(ny - 1, -1, -1)

    for j in y_indices:
        for i in range(nx):
            x = i * dist_x
            y = j * dist_y
            z = 0.0

            rot_z = 0.0
            rot_y = 0.0
            rot_x = 0.0

            line = f"{current_id:d}  {length:.3f}  {x:.3f}  {y:.3f}  {z:.3f}  {rot_z:.3f}  {rot_y:.3f}  {rot_x:.3f}"
            lines.append(line)
            current_id += 1

    content = "\n".join(lines) + "\n"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(content, encoding="utf-8")

    # Duplicate under source tree if workspace config was found
    src_cfg = find_aruco_map_source_config_dir()
    if src_cfg is not None:
        duplicate_path = src_cfg / output_path.name
        try:
            duplicate_path.parent.mkdir(parents=True, exist_ok=True)
            duplicate_path.write_text(content, encoding="utf-8")
        except OSError:
            pass


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a text ArUco marker map (aruco_map compatible).\n\n"
            "Example:\n"
            "  ros2 run aruco_pose genmap.py 0.33 2 4 1 1 0 --bottom-left -o new_map.txt"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )

    parser.add_argument("length", type=float, help="Marker size (meters).")
    parser.add_argument("x", type=int, help="Number of markers along X.")
    parser.add_argument("y", type=int, help="Number of markers along Y.")
    parser.add_argument("dist_x", type=float, help="Center-to-center spacing along X (m).")
    parser.add_argument("dist_y", type=float, help="Center-to-center spacing along Y (m).")
    parser.add_argument("first", type=int, help="ID of the first marker.")

    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help=(
            "Output map filename (e.g. new_map.txt). "
            "Relative name → <share/aruco_map>/config/<name>. "
            "Omitted → <share/aruco_map>/config/generated_map.txt. "
            "Absolute paths allowed."
        ),
    )

    parser.add_argument(
        "--bottom-left",
        action="store_true",
        help="Number markers from bottom-left (default: top-left).",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    output_path = resolve_output_path(args.output)
    generate_map(
        length=args.length,
        nx=args.x,
        ny=args.y,
        dist_x=args.dist_x,
        dist_y=args.dist_y,
        first_id=args.first,
        bottom_left=args.bottom_left,
        output_path=output_path,
    )

    print(f"Generated {args.x}x{args.y} marker map.")
    print(f"Marker size: {args.length} m, step X: {args.dist_x} m, Y: {args.dist_y} m.")
    print(f"First ID: {args.first}, last ID: {args.first + args.x * args.y - 1}.")
    print(f"File: {output_path}")


if __name__ == "__main__":
    main()
