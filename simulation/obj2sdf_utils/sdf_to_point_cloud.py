#!/usr/bin/env python3
import argparse, numpy as np
from pathlib import Path

def load_sdf(path):
    """
    Layout assumed:
      - 3x int32: grid info (may include negatives; take abs)
      - 6x float64: (xmin, ymin, zmin, xmax, ymax, zmax)
      - (nx+1)*(ny+1)*(nz+1) x float32: SDF samples (x-fastest)
    Returns (grid[z,y,x], xs, ys, zs)
    """
    with open(path, "rb") as f:
        dims = np.fromfile(f, dtype="<i4", count=3)
        nx, ny, nz = map(int, map(abs, dims))
        bbox = np.fromfile(f, dtype="<f8", count=6)
        data = np.fromfile(f, dtype="<f4")

    nxp, nyp, nzp = nx + 1, ny + 1, nz + 1
    expected = nxp * nyp * nzp
    if data.size != expected:
        raise ValueError(f"Data size mismatch: have {data.size}, expected {expected}")

    # Most tools write x-fastest; reshape to (z,y,x) for natural indexing [z,y,x]
    grid = data.reshape((nzp, nyp, nxp))  # if axes look swapped, try (nxp, nyp, nzp).transpose(2,1,0)

    xmin, ymin, zmin, xmax, ymax, zmax = bbox
    xs = np.linspace(xmin, xmax, nxp)
    ys = np.linspace(ymin, ymax, nyp)
    zs = np.linspace(zmin, zmax, nzp)
    return grid, xs, ys, zs

def sdf_to_points(grid, xs, ys, zs, band=None, max_points=None):
    """
    Extract a thin point cloud near the zero level set (|SDF| <= band).
    'band' is in world units. Default ~ half the smallest voxel step.
    """
    dx = xs[1] - xs[0] if xs.size > 1 else 1.0
    dy = ys[1] - ys[0] if ys.size > 1 else 1.0
    dz = zs[1] - zs[0] if zs.size > 1 else 1.0
    if band is None:
        band = 0.5 * min(dx, dy, dz)

    mask = grid <= 0 
    if not np.any(mask):
        return np.empty((0, 3)), np.empty((0, 3))

    iz, iy, ix = np.nonzero(mask)
    pts = np.column_stack((xs[ix], ys[iy], zs[iz]))

    # Simple color map: red inside (neg), blue outside (pos)
    vals = grid[mask]
    t = np.clip((vals + band) / (2 * band), 0, 1)  # 0..1 across the band
    colors = np.column_stack((t, np.full_like(t, 0.2), 1.0 - t))

    if max_points is not None and pts.shape[0] > max_points:
        sel = np.random.choice(pts.shape[0], max_points, replace=False)
        pts, colors = pts[sel], colors[sel]

    return pts, colors

def sdf_to_points_surface(grid, xs, ys, zs, band=None, max_points=None):
    dx = xs[1]-xs[0] if xs.size>1 else 1.0
    dy = ys[1]-ys[0] if ys.size>1 else 1.0
    dz = zs[1]-zs[0] if zs.size>1 else 1.0
    if band is None:
        band = 0.3 * min(dx, dy, dz)  # thin shell ≈ surface

    # surf = np.abs(grid) <= band      # << surface-only, shows inner + outer walls
    surf = grid <= 0
    cavity_wall = surf & (grid >= 0)   # air-side of the surface only
    surf = cavity_wall  
    if not np.any(surf):
        return np.empty((0,3)), np.empty((0,3))

    iz, iy, ix = np.nonzero(surf)    # grid is (Z,Y,X)
    pts = np.column_stack((xs[ix], ys[iy], zs[iz]))

    vals = grid[surf]
    # color by sign: inside-neg (solid side) vs outside-pos (air side)
    t = np.clip((vals + band)/(2*band), 0, 1)
    colors = np.column_stack((t, np.full_like(t, 0.2), 1.0 - t))

    if max_points is not None and pts.shape[0] > max_points:
        sel = np.random.choice(pts.shape[0], max_points, replace=False)
        pts, colors = pts[sel], colors[sel]
    return pts, colors

def save_ply(points, colors, out_path, ascii=False):
    """
    Write a colored PLY (float positions, uint8 colors).
    """
    points = np.asarray(points, dtype=np.float32)
    colors = np.clip(np.asarray(colors), 0, 1)
    colors_u8 = (colors * 255).astype(np.uint8)

    # Build minimal PLY
    header = [
        "ply",
        f"format {'ascii 1.0' if ascii else 'binary_little_endian 1.0'}",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header\n",
    ]
    with open(out_path, "wb") as f:
        f.write("\n".join(header).encode("ascii"))
        if ascii:
            for p, c in zip(points, colors_u8):
                f.write(f"{p[0]} {p[1]} {p[2]} {c[0]} {c[1]} {c[2]}\n".encode("ascii"))
        else:
            rec = np.zeros(points.shape[0], dtype=[("x","<f4"),("y","<f4"),("z","<f4"),
                                                   ("r","u1"),("g","u1"),("b","u1")])
            rec["x"], rec["y"], rec["z"] = points[:,0], points[:,1], points[:,2]
            rec["r"], rec["g"], rec["b"] = colors_u8[:,0], colors_u8[:,1], colors_u8[:,2]
            rec.tofile(f)

def main():
    ap = argparse.ArgumentParser(description="Visualize SDF as a point cloud near the zero isosurface")
    ap.add_argument("sdf", type=Path, help="Input .sdf file")
    ap.add_argument("--out", type=Path, default=None, help="Output .ply (if omitted, just prints stats)")
    ap.add_argument("--band", type=float, default=None, help="Band around zero in world units (default ~ 0.5 * voxel)")
    ap.add_argument("--max-points", type=int, default=2_000_000, help="Downsample cap for huge grids")
    ap.add_argument("--ascii", action="store_true", help="Write ASCII PLY (larger, but human-readable)")
    args = ap.parse_args()

    grid, xs, ys, zs = load_sdf(args.sdf)
    pts, cols = sdf_to_points_surface(grid, xs, ys, zs, band=args.band, max_points=args.max_points)
    print(f"Selected {len(pts):,} points.")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        save_ply(pts, cols, args.out, ascii=args.ascii)
        print(f"Wrote {args.out}")

if __name__ == "__main__":
    main()
