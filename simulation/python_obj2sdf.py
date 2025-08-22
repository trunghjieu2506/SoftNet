#!/usr/bin/env python3
# Create a dense binary SDF (.sdf) from an OBJ on macOS using pure Python.
# Output format matches your get_sdf(): header {int32[-R,R,R], float64[6] bbox} + (R+1)^3 float32 values
# Dependencies: pip install trimesh numpy

import argparse, struct, sys
from pathlib import Path
import numpy as np, trimesh

def as_mesh(m):
    if isinstance(m, trimesh.Scene):
        if len(m.geometry) == 0: raise RuntimeError("Empty scene")
        return trimesh.util.concatenate(tuple(trimesh.Trimesh(g.vertices, g.faces) for g in m.geometry.values()))
    return trimesh.Trimesh(m.vertices, m.faces)

def normalize_mesh(mesh: trimesh.Trimesh):
    pts, _ = trimesh.sample.sample_surface(mesh, 16384)
    c = pts.mean(axis=0).astype(np.float32)
    r = np.linalg.norm(pts - c, axis=1).max()
    mesh = mesh.copy()
    mesh.vertices = (mesh.vertices - c) / float(r)
    return mesh, c, float(r)

def signed_distance_grid(mesh: trimesh.Trimesh, res: int, expand: float):
    # Build bbox in world coords
    bmin, bmax = mesh.bounds
    center = (bmin + bmax) * 0.5
    half = (bmax - bmin) * 0.5
    # Expand uniformly
    half = np.max(half) * expand
    xmin, ymin, zmin = center - half
    xmax, ymax, zmax = center + half

    # Sample grid coordinates (R+1 points per axis)
    R = res
    xs = np.linspace(xmin, xmax, R+1, dtype=np.float64)
    ys = np.linspace(ymin, ymax, R+1, dtype=np.float64)
    zs = np.linspace(zmin, zmax, R+1, dtype=np.float64)

    # Evaluate signed distances in chunks to keep memory sane
    pq = trimesh.proximity.ProximityQuery(mesh)
    def sd(pts):
        # signed_distance is negative inside, positive outside (needs watertight mesh for reliable sign)
        return pq.signed_distance(pts)

    Z, Y, X = np.meshgrid(zs, ys, xs, indexing='ij')  # shapes: (R+1)^3
    pts = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

    out = np.empty((pts.shape[0],), dtype=np.float32)
    CHUNK = 400000  # tune for your RAM
    for i in range(0, pts.shape[0], CHUNK):
        out[i:i+CHUNK] = sd(pts[i:i+CHUNK]).astype(np.float32)

    # Reshape to (Z,Y,X) so it matches your reader's expectation
    phi_zyx = out.reshape((R+1, R+1, R+1))
    bbox = (xmin, ymin, zmin, xmax, ymax, zmax)
    return phi_zyx, bbox

def write_sdf_binary(path: Path, phi_zyx: np.ndarray, bbox):
    Rz, Ry, Rx = phi_zyx.shape  # each == R+1
    R = Rx - 1
    # Header: int32[-R, R, R] + float64 bbox + float32 data (Z,Y,X) row-major
    with open(path, 'wb') as f:
        f.write(struct.pack('<iii', -R, R, R))
        f.write(struct.pack('<6d', *bbox))
        phi_zyx.astype(np.float32, copy=False).tofile(f)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', required=True, help='input OBJ')
    ap.add_argument('--out', dest='out', default='isosurf.sdf', help='output .sdf')
    ap.add_argument('--res', type=int, default=128, help='grid resolution per axis (default 128)')
    ap.add_argument('--expand', type=float, default=1.3, help='bbox expand factor (default 1.3)')
    ap.add_argument('--no-normalize', action='store_true', help='skip unit-sphere normalization')
    args = ap.parse_args()

    mesh = trimesh.load_mesh(args.inp, process=False)
    mesh = as_mesh(mesh)

    if not args.no_normalize:
        mesh, c, s = normalize_mesh(mesh)
        print(f"[normalize] centroid={c.tolist()} scale={s:.6f}")

    phi_zyx, bbox = signed_distance_grid(mesh, args.res, args.expand)
    write_sdf_binary(Path(args.out), phi_zyx, bbox)
    print(f"[OK] wrote {args.out} with res={args.res}, bbox={bbox}")

if __name__ == '__main__':
    # Speed tip: pip install pyembree for faster distances (optional)
    # pip install pyembree
    main()
