#!/usr/bin/env python3
# Convert OBJ meshes to dense binary SDF grids (.sdf) using computeDistanceField
# - Normalizes each OBJ (optional)
# - Calls the isosurface tool to write <name>.sdf (dense grid with header)
#
# Requirements: pip install trimesh numpy
# Prereq binary: computeDistanceField (and its runtime libs)
#
# Example:
#   python obj2sdf.py --sdfgen /path/to/computeDistanceField \
#       --in my_parts/*.obj --out-dir sdf_out --res 256 --expand 1.3
#
# Output:
#   sdf_out/<stem>/isosurf.sdf   (dense SDF with (res+1)^3 float32 samples)

import argparse, os, sys, shutil, subprocess, tempfile
from pathlib import Path
import numpy as np
import trimesh

def as_mesh(scene_or_mesh):
    """Collapse a Trimesh Scene to a single mesh (drop materials)."""
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            return None
        return trimesh.util.concatenate(
            tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                  for g in scene_or_mesh.geometry.values()))
    assert isinstance(scene_or_mesh, trimesh.Trimesh)
    return trimesh.Trimesh(vertices=scene_or_mesh.vertices, faces=scene_or_mesh.faces)

def normalize_mesh_to_unit_sphere(obj_path: Path, out_dir: Path):
    """
    Sample the surface to get centroid & scale (max radius), apply to vertices,
    and write a normalized OBJ. Returns (normalized_obj_path, centroid(3), scale_m)
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    mesh_in = trimesh.load_mesh(str(obj_path), process=False)
    mesh = as_mesh(mesh_in)
    if mesh is None or len(mesh.vertices) == 0 or len(mesh.faces) == 0:
        raise RuntimeError(f"Mesh is empty: {obj_path}")

    # Surface-based centroid/scale (more robust than vertex-only)
    total = 16384
    pts, _ = trimesh.sample.sample_surface(mesh, total)
    centroid = pts.mean(axis=0)
    pts_centered = pts - centroid 
    m = np.linalg.norm(pts_centered, axis=1).max()  # radius

    # Apply transform: (v - centroid)/m
    mesh_norm = mesh.copy()
    mesh_norm.vertices = (mesh_norm.vertices - centroid) / float(m)

    norm_obj = out_dir / "pc_norm.obj"
    mesh_norm.export(str(norm_obj))
    return norm_obj, centroid.astype(np.float32), float(m)

def run_sdfgen(sdfgen_path: Path, norm_obj: Path, res: int, expand: float, out_sdf: Path, tmp_tag: str = "tmp"):
    """
    Call computeDistanceField:
      computeDistanceField <obj> <res> <res> <res> -s -e <expand> -o <tag>.dist -m 1 -c
    Then move <tag>.dist to out_sdf.
    """
    work_dir = out_sdf.parent
    work_dir.mkdir(parents=True, exist_ok=True)
    tmp_dist = work_dir / f"{tmp_tag}.dist"

    cmd = [
        str(sdfgen_path),
        str(norm_obj),
        str(res), str(res), str(res),
        "-s",
        "-e", str(expand),
        "-o", str(tmp_dist.with_suffix("").name),  # without .dist
        "-m", "1",
        "-c",
    ]

    print(f"[SDFGen] {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True, cwd=str(work_dir))
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"SDF generation failed (code {e.returncode}). Command: {' '.join(cmd)}") from e

    # The tool writes <tag>.dist in work_dir; move/rename to .sdf
    if not tmp_dist.exists():
        # Some builds write literally "<tag>.dist"; others write "<tag>"
        alt = work_dir / tmp_tag
        if alt.exists():
            alt.rename(tmp_dist)
        else:
            raise FileNotFoundError(f"Expected {tmp_dist} not found after SDFGen.")

    shutil.move(str(tmp_dist), str(out_sdf))
    print(f"[OK] wrote {out_sdf}")

def quick_read_sdf_header(sdf_path: Path):
    """
    Optional: read header to print grid size & bbox.
    Format (as in your code):
      - 3x int32 (grid info; first may be negative)
      - 6x float64 (xmin,ymin,zmin,xmax,ymax,zmax)
      - (res+1)^3 x float32 (data)
    """
    import struct
    with open(sdf_path, "rb") as f:
        header = f.read(3*4 + 6*8)
    a,b,c = struct.unpack("<iii", header[:12])
    xmin,ymin,zmin,xmax,ymax,zmax = struct.unpack("<6d", header[12:12+48])
    # Some exporters store (-res, res, res)
    res = abs(a)
    print(f"[SDF] grid: ({a},{b},{c}) -> using res={res}, bbox=({xmin:.4f},{ymin:.4f},{zmin:.4f})–({xmax:.4f},{ymax:.4f},{zmax:.4f})")
    return res, (xmin,ymin,zmin,xmax,ymax,zmax)

def process_one(obj_path: Path, out_dir: Path, sdfgen_path: Path, res: int, expand: float,
                keep_norm: bool, no_normalize: bool):
    stem = obj_path.stem
    target_dir = out_dir / stem
    target_dir.mkdir(parents=True, exist_ok=True)
    out_sdf = target_dir / "isosurf.sdf"

    if out_sdf.exists():
        print(f"[SKIP] {out_sdf} exists")
        return out_sdf

    if no_normalize:
        # Still write a copy (so we don't modify the original file)
        norm_dir = target_dir / "_norm"
        norm_dir.mkdir(exist_ok=True)
        norm_obj = norm_dir / "pc_norm.obj"
        shutil.copy2(str(obj_path), str(norm_obj))
        centroid = np.zeros(3, dtype=np.float32)
        m = 1.0
    else:
        norm_obj, centroid, m = normalize_mesh_to_unit_sphere(obj_path, target_dir / "_norm")

    # Tag for tmp .dist filename (avoid spaces)
    tmp_tag = "sdfgen"
    run_sdfgen(sdfgen_path, norm_obj, res, expand, out_sdf, tmp_tag=tmp_tag)

    # Optional: leave normalized OBJ (useful for reproducibility)
    if not keep_norm:
        try:
            shutil.rmtree(str(norm_obj.parent))
        except Exception:
            pass

    # Print a tiny header summary (handy to confirm)
    try:
        quick_read_sdf_header(out_sdf)
    except Exception as e:
        print(f"[WARN] couldn’t parse SDF header: {e}")

    # Also store normalization params if you want them later
    np.savez(target_dir / "norm_params.npz", centroid=centroid, scale=m)
    return out_sdf

def glob_inputs(patterns):
    files = []
    for p in patterns:
        pth = Path(p)
        if pth.is_dir():
            files.extend([f for f in pth.rglob("*.obj")])
        else:
            files.extend([f for f in map(Path, sorted([str(x) for x in Path().glob(p)]))])
    # Dedup and sort
    uniq = sorted(set(files))
    return uniq

def main():
    ap = argparse.ArgumentParser(description="Convert OBJ to binary SDF (.sdf) via computeDistanceField")
    ap.add_argument("--sdfgen", required=True, help="Path to computeDistanceField executable")
    ap.add_argument("--in_file", dest="inputs", nargs="+", required=True,
                    help="Input OBJ(s) or directory/glob (e.g., models/*.obj or models/)")
    ap.add_argument("--out-dir", default="sdf_out", help="Output directory (default: sdf_out)")
    ap.add_argument("--res", type=int, default=64, help="Grid resolution per axis for SDFGen (default: 256)")
    ap.add_argument("--expand", type=float, default=1.3,
                    help="Expand rate for SDF bbox relative to normalized mesh (default: 1.3)")
    ap.add_argument("--keep-norm-obj", action="store_true", help="Keep normalized OBJ copy")
    ap.add_argument("--no-normalize", action="store_true", help="Skip normalization; use OBJ as-is")
    args = ap.parse_args()

    sdfgen = Path(args.sdfgen)
    if not sdfgen.exists():
        print(f"ERROR: computeDistanceField not found at {sdfgen}")
        sys.exit(1)

    out_dir = Path(args.out_dir)
    objs = glob_inputs(args.inputs)
    if not objs:
        print("No OBJ files found.")
        sys.exit(1)

    print(f"Found {len(objs)} OBJ(s). Writing SDF to {out_dir}")
    for obj in objs:
        try:
            process_one(Path(obj), out_dir, sdfgen, args.res, args.expand,
                        keep_norm=args.keep_norm_obj, no_normalize=args.no_normalize)
        except Exception as e:
            print(f"[FAIL] {obj}: {e}")

if __name__ == "__main__":
    main()
