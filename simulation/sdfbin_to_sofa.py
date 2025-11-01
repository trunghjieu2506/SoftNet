#!/usr/bin/env python3
# Read a binary .sdf (computeDistanceField format) and call export_sdf_volume_to_sofa

import struct, argparse, numpy as np
from pathlib import Path
from process_sofa_input import export_sdf_volume_to_sofa  # <-- from the exporter we wrote earlier

def read_sdf_bin(path: str):
    with open(path, "rb") as f:
        hdr = f.read(3*4 + 6*8)  # 3 int32 + 6 float64
        a,b,c = struct.unpack("<iii", hdr[:12])
        xmin,ymin,zmin,xmax,ymax,zmax = struct.unpack("<6d", hdr[12:12+48])
        # Many builds store (-res, res, res). Use abs for resolution on each axis.
        Rx = abs(a); Ry = abs(b); Rz = abs(c)
        # Data is stored as (R+1)^3 float32, ordered as (Z,Y,X) in this repo
        n = (Rx+1)*(Ry+1)*(Rz+1)
        data = np.frombuffer(f.read(n*4), dtype=np.float32)
    V_zyx = data.reshape((Rz+1, Ry+1, Rx+1))     # (Z,Y,X)
    V_xyz = np.transpose(V_zyx, (2,1,0)).astype(np.float32, copy=False)  # -> (X,Y,Z)

    # spacing/origin in world units (whatever SDFGen used)
    nx, ny, nz = V_xyz.shape
    dx = (xmax - xmin) / (nx - 1)
    dy = (ymax - ymin) / (ny - 1)
    dz = (zmax - zmin) / (nz - 1)
    cal_band = find_cal_band(path)  # for export_sdf_volume_to_sofa
    origin = (xmin, ymin, zmin)
    voxel_size = (dx, dy, dz)
    return V_xyz, voxel_size, origin, cal_band

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdf", required=True, help="path to isosurf.sdf")
    ap.add_argument("--out-dir", default="sofa_assets", help="output folder for VTU/STLs")
    ap.add_argument("--invert-sign", action="store_true", help="flip sign if inside is positive")
    ap.add_argument("--tet-size", type=float, default=0.02, help="target tet size (m) via max_cell_circumradius")
    ap.add_argument("--facet-tol", type=float, default=0.004, help="surface tolerance (m)")
    args = ap.parse_args()

    V, vox, org, cal_band = read_sdf_bin(args.sdf)
    base = min(vox)            # current physical voxel step
    max_facet_distance = 1.5 * base   # surface tolerance (looser -> fewer tris)
    max_cell_circumradius = 3.0 * base  # larger cells -> fewer tets

    export_sdf_volume_to_sofa(
        V,
        cal_band,
        voxel_size=vox,
        origin=org,
        out_dir="simulation/out_dir",
        invert_sign=False,
        max_cell_circumradius=max_cell_circumradius,
        max_facet_distance=max_facet_distance
    )

if __name__ == "__main__":
    main()
