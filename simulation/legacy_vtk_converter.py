import argparse
import numpy as np
import meshio
import sys

def write_legacy_vtk_tetra(out_path, points, tets, title="mesh"):
    """
    Write a strict VTK Legacy 2.0 UNSTRUCTURED_GRID with tetrahedra only.
    points: (N,3) float
    tets:   (M,4) int (0-based)
    """
    points = np.asarray(points, dtype=float)
    tets   = np.asarray(tets,   dtype=int)

    npts = points.shape[0]
    ntet = tets.shape[0]
    # CELLS line expects total ints = ntet * (1 + 4) = ntet*5
    cells_size = ntet * 5

    with open(out_path, "w") as f:
        f.write("# vtk DataFile Version 2.0\n")
        f.write(f"{title}\n")
        f.write("ASCII\n")
        f.write("DATASET UNSTRUCTURED_GRID\n")

        # POINTS
        f.write(f"POINTS {npts} double\n")
        # write 3 doubles per line; VTK is fine with space-separated floats
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")

        # CELLS (tetra = 4 indices each)
        f.write(f"CELLS {ntet} {cells_size}\n")
        for c in tets:
            f.write(f"4 {c[0]} {c[1]} {c[2]} {c[3]}\n")

        # CELL_TYPES (10 = tetra)
        f.write(f"CELL_TYPES {ntet}\n")
        for _ in range(ntet):
            f.write("10\n")

    print(f"[OK] wrote legacy VTK 2.0: {out_path}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in",  dest="inp", required=True,  help="input .vtu/.vtk/.msh/.mesh etc.")
    ap.add_argument("--out", dest="out", required=True,  help="output .vtk (legacy 2.0)")
    ap.add_argument("--title", default="mesh", help="title string in the VTK header")
    args = ap.parse_args()

    m = meshio.read(args.inp)

    # Find tetra connectivity
    if "tetra" not in m.cells_dict or len(m.cells_dict["tetra"]) == 0:
        print("[ERROR] input has no tetra cells; found:", list(m.cells_dict.keys()))
        sys.exit(1)

    pts  = m.points
    tets = m.cells_dict["tetra"]

    write_legacy_vtk_tetra(args.out, pts, tets, title=args.title)

if __name__ == "__main__":
    main()